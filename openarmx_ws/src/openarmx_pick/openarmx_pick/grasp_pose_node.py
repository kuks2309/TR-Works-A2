#!/usr/bin/env python3
"""Stage B: box-top detection -> top-down 6-DoF grasp pose -> (optional) MoveL.

Bridges the existing ``box_plane`` vision output to the cyclo_control QP MoveL
solver. It consumes the box-top inlier cloud, expresses it in the robot base
frame (``openarmx_body_link0``), and synthesises a classic top-down grasp:

  * grasp point  = box-top centroid (optionally pushed down by ``grasp_depth``)
  * approach     = straight down (gripper tool axis aligned with -z_base, since
                   the box top normal is +z_base for a box resting on a table)
  * yaw          = principal axis of the box top (PCA in the base XY plane) so
                   the gripper opening straddles the box's short dimension

No learned grasp network is needed: a box on a table is fully constrained by its
top plane + footprint. Outputs a ``geometry_msgs/PoseStamped`` grasp pose and an
RViz marker; when ``auto_send`` is set it also emits a pre-grasp ``MoveL`` so the
arm hovers above the box (the descend/close/lift FSM is a later step).

Tool-frame convention: the controlled link ``openarmx_left_hand_tcp`` is assumed
to grasp along its local +z (approach) with the fingers opening along local +x.
``tool_approach_axis`` / ``tool_opening_axis`` make this explicit and tunable.
"""
import json
import math
import struct

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros

try:
    from robotis_interfaces.msg import MoveL
    _HAVE_MOVEL = True
except Exception:  # MoveL only needed when auto_send=True
    _HAVE_MOVEL = False


# ----------------------------- geometry helpers -----------------------------
def _read_xyz(cloud: PointCloud2, stride: int = 1) -> np.ndarray:
    """Extract Nx3 finite XYZ from a PointCloud2 (float32 x,y,z at field offsets)."""
    off = {f.name: f.offset for f in cloud.fields}
    if not all(k in off for k in ("x", "y", "z")):
        return np.empty((0, 3))
    ox, oy, oz = off["x"], off["y"], off["z"]
    step, data = cloud.point_step, cloud.data
    n = cloud.width * cloud.height
    out = []
    for i in range(0, n, stride):
        base = i * step
        x = struct.unpack_from("<f", data, base + ox)[0]
        y = struct.unpack_from("<f", data, base + oy)[0]
        z = struct.unpack_from("<f", data, base + oz)[0]
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            out.append((x, y, z))
    return np.asarray(out, dtype=np.float64) if out else np.empty((0, 3))


def _quat_from_matrix(m: np.ndarray) -> np.ndarray:
    """Rotation matrix -> quaternion [x, y, z, w]."""
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w, x, y, z = 0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w, x, y, z = (m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w, x, y, z = (m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w, x, y, z = (m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s
    return np.array([x, y, z, w])


def _grasp_rotation(approach: np.ndarray, opening: np.ndarray,
                    tool_approach: np.ndarray, tool_opening: np.ndarray) -> np.ndarray:
    """R_base_tool mapping the tool approach/opening axes onto the desired
    base-frame approach/opening directions (right-handed, orthonormal)."""
    a = approach / np.linalg.norm(approach)
    o = opening - np.dot(opening, a) * a
    if np.linalg.norm(o) < 1e-6:
        o = np.array([1.0, 0.0, 0.0]) - np.dot([1.0, 0.0, 0.0], a) * a
    o /= np.linalg.norm(o)
    B = np.column_stack((o, np.cross(a, o), a))      # desired: opening, binormal, approach
    ta = tool_approach / np.linalg.norm(tool_approach)
    to = tool_opening - np.dot(tool_opening, ta) * ta
    to /= np.linalg.norm(to)
    T = np.column_stack((to, np.cross(ta, to), ta))
    return B @ T.T


# --------------------------------- node ------------------------------------
class GraspPoseNode(Node):
    def __init__(self):
        super().__init__("grasp_pose_node")
        self.declare_parameter("cloud_topic", "/box_plane/cloud")
        self.declare_parameter("info_topic", "/box_plane/info")
        self.declare_parameter("base_frame", "openarmx_body_link0")
        self.declare_parameter("movel_topic", "/openarmx/movel")
        self.declare_parameter("grasp_pose_topic", "/openarmx/grasp_pose")
        self.declare_parameter("pregrasp_height", 0.10)   # m above box top
        self.declare_parameter("grasp_depth", 0.005)      # m below top surface
        self.declare_parameter("cloud_stride", 4)         # subsample for PCA speed
        self.declare_parameter("auto_send", False)        # publish a pre-grasp MoveL
        self.declare_parameter("move_time", 4.0)          # s for the MoveL motion
        # Debounce so we do NOT re-send a MoveL every camera frame (which would
        # restart the solver's trajectory each cycle -> the arm only crawls).
        # Re-send only when the target shifts > send_min_delta OR after a
        # send_min_interval cooldown (>= move_time lets a motion finish first).
        self.declare_parameter("send_min_interval", 5.0)  # s cooldown between sends
        self.declare_parameter("send_min_delta", 0.02)    # m target shift to re-send
        self.declare_parameter("tool_approach_axis", [0.0, 0.0, 1.0])
        self.declare_parameter("tool_opening_axis", [1.0, 0.0, 0.0])

        gp = self.get_parameter
        self.base_frame = gp("base_frame").value
        self.pregrasp_h = float(gp("pregrasp_height").value)
        self.grasp_depth = float(gp("grasp_depth").value)
        self.stride = int(gp("cloud_stride").value)
        self.auto_send = bool(gp("auto_send").value)
        self.move_time = float(gp("move_time").value)
        self.send_min_interval = float(gp("send_min_interval").value)
        self.send_min_delta = float(gp("send_min_delta").value)
        self._last_sent_xyz = None
        self._last_sent_t = None
        self.tool_a = np.array(gp("tool_approach_axis").value, dtype=float)
        self.tool_o = np.array(gp("tool_opening_axis").value, dtype=float)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self._box_height = None

        latched = QoSProfile(depth=1,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pose_pub = self.create_publisher(PoseStamped, gp("grasp_pose_topic").value, latched)
        self.marker_pub = self.create_publisher(MarkerArray, "/openarmx/grasp_markers", latched)
        self.movel_pub = None
        if self.auto_send and _HAVE_MOVEL:
            self.movel_pub = self.create_publisher(MoveL, gp("movel_topic").value, 10)
        elif self.auto_send:
            self.get_logger().warn("auto_send set but robotis_interfaces/MoveL unavailable.")

        self.create_subscription(String, gp("info_topic").value, self._on_info, 10)
        self.create_subscription(PointCloud2, gp("cloud_topic").value, self._on_cloud, 5)
        self.get_logger().info(
            f"grasp_pose_node up: cloud='{gp('cloud_topic').value}' -> base='{self.base_frame}', "
            f"auto_send={self.auto_send}")

    def _on_info(self, msg: String):
        try:
            self._box_height = json.loads(msg.data).get("box_height_m")
        except Exception:
            pass

    def _on_cloud(self, cloud: PointCloud2):
        pts_cam = _read_xyz(cloud, self.stride)
        if pts_cam.shape[0] < 30:
            self.get_logger().warn(f"box-top cloud too small ({pts_cam.shape[0]} pts).",
                                   throttle_duration_sec=2.0)
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, cloud.header.frame_id, rclpy.time.Time())
        except Exception as exc:
            self.get_logger().warn(f"TF {self.base_frame}<-{cloud.header.frame_id} failed: {exc}",
                                   throttle_duration_sec=2.0)
            return
        R, t = self._tf_to_Rt(tf)
        pts = (R @ pts_cam.T).T + t                       # Nx3 in base frame

        centroid = pts.mean(axis=0)
        xy = pts[:, :2] - centroid[:2]
        cov = xy.T @ xy / max(len(xy) - 1, 1)
        evals, evecs = np.linalg.eigh(cov)
        long_axis = evecs[:, int(np.argmax(evals))]       # box long direction (XY)
        long_axis = np.array([long_axis[0], long_axis[1], 0.0])
        opening = np.cross(np.array([0.0, 0.0, 1.0]), long_axis)  # span the short axis
        opening /= np.linalg.norm(opening)

        approach = np.array([0.0, 0.0, -1.0])             # straight down
        R_bt = _grasp_rotation(approach, opening, self.tool_a, self.tool_o)
        quat = _quat_from_matrix(R_bt)

        grasp_xyz = centroid.copy(); grasp_xyz[2] -= self.grasp_depth
        pre_xyz = centroid.copy(); pre_xyz[2] += self.pregrasp_h

        self._publish_pose(grasp_xyz, quat, cloud.header.stamp)
        self._publish_marker(grasp_xyz, pre_xyz, cloud.header.stamp)
        if self.movel_pub is not None and self._should_send(pre_xyz):
            self._send_movel(pre_xyz, quat, cloud.header.stamp)

        self.get_logger().info(
            f"grasp @ ({grasp_xyz[0]:+.3f},{grasp_xyz[1]:+.3f},{grasp_xyz[2]:+.3f}) base, "
            f"{pts.shape[0]} pts, h={self._box_height}", throttle_duration_sec=1.0)

    @staticmethod
    def _tf_to_Rt(tf):
        q = tf.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        R = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])
        tr = tf.transform.translation
        return R, np.array([tr.x, tr.y, tr.z])

    def _publish_pose(self, xyz, quat, stamp):
        p = PoseStamped()
        p.header.frame_id = self.base_frame
        p.header.stamp = stamp
        p.pose.position.x, p.pose.position.y, p.pose.position.z = map(float, xyz)
        (p.pose.orientation.x, p.pose.orientation.y,
         p.pose.orientation.z, p.pose.orientation.w) = map(float, quat)
        self.pose_pub.publish(p)

    def _should_send(self, pre_xyz) -> bool:
        """Debounce: send a fresh MoveL only when the target moved enough or the
        cooldown elapsed, so we don't restart the solver trajectory every frame."""
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._last_sent_xyz is None:
            self._last_sent_xyz, self._last_sent_t = np.asarray(pre_xyz), now
            return True
        moved = float(np.linalg.norm(np.asarray(pre_xyz) - self._last_sent_xyz))
        elapsed = now - self._last_sent_t
        if moved > self.send_min_delta or elapsed > self.send_min_interval:
            self._last_sent_xyz, self._last_sent_t = np.asarray(pre_xyz), now
            return True
        return False

    def _send_movel(self, xyz, quat, stamp):
        m = MoveL()
        m.pose.header.frame_id = self.base_frame
        m.pose.header.stamp = stamp
        m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = map(float, xyz)
        (m.pose.pose.orientation.x, m.pose.pose.orientation.y,
         m.pose.pose.orientation.z, m.pose.pose.orientation.w) = map(float, quat)
        m.time_from_start.sec = int(self.move_time)
        m.time_from_start.nanosec = int((self.move_time % 1.0) * 1e9)
        self.movel_pub.publish(m)

    def _publish_marker(self, grasp_xyz, pre_xyz, stamp):
        a = Marker()
        a.header.frame_id = self.base_frame
        a.header.stamp = stamp
        a.ns = "grasp"; a.id = 0
        a.type = Marker.ARROW; a.action = Marker.ADD
        a.scale.x, a.scale.y, a.scale.z = 0.01, 0.02, 0.0
        a.color.r, a.color.g, a.color.b, a.color.a = 0.1, 1.0, 0.2, 0.9
        a.points = [Point(x=float(pre_xyz[0]), y=float(pre_xyz[1]), z=float(pre_xyz[2])),
                    Point(x=float(grasp_xyz[0]), y=float(grasp_xyz[1]), z=float(grasp_xyz[2]))]
        arr = MarkerArray(); arr.markers.append(a)
        self.marker_pub.publish(arr)


def main():
    rclpy.init()
    node = GraspPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
