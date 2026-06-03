#!/usr/bin/env python3
"""Bimanual box-align action server.

On an ``AlignToBoxes`` goal it:
  1. detecting  -- triggers the on-demand YOLOv8 ``DetectBox`` action over several
     frames, accumulates + clusters the detections in 3D, and computes each box's
     TOP-surface centroid in the robot base frame (publishes box_<i> TFs).
  2. assigning  -- sorts boxes left->right by base-frame Y and assigns +Y boxes to
     the LEFT arm, -Y boxes to the RIGHT arm (honouring goal.arms).
  3. moving     -- for each assigned arm, builds a link7 target at (box.x, box.y, z)
     with the commanded hand orientation (default R180 P0 Y0 = vertical-down),
     compensates the fixed link7->hand_tcp offset, and publishes a MoveL to the
     cyclo controller on /openarmx/<side>/movel.

Prerequisites (must already be running): the D435 camera, the YOLOv8 DetectBox
action server (/yolov8_node/detect), the calibrated base->camera TF, and the
cyclo MoveL controllers (/openarmx/{left,right}/movel).
"""
import json
import math
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener

from yolov8_detection_msgs.action import DetectBox
from openarmx_scenario_player_msgs.msg import MoveL
from openarmx_cyclo_box_align_msgs.action import AlignToBoxes

BASE = "openarmx_body_link0"
CAM = "camera_color_optical_frame"
DEFAULT_PROMPTS = "box, cube, block, colored cube, toy block, cardboard box, square box"


def rpy_to_quat(roll, pitch, yaw):
    """Roll/pitch/yaw in DEGREES -> quaternion [x, y, z, w] (ZYX intrinsic)."""
    r, p, y = math.radians(roll), math.radians(pitch), math.radians(yaw)
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return [sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy]


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def iou(a, b):
    ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
    ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / ua if ua > 0 else 0.0


class BoxAlignNode(Node):
    def __init__(self):
        super().__init__("box_align_node")
        self.declare_parameter("n_frames", 5)
        self.declare_parameter("cluster_radius", 0.08)
        self.declare_parameter("min_hits", 2)          # cluster must appear in >= this many frames
        self.declare_parameter("move_time", 6.0)
        self.declare_parameter("default_confidence", 0.02)
        self.declare_parameter("ws_x", [0.05, 0.70])
        self.declare_parameter("ws_y_abs", 0.45)
        self.declare_parameter("ws_z", [0.10, 0.32])
        gp = self.get_parameter
        self.n_frames = int(gp("n_frames").value)
        self.cluster_r = float(gp("cluster_radius").value)
        self.min_hits = int(gp("min_hits").value)
        self.move_time = float(gp("move_time").value)
        self.default_conf = float(gp("default_confidence").value)
        self.ws_x = [float(v) for v in gp("ws_x").value]
        self.ws_y = float(gp("ws_y_abs").value)
        self.ws_z = [float(v) for v in gp("ws_z").value]

        self.bridge = CvBridge()
        self.depth = None
        self.fx = self.fy = self.cx = self.cy = None
        cb = ReentrantCallbackGroup()
        self.create_subscription(
            Image, "/camera/camera/aligned_depth_to_color/image_raw",
            self._on_depth, qos_profile_sensor_data, callback_group=cb)
        self.create_subscription(
            CameraInfo, "/camera/camera/color/camera_info",
            self._on_ci, 10, callback_group=cb)
        self.br = StaticTransformBroadcaster(self)
        self._max_box = 0          # highest box_<i> count published (for stale-frame parking)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.movel_pub = {
            "left": self.create_publisher(MoveL, "/openarmx/left/movel", 10),
            "right": self.create_publisher(MoveL, "/openarmx/right/movel", 10),
        }
        self.detect_client = ActionClient(
            self, DetectBox, "/yolov8_node/detect", callback_group=cb)
        self.srv = ActionServer(
            self, AlignToBoxes, "/openarmx/align_to_boxes",
            execute_callback=self._execute,
            goal_callback=lambda _g: GoalResponse.ACCEPT,
            cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=cb)
        self.get_logger().info("box_align_node ready: action /openarmx/align_to_boxes")

    # --------------------------------------------------------------- sensors
    def _on_depth(self, m):
        self.depth = self.bridge.imgmsg_to_cv2(m, "passthrough")

    def _on_ci(self, m):
        if self.fx is None:
            k = m.k
            self.fx, self.fy, self.cx, self.cy = k[0], k[4], k[2], k[5]

    def _wait(self, future, timeout=90.0):
        t0 = time.time()
        while rclpy.ok() and not future.done() and time.time() - t0 < timeout:
            time.sleep(0.05)
        return future.done()

    def _tf(self, target, source, timeout=5.0):
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < timeout:
            try:
                return self.tf_buffer.lookup_transform(target, source, Time()).transform
            except Exception:
                time.sleep(0.1)
        return None

    # --------------------------------------------------------------- detect
    def _detect_once(self, prompts, conf):
        g = DetectBox.Goal()
        g.prompts = prompts
        g.confidence = conf
        g.publish_annotated = True
        gh_f = self.detect_client.send_goal_async(g)
        if not self._wait(gh_f):
            return []
        gh = gh_f.result()
        r_f = gh.get_result_async()
        if not self._wait(r_f):
            return []
        return json.loads(r_f.result().result.detections_json).get("detections", [])

    def _dedup(self, dets):
        dets = sorted(dets, key=lambda d: -d["confidence"])
        kept = []
        for d in dets:
            if all(iou(d["bbox_xyxy"], k["bbox_xyxy"]) < 0.6 for k in kept):
                kept.append(d)
        return kept

    def _point3d(self, bbox):
        """Box-top surface centroid (nearest-depth cluster in bbox), camera frame."""
        if self.depth is None or self.fx is None:
            return None
        x1, y1, x2, y2 = [int(round(v)) for v in bbox]
        H, W = self.depth.shape[:2]
        x1, y1, x2, y2 = max(0, x1), max(0, y1), min(W, x2), min(H, y2)
        roi = self.depth[y1:y2, x1:x2].astype(float) / 1000.0
        vv, uu = np.where(roi > 0)
        if uu.size < 20:
            return None
        zz = roi[vv, uu]
        z_near = np.percentile(zz, 5)
        top = zz < (z_near + 0.03)
        if top.sum() < 10:
            top = zz <= np.percentile(zz, 30)
        u, v, z = uu[top] + x1, vv[top] + y1, zz[top]
        X = (u - self.cx) * z / self.fx
        Y = (v - self.cy) * z / self.fy
        return [float(np.mean(X)), float(np.mean(Y)), float(np.mean(z))]

    def _cam_to_base(self, p):
        tf = self._tf(BASE, CAM)
        if tf is None:
            return None
        q, t = tf.rotation, tf.translation
        R = quat_to_R([q.x, q.y, q.z, q.w])
        return (R @ np.array(p) + np.array([t.x, t.y, t.z])).tolist()

    def _base_align_quat(self):
        tf = self._tf(CAM, BASE)
        if tf is None:
            return [0.0, 0.0, 0.0, 1.0]
        r = tf.rotation
        return [r.x, r.y, r.z, r.w]

    def detect_boxes(self, prompts, conf):
        """Returns list of dicts {cam:[x,y,z], base:[x,y,z]} for each box."""
        raw = []
        for _ in range(self.n_frames):
            for d in self._dedup([x for x in self._detect_once(prompts, conf)
                                  if x["confidence"] > 0.005]):
                p = self._point3d(d["bbox_xyxy"])
                if p:
                    raw.append(p)
        clusters = []
        for p in raw:
            for cl in clusters:
                m = cl["mean"]
                if sum((p[i] - m[i]) ** 2 for i in range(3)) < self.cluster_r ** 2:
                    cl["pts"].append(p)
                    cl["mean"] = [sum(c[i] for c in cl["pts"]) / len(cl["pts"]) for i in range(3)]
                    break
            else:
                clusters.append({"pts": [p], "mean": list(p)})
        boxes = []
        for cl in clusters:
            if len(cl["pts"]) < self.min_hits:        # drop one-off (noise) detections
                continue
            b = self._cam_to_base(cl["mean"])
            if b is None:
                continue
            if (self.ws_x[0] < b[0] < self.ws_x[1] and abs(b[1]) < self.ws_y
                    and self.ws_z[0] < b[2] < self.ws_z[1]):
                boxes.append({"cam": cl["mean"], "base": b, "hits": len(cl["pts"])})
        boxes.sort(key=lambda d: -d["base"][1])   # left (+Y) -> right
        return boxes

    def publish_box_tfs(self, boxes):
        """Publish box_<i> for the current detection. Frames from a previous, larger
        detection are 'parked' far below (z=-100 in camera frame) so stale axes from
        an earlier run disappear instead of lingering in RViz's static TF buffer."""
        q = self._base_align_quat()
        tfs = []
        for i, b in enumerate(boxes):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = CAM
            t.child_frame_id = f"box_{i}"
            c = b["cam"]
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, c)
            t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = map(float, q)
            tfs.append(t)
        for i in range(len(boxes), self._max_box):     # park stale frames from a prior run
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = CAM
            t.child_frame_id = f"box_{i}"
            t.transform.translation.z = -100.0
            t.transform.rotation.w = 1.0
            tfs.append(t)
        self._max_box = max(self._max_box, len(boxes))
        if tfs:
            self.br.sendTransform(tfs)

    # ----------------------------------------------------------------- move
    def move_arm(self, side, box_base, z, quat):
        """Command <side> arm so link7 reaches (box.x, box.y, z) with orientation
        quat. Compensates the fixed link7->hand_tcp offset (cyclo controls tcp)."""
        link7 = f"openarmx_{side}_link7"
        tcp = f"openarmx_{side}_hand_tcp"
        off = self._tf(link7, tcp)            # fixed link7 -> hand_tcp transform
        R = quat_to_R(quat)
        if off is not None:
            t_lt = np.array([off.translation.x, off.translation.y, off.translation.z])
        else:
            t_lt = np.array([0.0, 0.0, 0.180])  # measured fallback
        link7_target = np.array([box_base[0], box_base[1], z])
        tcp_pos = link7_target + R @ t_lt
        m = MoveL()
        m.pose.header.frame_id = BASE
        m.pose.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = map(float, tcp_pos)
        m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z, m.pose.pose.orientation.w = map(float, quat)
        m.time_from_start.sec = int(self.move_time)
        m.time_from_start.nanosec = int((self.move_time % 1.0) * 1e9)
        self.movel_pub[side].publish(m)
        return link7_target.tolist()

    def link7_pos(self, side):
        tf = self._tf(BASE, f"openarmx_{side}_link7", timeout=1.0)
        if tf is None:
            return None
        return [tf.translation.x, tf.translation.y, tf.translation.z]

    # -------------------------------------------------------------- execute
    def _fb(self, gh, phase, prog):
        fb = AlignToBoxes.Feedback()
        fb.phase = phase
        fb.progress = float(prog)
        gh.publish_feedback(fb)

    def _execute(self, gh):
        goal = gh.request
        res = AlignToBoxes.Result()
        prompts = goal.prompts.strip() if goal.prompts.strip() else DEFAULT_PROMPTS
        conf = goal.confidence if goal.confidence > 0.0 else self.default_conf
        quat = rpy_to_quat(goal.roll_deg, goal.pitch_deg, goal.yaw_deg)

        self._fb(gh, "detecting", 0.1)
        boxes = self.detect_boxes(prompts, conf)
        self.publish_box_tfs(boxes)
        res.detections_json = json.dumps([{"base": b["base"], "hits": b["hits"]} for b in boxes])
        if not boxes:
            gh.abort()
            res.success = False
            res.message = "no boxes detected"
            return res

        self._fb(gh, "assigning", 0.4)
        want = goal.arms.strip().lower() or "both"
        assigned = {}   # side -> box
        # pick the most-reliably-detected (highest-hits) box on each side
        left_boxes = sorted([b for b in boxes if b["base"][1] >= 0], key=lambda b: -b["hits"])
        right_boxes = sorted([b for b in boxes if b["base"][1] < 0], key=lambda b: -b["hits"])
        if want in ("both", "left") and left_boxes:
            assigned["left"] = left_boxes[0]
        if want in ("both", "right") and right_boxes:
            assigned["right"] = right_boxes[0]
        # fallback: requested arm with no same-side box -> highest-hits box overall
        if want in ("both", "left") and "left" not in assigned and boxes:
            assigned["left"] = max(boxes, key=lambda b: b["hits"])
        if want in ("both", "right") and "right" not in assigned and boxes:
            assigned["right"] = max(boxes, key=lambda b: b["hits"])

        self._fb(gh, "moving", 0.6)
        report = {}
        for side, b in assigned.items():
            tgt = self.move_arm(side, b["base"], goal.z, quat)
            report[side] = {"box_base": b["base"], "link7_target": tgt}
        # let the motions settle, then measure error
        time.sleep(self.move_time + 1.0)
        for side in assigned:
            cur = self.link7_pos(side)
            if cur is not None:
                tgt = report[side]["link7_target"]
                err = math.sqrt(sum((cur[i] - tgt[i]) ** 2 for i in range(3)))
                report[side]["link7_final"] = cur
                report[side]["err_mm"] = round(err * 1000, 1)

        self._fb(gh, "done", 1.0)
        gh.succeed()
        res.success = True
        res.message = f"detected {len(boxes)} box(es); moved {list(assigned.keys())}"
        res.assignments_json = json.dumps(report)
        return res


def main():
    rclpy.init()
    node = BoxAlignNode()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        ex.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
