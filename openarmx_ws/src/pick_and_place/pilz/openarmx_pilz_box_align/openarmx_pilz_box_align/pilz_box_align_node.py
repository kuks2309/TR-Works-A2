#!/usr/bin/env python3
"""Bimanual box-align action server using MoveIt Pilz planning.

On an ``AlignToBoxes`` goal it:
  1. detecting -- triggers YOLOv8 ``DetectBox`` over several frames, clusters in
     3D, and computes each box's TOP-surface centroid in the base frame.
  2. assigning -- assigns +Y boxes to the LEFT arm, -Y to the RIGHT arm.
  3. planning+moving -- for each assigned arm, plans a Pilz LIN motion that brings
     link7 to (box.x, box.y, z) with the commanded hand orientation via the
     ``/plan_kinematic_path`` SERVICE (one round-trip, no MoveGroup action 2-stage
     handshake), then publishes the planned trajectory straight to the arm's JTC
     topic (/<side>_joint_trajectory_controller/joint_trajectory).

Pilz constrains link7 directly, so no link7->hand_tcp offset compensation is
needed (unlike the cyclo variant).

Prerequisites (already running): D435 camera, YOLOv8 DetectBox server, calibrated
base->camera TF, MoveIt move_group (Pilz pipeline) + the arm JTCs.
"""
import json
import math
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, Quaternion, TransformStamped, Vector3
from moveit_msgs.msg import (BoundingVolume, Constraints, OrientationConstraint,
                             PositionConstraint, RobotState)
from moveit_msgs.srv import GetMotionPlan
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener
from trajectory_msgs.msg import JointTrajectory

from yolov8_detection_msgs.action import DetectBox
from openarmx_pilz_box_align_msgs.action import AlignToBoxes

BASE = "openarmx_body_link0"
CAM = "camera_color_optical_frame"
DEFAULT_PROMPTS = "box, cube, block, colored cube, toy block, cardboard box, square box"


def rpy_to_quat(roll, pitch, yaw):
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


class PilzBoxAlignNode(Node):
    def __init__(self):
        super().__init__("pilz_box_align_node")
        self.declare_parameter("n_frames", 5)
        self.declare_parameter("cluster_radius", 0.08)
        self.declare_parameter("min_hits", 2)
        self.declare_parameter("default_confidence", 0.02)
        self.declare_parameter("default_vel_scale", 0.3)
        self.declare_parameter("plan_time", 5.0)
        self.declare_parameter("ws_x", [0.05, 0.70])
        self.declare_parameter("ws_y_abs", 0.45)
        self.declare_parameter("ws_z", [0.10, 0.32])
        gp = self.get_parameter
        self.n_frames = int(gp("n_frames").value)
        self.cluster_r = float(gp("cluster_radius").value)
        self.min_hits = int(gp("min_hits").value)
        self.default_conf = float(gp("default_confidence").value)
        self.default_vel = float(gp("default_vel_scale").value)
        self.plan_time = float(gp("plan_time").value)
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
        self._max_box = 0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.jtc_pub = {
            "left": self.create_publisher(
                JointTrajectory, "/left_joint_trajectory_controller/joint_trajectory", 10),
            "right": self.create_publisher(
                JointTrajectory, "/right_joint_trajectory_controller/joint_trajectory", 10),
        }
        self.detect_client = ActionClient(
            self, DetectBox, "/yolov8_node/detect", callback_group=cb)
        self.plan_client = self.create_client(
            GetMotionPlan, "/plan_kinematic_path", callback_group=cb)
        self.srv = ActionServer(
            self, AlignToBoxes, "/openarmx/pilz_align_to_boxes",
            execute_callback=self._execute,
            goal_callback=lambda _g: GoalResponse.ACCEPT,
            cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=cb)
        self.get_logger().info("pilz_box_align_node ready: action /openarmx/pilz_align_to_boxes")

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
        r_f = gh_f.result().get_result_async()
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
            if len(cl["pts"]) < self.min_hits:
                continue
            b = self._cam_to_base(cl["mean"])
            if b is None:
                continue
            if (self.ws_x[0] < b[0] < self.ws_x[1] and abs(b[1]) < self.ws_y
                    and self.ws_z[0] < b[2] < self.ws_z[1]):
                boxes.append({"cam": cl["mean"], "base": b, "hits": len(cl["pts"])})
        boxes.sort(key=lambda d: -d["base"][1])
        return boxes

    def publish_box_tfs(self, boxes):
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
        for i in range(len(boxes), self._max_box):
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

    # ------------------------------------------------------------- Pilz move
    @staticmethod
    def _pose_constraints(link_name, frame_id, xyz, quat, tol_pos=0.001, tol_ang=0.01):
        c = Constraints()
        c.name = "pose_goal"
        pc = PositionConstraint()
        pc.header.frame_id = frame_id
        pc.link_name = link_name
        pc.target_point_offset = Vector3(x=0.0, y=0.0, z=0.0)
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [tol_pos]
        bv = BoundingVolume()
        bv.primitives = [sphere]
        rp = Pose()
        rp.position.x, rp.position.y, rp.position.z = float(xyz[0]), float(xyz[1]), float(xyz[2])
        rp.orientation.w = 1.0
        bv.primitive_poses = [rp]
        pc.constraint_region = bv
        pc.weight = 1.0
        c.position_constraints = [pc]
        oc = OrientationConstraint()
        oc.header.frame_id = frame_id
        oc.link_name = link_name
        oc.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        oc.absolute_x_axis_tolerance = tol_ang
        oc.absolute_y_axis_tolerance = tol_ang
        oc.absolute_z_axis_tolerance = tol_ang
        oc.weight = 1.0
        c.orientation_constraints = [oc]
        return c

    def move_arm(self, side, box_base, z, quat, vel_scale, planner):
        """Plan a Pilz motion (link7 -> target) via /plan_kinematic_path and publish
        the trajectory to the arm JTC. Returns (link7_target, duration, error)."""
        target = [box_base[0], box_base[1], z]
        req = GetMotionPlan.Request()
        mpr = req.motion_plan_request
        st = RobotState()
        st.is_diff = True                 # start from the current planning-scene state
        mpr.start_state = st
        mpr.group_name = f"{side}_arm"
        mpr.pipeline_id = "pilz_industrial_motion_planner"
        mpr.planner_id = (planner or "LIN").upper()
        mpr.num_planning_attempts = 1
        mpr.allowed_planning_time = self.plan_time
        mpr.max_velocity_scaling_factor = vel_scale
        mpr.max_acceleration_scaling_factor = vel_scale
        mpr.goal_constraints = [self._pose_constraints(
            f"openarmx_{side}_link7", BASE, target, quat)]
        if not self.plan_client.wait_for_service(timeout_sec=5.0):
            return target, 0.0, "plan service unavailable"
        fut = self.plan_client.call_async(req)
        if not self._wait(fut, 20.0):
            return target, 0.0, "plan timeout"
        resp = fut.result().motion_plan_response
        if resp.error_code.val != 1:      # MoveItErrorCodes.SUCCESS == 1
            return target, 0.0, f"plan failed (code {resp.error_code.val})"
        traj = resp.trajectory.joint_trajectory
        if not traj.points:
            return target, 0.0, "empty trajectory"
        self.jtc_pub[side].publish(traj)
        d = traj.points[-1].time_from_start
        return target, d.sec + d.nanosec * 1e-9, ""

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
        vel = goal.vel_scale if goal.vel_scale > 0.0 else self.default_vel
        planner = goal.planner.strip() if goal.planner.strip() else "LIN"
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
        assigned = {}
        left_boxes = sorted([b for b in boxes if b["base"][1] >= 0], key=lambda b: -b["hits"])
        right_boxes = sorted([b for b in boxes if b["base"][1] < 0], key=lambda b: -b["hits"])
        if want in ("both", "left") and left_boxes:
            assigned["left"] = left_boxes[0]
        if want in ("both", "right") and right_boxes:
            assigned["right"] = right_boxes[0]
        if want in ("both", "left") and "left" not in assigned and boxes:
            assigned["left"] = max(boxes, key=lambda b: b["hits"])
        if want in ("both", "right") and "right" not in assigned and boxes:
            assigned["right"] = max(boxes, key=lambda b: b["hits"])

        self._fb(gh, "planning", 0.6)
        report = {}
        durations = []
        for side, b in assigned.items():
            tgt, dur, err = self.move_arm(side, b["base"], goal.z, quat, vel, planner)
            report[side] = {"box_base": b["base"], "link7_target": tgt, "plan_error": err}
            if err:
                self.get_logger().warning(f"{side}: {err}")
            else:
                durations.append(dur)

        self._fb(gh, "moving", 0.8)
        time.sleep((max(durations) if durations else 3.0) + 1.5)
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
        res.message = f"detected {len(boxes)} box(es); planned/moved {list(assigned.keys())} (Pilz {planner})"
        res.assignments_json = json.dumps(report)
        return res


def main():
    rclpy.init()
    node = PilzBoxAlignNode()
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
