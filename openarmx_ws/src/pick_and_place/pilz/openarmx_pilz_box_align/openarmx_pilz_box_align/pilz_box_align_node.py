#!/usr/bin/env python3
"""Bimanual box-align action server using MoveIt Pilz planning (MOTION only).

인지/모션 분리: 이 노드는 검출을 하지 않는다. 인지 노드(box_perception_node)가
발행하는 ``/detected_boxes`` (geometry_msgs/PoseArray, base frame)를 구독해, 그
박스 좌표로 좌/우 팔을 정렬한다.

On an ``AlignToBoxes`` goal it:
  1. reading_boxes -- 인지가 발행한 최신 /detected_boxes(base frame)를 읽는다.
  2. assigning     -- +Y 박스는 LEFT, -Y 박스는 RIGHT 팔에 배정(goal.arms 반영).
  3. planning+moving -- 배정된 팔마다 Pilz 모션(link7 -> box.x,box.y,z)을
     ``/plan_kinematic_path`` 서비스로 계획하고 트래젝토리를 팔 JTC 로 발행.

Pilz 가 link7 을 직접 구속하므로 link7->hand_tcp 보정은 불필요.

전제 노드: box_perception_node(/detected_boxes), MoveIt move_group(Pilz) + 좌/우 JTC.
검출/3D 변환/박스 TF 는 인지(box_perception_node) 책임 — 본 노드에는 없다.
"""
import json
import math
import time

import rclpy
from geometry_msgs.msg import Pose, PoseArray, Quaternion, Vector3
from moveit_msgs.msg import (BoundingVolume, Constraints, OrientationConstraint,
                             PositionConstraint, RobotState)
from moveit_msgs.srv import GetMotionPlan
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory

from openarmx_pilz_box_align_msgs.action import AlignToBoxes

BASE = "openarmx_body_link0"


def rpy_to_quat(roll, pitch, yaw):
    r, p, y = math.radians(roll), math.radians(pitch), math.radians(yaw)
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return [sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy]


class PilzBoxAlignNode(Node):
    def __init__(self):
        super().__init__("pilz_box_align_node")
        self.declare_parameter("default_vel_scale", 0.3)
        self.declare_parameter("plan_time", 5.0)
        self.declare_parameter("detected_boxes_topic", "/detected_boxes")
        self.declare_parameter("max_box_age", 60.0)
        gp = self.get_parameter
        self.default_vel = float(gp("default_vel_scale").value)
        self.plan_time = float(gp("plan_time").value)
        self.max_age = float(gp("max_box_age").value)

        cb = ReentrantCallbackGroup()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._boxes = None        # latest /detected_boxes poses (base frame)
        self._boxes_t = None      # rclpy Time when received
        # latched(transient_local): /detected_boxes 는 검출 1회당 1개 발행이라,
        # 이 백엔드가 늦게 떠도 최신 1개를 받도록 한다(발행측과 QoS 일치 필수).
        self.create_subscription(
            PoseArray, str(gp("detected_boxes_topic").value), self._on_boxes,
            QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group=cb)
        self.jtc_pub = {
            "left": self.create_publisher(
                JointTrajectory, "/left_joint_trajectory_controller/joint_trajectory", 10),
            "right": self.create_publisher(
                JointTrajectory, "/right_joint_trajectory_controller/joint_trajectory", 10),
        }
        self.plan_client = self.create_client(
            GetMotionPlan, "/plan_kinematic_path", callback_group=cb)
        self.srv = ActionServer(
            self, AlignToBoxes, "/openarmx/pilz_align_to_boxes",
            execute_callback=self._execute,
            goal_callback=lambda _g: GoalResponse.ACCEPT,
            cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=cb)
        self.get_logger().info(
            "pilz_box_align_node ready: action /openarmx/pilz_align_to_boxes "
            "(consumes /detected_boxes)")

    # ------------------------------------------------ perception input
    def _on_boxes(self, msg):
        self._boxes = list(msg.poses)
        self._boxes_t = self.get_clock().now()

    def _current_boxes(self):
        """최신 /detected_boxes -> [{base:[x,y,z]}], max_box_age 이내만 유효."""
        if self._boxes is None or self._boxes_t is None:
            return []
        age = (self.get_clock().now() - self._boxes_t).nanoseconds * 1e-9
        if age > self.max_age:
            self.get_logger().warning(
                f"/detected_boxes 가 오래됨({age:.1f}s) — '검출요청'을 다시 실행하세요.")
            return []
        boxes = [{"base": [p.position.x, p.position.y, p.position.z]} for p in self._boxes]
        boxes.sort(key=lambda b: -b["base"][1])     # +Y(left) 우선
        return boxes

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
        st.is_diff = True
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
        vel = goal.vel_scale if goal.vel_scale > 0.0 else self.default_vel
        planner = goal.planner.strip() if goal.planner.strip() else "LIN"
        quat = rpy_to_quat(goal.roll_deg, goal.pitch_deg, goal.yaw_deg)

        # 검출은 인지(box_perception_node) 책임 — 여기서는 최신 /detected_boxes 사용.
        self._fb(gh, "reading_boxes", 0.1)
        boxes = self._current_boxes()
        res.detections_json = json.dumps([{"base": b["base"]} for b in boxes])
        if not boxes:
            gh.abort()
            res.success = False
            res.message = "no detected boxes (먼저 '검출요청'으로 인지 실행 → /detected_boxes)"
            return res

        self._fb(gh, "assigning", 0.4)
        want = goal.arms.strip().lower() or "both"
        assigned = {}
        left_boxes = [b for b in boxes if b["base"][1] >= 0]                  # 이미 +Y 우선 정렬
        right_boxes = sorted([b for b in boxes if b["base"][1] < 0], key=lambda b: b["base"][1])
        if want in ("both", "left") and left_boxes:
            assigned["left"] = left_boxes[0]
        if want in ("both", "right") and right_boxes:
            assigned["right"] = right_boxes[0]
        if want in ("both", "left") and "left" not in assigned and boxes:
            assigned["left"] = boxes[0]
        if want in ("both", "right") and "right" not in assigned and boxes:
            assigned["right"] = boxes[-1]

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
        res.message = (f"{len(boxes)} box(es) from perception; "
                       f"planned/moved {list(assigned.keys())} (Pilz {planner})")
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
