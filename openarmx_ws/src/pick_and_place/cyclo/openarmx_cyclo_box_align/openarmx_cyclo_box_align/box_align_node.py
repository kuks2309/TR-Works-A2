#!/usr/bin/env python3
"""Bimanual box-align action server (cyclo MoveL, MOTION only).

인지/모션 분리: 이 노드는 검출하지 않는다. 인지 노드(box_perception_node)가
발행하는 ``/detected_boxes`` (geometry_msgs/PoseArray, base frame)를 구독해, 그
박스 좌표로 좌/우 팔을 cyclo MoveL 로 정렬한다.

On an ``AlignToBoxes`` goal it:
  1. reading_boxes -- 인지가 발행한 최신 /detected_boxes(base frame)를 읽는다.
  2. assigning     -- +Y 박스는 LEFT, -Y 박스는 RIGHT 팔에 배정(goal.arms 반영).
  3. moving        -- 팔마다 link7 target (box.x, box.y, z) 를 만들고 고정
     link7->hand_tcp 오프셋을 보정해 ``/openarmx/<side>/movel`` 로 MoveL 발행.
     이후 컨트롤러가 내는 ee_pose 가 명령 tcp_target 의 arrive_tol_m 안에 들면
     도달로 보고(고정 sleep 없음), 도달한 팔의 gripper 를 연다.

전제 노드: box_perception_node(/detected_boxes), cyclo MoveL 컨트롤러
(/openarmx/{left,right}/movel). 검출/3D 변환/박스 TF 는 인지 책임 — 본 노드엔 없다.
"""
import json
import math
import threading
import time

import numpy as np
import rclpy
from control_msgs.action import GripperCommand
from geometry_msgs.msg import PoseArray, PoseStamped
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from openarmx_scenario_player_msgs.msg import MoveL
from openarmx_cyclo_box_align_msgs.action import AlignToBoxes

BASE = "openarmx_body_link0"


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


class BoxAlignNode(Node):
    def __init__(self):
        super().__init__("box_align_node")
        self.declare_parameter("move_time", 6.0)
        self.declare_parameter("detected_boxes_topic", "/detected_boxes")
        self.declare_parameter("max_box_age", 60.0)
        # 목표 도달 시 gripper open: finger joint 범위 0.0(닫힘)~0.044m(완전 열림).
        self.declare_parameter("gripper_open_pos", 0.044)
        self.declare_parameter("gripper_effort", 14.0)
        # 고정 sleep 대신 ee_pose 피드백 기반 도달 대기: 컨트롤러가 발행하는 EE
        # 포즈가 명령한 tcp_target 의 arrive_tol_m(미터) 안에 들면 도달로 본다.
        self.declare_parameter("arrive_tol_m", 0.005)
        self.declare_parameter("ee_pose_topic_template", "/openarmx/{side}/ee_pose")
        gp = self.get_parameter
        self.move_time = float(gp("move_time").value)
        self.max_age = float(gp("max_box_age").value)
        self.gripper_open_pos = float(gp("gripper_open_pos").value)
        self.gripper_effort = float(gp("gripper_effort").value)
        self.arrive_tol = float(gp("arrive_tol_m").value)
        ee_tmpl = str(gp("ee_pose_topic_template").value)

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
        self.movel_pub = {
            "left": self.create_publisher(MoveL, "/openarmx/left/movel", 10),
            "right": self.create_publisher(MoveL, "/openarmx/right/movel", 10),
        }
        # 목표 도달 후 여는 GripperActionController (control_msgs/GripperCommand).
        self.grip_client = {
            "left": ActionClient(
                self, GripperCommand, "/left_gripper_controller/gripper_cmd",
                callback_group=cb),
            "right": ActionClient(
                self, GripperCommand, "/right_gripper_controller/gripper_cmd",
                callback_group=cb),
        }
        # 컨트롤러가 발행하는 현재 EE(hand_tcp) 포즈 — 도달 판정용(완료 신호 대용).
        self._ee_pose = {"left": None, "right": None}
        for side in ("left", "right"):
            self.create_subscription(
                PoseStamped, ee_tmpl.format(side=side),
                lambda msg, s=side: self._on_ee_pose(s, msg), 10,
                callback_group=cb)
        self.srv = ActionServer(
            self, AlignToBoxes, "/openarmx/align_to_boxes",
            execute_callback=self._execute,
            goal_callback=lambda _g: GoalResponse.ACCEPT,
            cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=cb)
        self.get_logger().info(
            "box_align_node ready: action /openarmx/align_to_boxes (consumes /detected_boxes)")

    # ------------------------------------------------ perception input
    def _on_boxes(self, msg):
        self._boxes = list(msg.poses)
        self._boxes_t = self.get_clock().now()

    def _on_ee_pose(self, side, msg):
        p = msg.pose.position
        self._ee_pose[side] = [p.x, p.y, p.z]

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

    def _tf(self, target, source, timeout=5.0):
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < timeout:
            try:
                return self.tf_buffer.lookup_transform(target, source, Time()).transform
            except Exception:
                time.sleep(0.1)
        return None

    # ----------------------------------------------------------------- move
    def move_arm(self, side, box_base, z, quat):
        """Command <side> arm so link7 reaches (box.x, box.y, z) with orientation
        quat. Compensates the fixed link7->hand_tcp offset (cyclo controls tcp).
        Returns (link7_target, tcp_target) — tcp_target is the EE goal we command,
        used to judge arrival against the controller's ee_pose feedback."""
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
        return link7_target.tolist(), tcp_pos.tolist()

    def link7_pos(self, side):
        tf = self._tf(BASE, f"openarmx_{side}_link7", timeout=1.0)
        if tf is None:
            return None
        return [tf.translation.x, tf.translation.y, tf.translation.z]

    # ----------------------------------------------------------------- arrival
    def _wait_until_arrived(self, assigned, report, timeout):
        """Wait until each assigned arm's ee_pose (controller feedback) is within
        arrive_tol of its tcp_target, or until timeout. Replaces a blind fixed sleep
        so verification/gripper happen at real arrival. ee_pose 미수신/미수렴 시에는
        timeout 까지 대기(기존 동작으로 안전 폴백). Records per-side 'wait'."""
        pending = set(assigned.keys())
        t0 = time.time()
        while pending and rclpy.ok() and time.time() - t0 < timeout:
            for side in list(pending):
                ee = self._ee_pose.get(side)
                if ee is None:
                    continue
                tgt = report[side]["tcp_target"]
                err = math.sqrt(sum((ee[i] - tgt[i]) ** 2 for i in range(3)))
                if err <= self.arrive_tol:
                    report[side]["wait"] = "arrived"
                    pending.discard(side)
            if pending:
                time.sleep(0.05)
        for side in pending:
            report[side].setdefault("wait", "timeout")

    # ---------------------------------------------------------------- gripper
    def open_gripper(self, side):
        """Open the <side> gripper via GripperActionController; blocks until result.
        Returns "" on success or an error string (MultiThreadedExecutor-safe)."""
        client = self.grip_client.get(side)
        if client is None:
            return f"unknown gripper side {side!r}"
        if not client.wait_for_server(timeout_sec=3.0):
            return f"{side} gripper action unavailable"
        goal = GripperCommand.Goal()
        goal.command.position = self.gripper_open_pos
        goal.command.max_effort = self.gripper_effort
        done = threading.Event()
        box = {"err": ""}

        def on_result(fut):
            try:
                fut.result()
            except Exception as e:
                box["err"] = str(e)
            finally:
                done.set()

        def on_response(fut):
            try:
                gh = fut.result()
            except Exception as e:
                box["err"] = str(e)
                done.set()
                return
            if not gh.accepted:
                box["err"] = "goal rejected"
                done.set()
                return
            gh.get_result_async().add_done_callback(on_result)

        client.send_goal_async(goal).add_done_callback(on_response)
        if not done.wait(timeout=10.0):
            return f"{side} gripper open timeout"
        return box["err"]

    # -------------------------------------------------------------- execute
    def _fb(self, gh, phase, prog):
        fb = AlignToBoxes.Feedback()
        fb.phase = phase
        fb.progress = float(prog)
        gh.publish_feedback(fb)

    def _execute(self, gh):
        goal = gh.request
        res = AlignToBoxes.Result()
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

        self._fb(gh, "moving", 0.6)
        report = {}
        for side, b in assigned.items():
            tgt, tcp_tgt = self.move_arm(side, b["base"], goal.z, quat)
            report[side] = {"box_base": b["base"], "link7_target": tgt, "tcp_target": tcp_tgt}
        # ee_pose 피드백으로 실제 도달까지 대기(고정 sleep 대체). 미수신/미수렴 시
        # move_time+1.0 까지 대기 = 기존 동작으로 안전 폴백.
        self._wait_until_arrived(assigned, report, timeout=self.move_time + 1.0)
        for side in assigned:
            cur = self.link7_pos(side)
            if cur is not None:
                tgt = report[side]["link7_target"]
                err = math.sqrt(sum((cur[i] - tgt[i]) ** 2 for i in range(3)))
                report[side]["link7_final"] = cur
                report[side]["err_mm"] = round(err * 1000, 1)

        # 목표 지점 도달 -> 배정된 팔의 gripper open.
        self._fb(gh, "opening_gripper", 0.9)
        for side in assigned:
            gerr = self.open_gripper(side)
            report[side]["gripper"] = "open" if not gerr else f"open_failed: {gerr}"
            if gerr:
                self.get_logger().warning(f"{side} gripper open: {gerr}")

        self._fb(gh, "done", 1.0)
        gh.succeed()
        res.success = True
        res.message = f"{len(boxes)} box(es) from perception; moved {list(assigned.keys())}"
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
