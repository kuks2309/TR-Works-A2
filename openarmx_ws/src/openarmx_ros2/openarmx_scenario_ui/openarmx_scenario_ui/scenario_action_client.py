"""ROS2 bridge for the scenario UI.

Runs an rclpy node in a background thread (so the Qt event loop is not blocked)
and exposes Qt signals for status events, action feedback, and result.

Public API (called from Qt main thread):
  - list_scenarios(timeout_sec) -> dict | None        (synchronous service call)
  - play(scenario_path, start_sub, end_sub, speed, dry_run) -> bool
  - cancel() -> None
  - shutdown() -> None

Qt signals emitted (received by main_window):
  - sig_status_event(dict)         JSON dict from /scenario_player/status
  - sig_feedback(dict)             Action feedback
  - sig_result(dict)               Action result (success, message, etc.)
  - sig_error(str)                 Connection / call errors
"""

from __future__ import annotations

import json
import math
import threading
import time
from typing import Optional

import rclpy
from PyQt5.QtCore import QObject, pyqtSignal
from builtin_interfaces.msg import Duration as DurationMsg
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import tf2_ros
from geometry_msgs.msg import Pose, Quaternion, Vector3
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume, Constraints, OrientationConstraint, PositionConstraint,
    RobotState,
)
from rclpy.duration import Duration
from rclpy.time import Time
from shape_msgs.msg import SolidPrimitive

from openarmx_scenario_player_msgs.action import PlayScenario
from openarmx_scenario_player_msgs.srv import ListScenarios
from openarmx_scenario_ui import joint_data as jd


_MOVEIT_ERR_CODES = {
    1: "SUCCESS",
    99999: "FAILURE",
    -1: "PLANNING_FAILED",
    -2: "INVALID_MOTION_PLAN",
    -3: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
    -4: "CONTROL_FAILED",
    -5: "UNABLE_TO_AQUIRE_SENSOR_DATA",
    -6: "TIMED_OUT",
    -7: "PREEMPTED",
    -10: "START_STATE_IN_COLLISION",
    -11: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
    -12: "GOAL_IN_COLLISION",
    -13: "GOAL_VIOLATES_PATH_CONSTRAINTS",
    -14: "GOAL_CONSTRAINTS_VIOLATED",
    -15: "INVALID_GROUP_NAME",
    -16: "INVALID_GOAL_CONSTRAINTS",
    -17: "INVALID_ROBOT_STATE",
    -18: "INVALID_LINK_NAME",
    -19: "INVALID_OBJECT_NAME",
    -21: "FRAME_TRANSFORM_FAILURE",
    -22: "COLLISION_CHECKING_UNAVAILABLE",
    -23: "ROBOT_STATE_STALE",
    -24: "SENSOR_INFO_STALE",
    -25: "COMMUNICATION_FAILURE",
    -31: "NO_IK_SOLUTION",
}


def _moveit_err_str(code: int) -> str:
    return _MOVEIT_ERR_CODES.get(code, f"code {code}")


def _rpy_to_quat(roll: float, pitch: float, yaw: float):
    cr, sr = math.cos(roll / 2.0),  math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0),   math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,   # x
        cr * sp * cy + sr * cp * sy,   # y
        cr * cp * sy - sr * sp * cy,   # z
        cr * cp * cy + sr * sp * sy,   # w
    )


def _quat_to_rpy(qx: float, qy: float, qz: float, qw: float):
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class ScenarioRosBridge(QObject):
    sig_status_event = pyqtSignal(dict)
    sig_feedback = pyqtSignal(dict)
    sig_result = pyqtSignal(dict)
    sig_error = pyqtSignal(str)
    # Joint Control tab: {joint_name: value} — arms in deg, grippers in meters.
    sig_joint_state = pyqtSignal(dict)
    # Cartesian Control tab: {phase, success, error_code, message}
    sig_motion_result = pyqtSignal(dict)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        rclpy.init(args=None)
        self._node: Node = rclpy.create_node("openarmx_scenario_ui")
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)

        self._list_client = self._node.create_client(
            ListScenarios, "scenario_player/list")
        self._play_client = ActionClient(
            self._node, PlayScenario, "scenario_player/play")

        status_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_sub = self._node.create_subscription(
            String, "/scenario_player/status", self._on_status, status_qos)

        self._active_goal_handle = None

        # ---- Joint Control tab wiring (shares this node + spin thread) ----
        self._setup_joint_control()
        # ---- Diagnostics tab: rate-monitoring subscriptions ----
        self._setup_diagnostics()
        # ---- Cartesian Control tab: MoveGroup + TF lookup ----
        self._setup_cartesian_control()

        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._thread.start()

    # ---------- background spin ----------

    def _spin_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                self._executor.spin_once(timeout_sec=0.1)
            except Exception as e:
                self.sig_error.emit(f"Executor: {e}")

    # ---------- subscribers ----------

    def _on_status(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            payload = {"event": "raw", "data": msg.data}
        self.sig_status_event.emit(payload)

    # ---------- service ----------

    def list_scenarios(self, timeout_sec: float = 3.0) -> Optional[dict]:
        if not self._list_client.wait_for_service(timeout_sec=timeout_sec):
            self.sig_error.emit("scenario_player/list service unavailable")
            return None
        req = ListScenarios.Request()
        future = self._list_client.call_async(req)
        deadline = self._node.get_clock().now() + rclpy.duration.Duration(seconds=timeout_sec)
        while not future.done() and self._node.get_clock().now() < deadline:
            threading.Event().wait(0.05)
        if not future.done():
            self.sig_error.emit("ListScenarios call timed out")
            return None
        resp = future.result()
        scenarios = []
        for name, desc, count, subs in zip(
                resp.names, resp.descriptions, resp.sub_counts, resp.sub_names):
            scenarios.append({
                "name": name,
                "description": desc,
                "sub_count": count,
                "sub_names": subs.split(",") if subs else [],
            })
        return {"scenarios": scenarios}

    # ---------- action ----------

    def play(self, scenario_path: str, start_sub: int, end_sub: int,
             speed: float, dry_run: bool) -> bool:
        if not self._play_client.wait_for_server(timeout_sec=3.0):
            self.sig_error.emit("scenario_player/play action server unavailable")
            return False

        goal = PlayScenario.Goal()
        goal.scenario_path = scenario_path
        goal.start_sub = int(start_sub)
        goal.end_sub = int(end_sub)
        goal.speed_scale = float(speed)
        goal.dry_run = bool(dry_run)

        send_future = self._play_client.send_goal_async(
            goal, feedback_callback=self._on_feedback)
        send_future.add_done_callback(self._on_goal_response)
        return True

    def _on_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as e:
            self.sig_error.emit(f"Goal send failed: {e}")
            return
        if goal_handle is None or not goal_handle.accepted:
            self.sig_error.emit("Goal rejected by server")
            return
        self._active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_feedback(self, msg) -> None:
        fb = msg.feedback
        self.sig_feedback.emit({
            "current_sub": fb.current_sub,
            "total_subs": fb.total_subs,
            "sub_name": fb.sub_name,
            "current_step": fb.current_step,
            "total_steps": fb.total_steps,
            "step_name": fb.step_name,
            "phase": fb.phase,
            "step_progress": fb.step_progress,
            "overall_progress": fb.overall_progress,
            "elapsed_sec": fb.elapsed_sec,
        })

    def _on_result(self, future) -> None:
        self._active_goal_handle = None
        try:
            wrapped = future.result()
        except Exception as e:
            self.sig_error.emit(f"Result fetch failed: {e}")
            return
        r = wrapped.result
        self.sig_result.emit({
            "success": r.success,
            "message": r.message,
            "subs_completed": r.subs_completed,
            "subs_total": r.subs_total,
            "steps_completed": r.steps_completed,
            "total_duration_sec": r.total_duration_sec,
        })

    def cancel(self) -> None:
        if self._active_goal_handle is None:
            return
        self._active_goal_handle.cancel_goal_async()

    # ==================================================================
    # Joint Control tab — /joint_states (SIL) + trajectory/gripper (HIL)
    # ==================================================================

    def _setup_joint_control(self) -> None:
        joint_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        # SIL: publish full robot pose for robot_state_publisher / RViz.
        self._js_pub = self._node.create_publisher(
            JointState, "/joint_states", joint_qos)
        # HIL: per-arm trajectory command publishers.
        self._traj_pub_left = self._node.create_publisher(
            JointTrajectory, jd.LEFT_TRAJ_TOPIC, joint_qos)
        self._traj_pub_right = self._node.create_publisher(
            JointTrajectory, jd.RIGHT_TRAJ_TOPIC, joint_qos)
        # HIL: gripper action clients.
        self._grip_client_left = ActionClient(
            self._node, GripperCommand, jd.LEFT_GRIPPER_ACTION)
        self._grip_client_right = ActionClient(
            self._node, GripperCommand, jd.RIGHT_GRIPPER_ACTION)
        # Feedback subscriber for actual angles.
        self._js_sub = self._node.create_subscription(
            JointState, "/joint_states", self._on_joint_state, joint_qos)

        self._arm_names = list(jd.ARM_JOINT_NAMES)
        self._gripper_names = list(jd.GRIPPER_NAMES)
        # Self-echo filter: ignore our own SIL publish bouncing back.
        self._last_self_publish = None  # {name: value}

    def clear_self_echo(self) -> None:
        """Drop the self-echo filter (call when switching to HIL so the first
        real feedback is not mistaken for our own publish)."""
        self._last_self_publish = None

    def _on_joint_state(self, msg: JointState) -> None:
        # Drop echoes of our own SIL publish to avoid a feedback loop.
        if self._last_self_publish:
            checked = [(n, p) for n, p in zip(msg.name, msg.position)
                       if n in self._last_self_publish]
            if checked and all(
                abs(self._to_display(n, p) - self._last_self_publish[n]) < 0.01
                for n, p in checked
            ):
                return
        positions = {n: self._to_display(n, p)
                     for n, p in zip(msg.name, msg.position)}
        self.sig_joint_state.emit(positions)

    @staticmethod
    def _to_display(name: str, raw: float) -> float:
        """Convert a raw /joint_states value to the UI unit:
        arm joints rad->deg, prismatic grippers stay in meters."""
        if name in jd.GRIPPER_NAMES:
            return raw
        return math.degrees(raw)

    def topic_exists(self, topic: str) -> bool:
        return topic in dict(self._node.get_topic_names_and_types())

    def traj_subscriber_count(self) -> int:
        try:
            return (self._node.count_subscribers(jd.LEFT_TRAJ_TOPIC)
                    + self._node.count_subscribers(jd.RIGHT_TRAJ_TOPIC))
        except Exception:
            return 0

    def publish_joint_states(self, arm_deg: dict, gripper_m: dict) -> None:
        """SIL: publish all joints to /joint_states (arms deg->rad, grippers m)."""
        names, positions = [], []
        for n in self._arm_names:
            if n in arm_deg:
                names.append(n)
                positions.append(math.radians(arm_deg[n]))
        for n in self._gripper_names:
            if n in gripper_m:
                names.append(n)
                positions.append(float(gripper_m[n]))
        msg = JointState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.name = names
        msg.position = positions
        msg.velocity = [0.0] * len(names)
        msg.effort = [0.0] * len(names)
        self._js_pub.publish(msg)
        # Record in display units for the self-echo filter.
        self._last_self_publish = {}
        for n in self._arm_names:
            if n in arm_deg:
                self._last_self_publish[n] = arm_deg[n]
        for n in self._gripper_names:
            if n in gripper_m:
                self._last_self_publish[n] = float(gripper_m[n])

    def send_trajectory(self, arm_deg: dict, duration_sec: float) -> None:
        """HIL: send per-arm JointTrajectory to the trajectory controllers."""
        sec = int(duration_sec)
        nanosec = int((duration_sec - sec) * 1e9)
        stamp = self._node.get_clock().now().to_msg()
        for names, pub in (
            (jd.LEFT_ARM_JOINTS, self._traj_pub_left),
            (jd.RIGHT_ARM_JOINTS, self._traj_pub_right),
        ):
            traj = JointTrajectory()
            traj.header.stamp = stamp
            traj.joint_names = list(names)
            pt = JointTrajectoryPoint()
            pt.positions = [math.radians(arm_deg.get(n, 0.0)) for n in names]
            pt.velocities = [0.0] * len(names)
            pt.time_from_start = DurationMsg(sec=sec, nanosec=nanosec)
            traj.points = [pt]
            pub.publish(traj)
        self._node.get_logger().info(
            f"HIL: sent arm trajectories (duration={duration_sec:.1f}s)")

    def send_gripper(self, gripper_m: dict, max_effort: float = 14.0) -> None:
        """HIL: send GripperCommand goals to each gripper controller."""
        for name, client in (
            (jd.LEFT_GRIPPER, self._grip_client_left),
            (jd.RIGHT_GRIPPER, self._grip_client_right),
        ):
            if name not in gripper_m:
                continue
            if not client.server_is_ready():
                continue
            goal = GripperCommand.Goal()
            goal.command.position = float(gripper_m[name])
            goal.command.max_effort = float(max_effort)
            client.send_goal_async(goal)

    # ==================================================================
    # Diagnostics tab — node liveness + topic publish-rate monitoring
    # ==================================================================

    def _setup_diagnostics(self) -> None:
        from openarmx_scenario_ui import diagnostics_spec as ds
        self._diag = {}                      # topic -> {"count": int, "last": float}
        self._diag_t0 = time.monotonic()
        qos_map = {
            ds.QOS_SENSOR: QoSProfile(
                depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE),
            ds.QOS_LATCHED: QoSProfile(
                depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL),
            ds.QOS_DEFAULT: QoSProfile(
                depth=10, reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE),
        }
        for _cat, topic, mtype, qkind, _kind, _detail in ds.TOPIC_SPECS:
            self._diag[topic] = {"count": 0, "last": 0.0}

            def _cb(_msg, t=topic):
                d = self._diag[t]
                d["count"] += 1
                d["last"] = time.monotonic()

            self._node.create_subscription(mtype, topic, _cb, qos_map[qkind])

    def diag_consume_rates(self) -> dict:
        """Return {topic: {hz, age}} measured since the last call, then reset."""
        now = time.monotonic()
        elapsed = max(now - self._diag_t0, 1e-3)
        out = {}
        for topic, d in self._diag.items():
            out[topic] = {
                "hz": d["count"] / elapsed,
                "age": (now - d["last"]) if d["last"] > 0 else None,
            }
            d["count"] = 0
        self._diag_t0 = now
        return out

    def live_node_names(self) -> set:
        """Set of currently discoverable node names (short + fully-qualified)."""
        names = set()
        try:
            for n, ns in self._node.get_node_names_and_namespaces():
                names.add(n)
                names.add((ns.rstrip("/") + "/" + n) if ns and ns != "/" else "/" + n)
        except Exception:
            pass
        return names

    def topic_pub_count(self, topic: str) -> int:
        try:
            return self._node.count_publishers(topic)
        except Exception:
            return 0

    # ==================================================================
    # Cartesian Control tab — MoveGroup action + TF lookup
    # ==================================================================

    def _setup_cartesian_control(self) -> None:
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self._node)
        self._move_group_client = ActionClient(
            self._node, MoveGroup, "/move_action")
        self._move_goal_handle = None

    def get_ee_pose(self, arm: str, frame_id: str = "") -> Optional[dict]:
        """TF lookup: pose of openarmx_<arm>_link7 expressed in `frame_id`
        (default: openarmx_<arm>_link0). Returns dict or None."""
        target_link = f"openarmx_{arm}_link7"
        if not frame_id:
            frame_id = f"openarmx_{arm}_link0"
        try:
            if not self._tf_buffer.can_transform(
                    frame_id, target_link, Time(),
                    timeout=Duration(seconds=0.1)):
                return None
            ts = self._tf_buffer.lookup_transform(
                frame_id, target_link, Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        tr = ts.transform.translation
        rq = ts.transform.rotation
        roll, pitch, yaw = _quat_to_rpy(rq.x, rq.y, rq.z, rq.w)
        return {
            "x": tr.x, "y": tr.y, "z": tr.z,
            "qx": rq.x, "qy": rq.y, "qz": rq.z, "qw": rq.w,
            "roll": roll, "pitch": pitch, "yaw": yaw,
            "frame_id": frame_id, "link": target_link,
        }

    def plan_and_execute_cartesian(
            self, arm: str, planner_id: str, target_pose: dict,
            frame_id: str, vel_scale: float = 0.1, acc_scale: float = 0.1,
            plan_time: float = 5.0, execute: bool = True) -> bool:
        """Send a MoveGroup goal for pose target. planner_id ∈
        {PILZ_LIN, PILZ_PTP, PILZ_CIRC, OMPL}. target_pose has
        keys x/y/z (m) and roll/pitch/yaw (rad)."""
        if not self._move_group_client.wait_for_server(timeout_sec=2.0):
            self.sig_motion_result.emit({
                "phase": "send", "success": False, "error_code": -2,
                "message": "/move_action unavailable",
            })
            return False
        goal_msg = MoveGroup.Goal()
        req = goal_msg.request
        # is_diff=True tells MoveIt "use current PlanningScene state as the
        # start"; without this, an empty RobotState is interpreted as ZERO
        # joints and the plan succeeds with a trivial/wrong trajectory.
        start = RobotState()
        start.is_diff = True
        req.start_state = start
        req.group_name = f"{arm}_arm"
        pid = (planner_id or "").upper()
        if pid.startswith("PILZ_"):
            req.pipeline_id = "pilz_industrial_motion_planner"
            req.planner_id = pid.split("_", 1)[1]   # LIN / PTP / CIRC
        elif pid == "OMPL":
            req.pipeline_id = "ompl"
            req.planner_id = ""
        else:
            req.pipeline_id = planner_id or ""
            req.planner_id = ""
        req.num_planning_attempts = 1
        req.allowed_planning_time = float(plan_time)
        req.max_velocity_scaling_factor = float(vel_scale)
        req.max_acceleration_scaling_factor = float(acc_scale)
        req.goal_constraints = [self._make_pose_goal_constraints(
            link_name=f"openarmx_{arm}_link7", frame_id=frame_id,
            pose=target_pose,
        )]
        opts = goal_msg.planning_options
        opts.plan_only = (not execute)
        opts.look_around = False
        opts.replan = False
        send_future = self._move_group_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._on_move_goal_response)
        return True

    @staticmethod
    def _make_pose_goal_constraints(
            link_name: str, frame_id: str, pose: dict,
            tol_pos: float = 0.001, tol_ang: float = 0.01) -> Constraints:
        qx, qy, qz, qw = _rpy_to_quat(
            pose["roll"], pose["pitch"], pose["yaw"])
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
        region_pose = Pose()
        region_pose.position.x = float(pose["x"])
        region_pose.position.y = float(pose["y"])
        region_pose.position.z = float(pose["z"])
        region_pose.orientation.w = 1.0
        bv.primitive_poses = [region_pose]
        pc.constraint_region = bv
        pc.weight = 1.0
        c.position_constraints = [pc]
        oc = OrientationConstraint()
        oc.header.frame_id = frame_id
        oc.link_name = link_name
        oc.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        oc.absolute_x_axis_tolerance = tol_ang
        oc.absolute_y_axis_tolerance = tol_ang
        oc.absolute_z_axis_tolerance = tol_ang
        oc.weight = 1.0
        c.orientation_constraints = [oc]
        return c

    def _on_move_goal_response(self, future) -> None:
        try:
            gh = future.result()
        except Exception as e:
            self.sig_motion_result.emit({
                "phase": "send", "success": False, "error_code": -3,
                "message": str(e),
            })
            return
        if not gh.accepted:
            self.sig_motion_result.emit({
                "phase": "send", "success": False, "error_code": -4,
                "message": "goal rejected",
            })
            return
        self._move_goal_handle = gh
        result_future = gh.get_result_async()
        result_future.add_done_callback(self._on_move_result)

    def _on_move_result(self, future) -> None:
        self._move_goal_handle = None
        try:
            wrapped = future.result()
        except Exception as e:
            self.sig_motion_result.emit({
                "phase": "result", "success": False, "error_code": -5,
                "message": str(e),
            })
            return
        r = wrapped.result
        code = r.error_code.val if r and r.error_code else -1
        self.sig_motion_result.emit({
            "phase": "done",
            "success": (code == 1),
            "error_code": code,
            "message": "OK" if code == 1 else _moveit_err_str(code),
        })

    def cancel_motion(self) -> None:
        if self._move_goal_handle is not None:
            self._move_goal_handle.cancel_goal_async()

    # ---------- shutdown ----------

    def shutdown(self) -> None:
        self._stop_event.set()
        self._thread.join(timeout=2.0)
        try:
            self._node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
