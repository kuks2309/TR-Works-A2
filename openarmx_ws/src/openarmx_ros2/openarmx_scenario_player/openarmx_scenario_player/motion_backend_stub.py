"""Motion backend — executes scenario steps as real robot motion (Phase 1).

`execute_step(step_def)` is called once per step from ScenarioPlayerNode's
execute loop (unless dry_run), from inside the PlayScenario action's execute
callback. That node runs a MultiThreadedExecutor with a ReentrantCallbackGroup,
so this method may BLOCK waiting on an action result without deadlocking
(other executor threads service the MoveGroup / gripper client callbacks).

Step types (matches the marker / Cartesian-tab "Capture -> MoveL Step"):
  - 'movel'  : Cartesian pose goal via MoveGroup, Pilz LIN (straight line)
  - 'movej'  : Cartesian pose goal via MoveGroup, Pilz PTP (joint interp)
  - 'gripper': control_msgs/GripperCommand
  - 'wait' / None : no-op (the player's sleep loop covers the dwell)

movel / movej step dict:
  {type, arm:'left'|'right', frame_id, planner:'pilz_lin'|'pilz_ptp'|'ompl',
   target_pose:{position:[x,y,z], orientation_xyzw:[qx,qy,qz,qw]},
   max_vel_scale, max_acc_scale}

Requires move_group (/move_action) running, e.g.:
  ros2 launch openarmx_bimanual_moveit_config move_group.launch.py
"""

from __future__ import annotations

import threading
from typing import Any, Dict, Optional

from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from control_msgs.action import GripperCommand
from geometry_msgs.msg import Pose, Quaternion, Vector3
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume, Constraints, OrientationConstraint, PositionConstraint,
    RobotState,
)
from shape_msgs.msg import SolidPrimitive


# Keep the class name for import compatibility with scenario_player_node.
class MotionBackendStub:
    def __init__(self, node: Node) -> None:
        self._node = node
        self._logger = node.get_logger()
        # Reentrant group so client callbacks fire on other executor threads
        # while execute_step() blocks on its result Event.
        self._cbg = ReentrantCallbackGroup()
        self._mg = ActionClient(
            node, MoveGroup, "/move_action", callback_group=self._cbg)
        self._grip = {
            "left": ActionClient(
                node, GripperCommand,
                "/left_gripper_controller/gripper_cmd", callback_group=self._cbg),
            "right": ActionClient(
                node, GripperCommand,
                "/right_gripper_controller/gripper_cmd", callback_group=self._cbg),
        }

    # ------------------------------------------------------------------
    def execute_step(self, step_def: Dict[str, Any]) -> None:
        if not isinstance(step_def, dict):
            return None
        t = (step_def.get("type") or "wait").lower()
        try:
            if t == "movel":
                self._send_pose_goal(step_def, default_plan_id="LIN")
            elif t == "movej":
                self._send_pose_goal(step_def, default_plan_id="PTP")
            elif t == "gripper":
                self._send_gripper(step_def)
            elif t in ("wait", "none", ""):
                pass  # the player's sleep loop covers the dwell
            else:
                self._logger.warn(f"Unknown step type: {t!r} (skipped)")
        except Exception as exc:  # never let one bad step crash the player
            self._logger.error(f"execute_step({t}) failed: {exc}")
        return None

    # ---- movel / movej via MoveGroup ---------------------------------
    def _send_pose_goal(self, step: Dict[str, Any], default_plan_id: str) -> None:
        name = step.get("name", "?")
        arm = (step.get("arm") or "right").lower()
        tp = step.get("target_pose") or {}
        pos = tp.get("position")
        quat = tp.get("orientation_xyzw")
        if not (isinstance(pos, (list, tuple)) and len(pos) == 3
                and isinstance(quat, (list, tuple)) and len(quat) == 4):
            self._logger.error(
                f"step '{name}': missing target_pose position/orientation_xyzw")
            return
        frame_id = step.get("frame_id") or f"openarmx_{arm}_link0"

        pipeline = "pilz_industrial_motion_planner"
        plan_id = default_plan_id
        planner = (step.get("planner") or "").upper()
        if planner.startswith("PILZ_"):
            plan_id = planner.split("_", 1)[1]          # LIN / PTP / CIRC
        elif planner == "OMPL":
            pipeline, plan_id = "ompl", ""

        vel = _clamp(float(step.get("max_vel_scale", 0.1) or 0.1))
        acc = _clamp(float(step.get("max_acc_scale", 0.1) or 0.1))
        plan_time = float(step.get("plan_time", 5.0) or 5.0)

        if not self._mg.wait_for_server(timeout_sec=3.0):
            self._logger.error(
                f"step '{name}': /move_action unavailable (move_group not running?)")
            return

        goal = MoveGroup.Goal()
        req = goal.request
        start = RobotState()
        start.is_diff = True                            # plan from current state
        req.start_state = start
        req.group_name = f"{arm}_arm"
        req.pipeline_id = pipeline
        req.planner_id = plan_id
        req.num_planning_attempts = 1
        req.allowed_planning_time = plan_time
        req.max_velocity_scaling_factor = vel
        req.max_acceleration_scaling_factor = acc
        req.goal_constraints = [self._pose_constraints(
            f"openarmx_{arm}_link7", frame_id, pos, quat)]
        goal.planning_options.plan_only = False

        res = self._send_and_wait(
            self._mg, goal, plan_time + 30.0,
            label=f"step '{name}' [{plan_id} {arm} @ {frame_id}]")
        if res is None:
            self._logger.error(f"step '{name}': no result / timeout")
            return
        code = getattr(getattr(res, "error_code", None), "val", 0)
        if code == 1:
            self._logger.info(f"step '{name}': OK")
        else:
            self._logger.error(
                f"step '{name}': MoveIt FAILED (error_code={code})")

    @staticmethod
    def _pose_constraints(link: str, frame_id: str, pos, quat,
                          tol_pos: float = 0.001, tol_ang: float = 0.01
                          ) -> Constraints:
        c = Constraints()
        c.name = "pose_goal"
        pc = PositionConstraint()
        pc.header.frame_id = frame_id
        pc.link_name = link
        pc.target_point_offset = Vector3(x=0.0, y=0.0, z=0.0)
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [tol_pos]
        bv = BoundingVolume()
        bv.primitives = [sphere]
        region_pose = Pose()
        region_pose.position.x = float(pos[0])
        region_pose.position.y = float(pos[1])
        region_pose.position.z = float(pos[2])
        region_pose.orientation.w = 1.0
        bv.primitive_poses = [region_pose]
        pc.constraint_region = bv
        pc.weight = 1.0
        c.position_constraints = [pc]
        oc = OrientationConstraint()
        oc.header.frame_id = frame_id
        oc.link_name = link
        oc.orientation = Quaternion(
            x=float(quat[0]), y=float(quat[1]),
            z=float(quat[2]), w=float(quat[3]))
        oc.absolute_x_axis_tolerance = tol_ang
        oc.absolute_y_axis_tolerance = tol_ang
        oc.absolute_z_axis_tolerance = tol_ang
        oc.weight = 1.0
        c.orientation_constraints = [oc]
        return c

    # ---- gripper -----------------------------------------------------
    def _send_gripper(self, step: Dict[str, Any]) -> None:
        name = step.get("name", "?")
        arm = (step.get("arm") or "right").lower()
        client = self._grip.get(arm)
        if client is None:
            self._logger.error(f"step '{name}': unknown gripper arm {arm!r}")
            return
        if not client.wait_for_server(timeout_sec=3.0):
            self._logger.error(f"step '{name}': {arm} gripper action unavailable")
            return
        goal = GripperCommand.Goal()
        goal.command.position = float(step.get("position", 0.0) or 0.0)
        goal.command.max_effort = float(step.get("max_effort", 14.0) or 14.0)
        self._send_and_wait(client, goal, 10.0, label=f"step '{name}' [gripper {arm}]")

    # ---- blocking send (MultiThreadedExecutor-safe) ------------------
    def _send_and_wait(self, client, goal, timeout: float,
                       label: str = "") -> Optional[Any]:
        done = threading.Event()
        box: Dict[str, Any] = {"result": None}

        def on_result(fut):
            try:
                box["result"] = fut.result().result
            except Exception as e:
                self._logger.error(f"{label} result error: {e}")
            finally:
                done.set()

        def on_response(fut):
            try:
                gh = fut.result()
            except Exception as e:
                self._logger.error(f"{label} send error: {e}")
                done.set()
                return
            if not gh.accepted:
                self._logger.error(f"{label} goal rejected")
                done.set()
                return
            gh.get_result_async().add_done_callback(on_result)

        client.send_goal_async(goal).add_done_callback(on_response)
        if not done.wait(timeout):
            self._logger.error(f"{label} timed out after {timeout:.0f}s")
            return None
        return box["result"]


def _clamp(v: float, lo: float = 0.01, hi: float = 1.0) -> float:
    return max(lo, min(hi, v))
