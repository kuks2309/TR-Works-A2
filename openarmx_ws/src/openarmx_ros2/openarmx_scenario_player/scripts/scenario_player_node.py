#!/usr/bin/env python3
"""openarmx_scenario_player — stub backend node.

ROS 2 surface (matches the openarmx_scenario_ui contract 1:1):
  Action  : openarmx_scenario_player_msgs/action/PlayScenario  @ 'scenario_player/play'
  Service : openarmx_scenario_player_msgs/srv/ListScenarios    @ 'scenario_player/list'
  Topic   : /scenario_player/status (std_msgs/String JSON, RELIABLE+TRANSIENT_LOCAL depth=10)
  Param   : scenario_search_path (string, default '')

Executable name 'scenario_player_node.py' is hardcoded by the UI
(main_window.PLAYER_RUN_CMD) — do not rename.
"""

from __future__ import annotations

import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from openarmx_scenario_player_msgs.action import PlayScenario
from openarmx_scenario_player_msgs.msg import ScenarioInfo
from openarmx_scenario_player_msgs.srv import ListScenarios

from openarmx_scenario_player.motion_backend_stub import MotionBackendStub
from openarmx_scenario_player.scenario_repo import Scenario, ScenarioRepo
from openarmx_scenario_player.status_publisher import StatusPublisher


STUB_STEP_BASE_SEC = 1.0      # default step length at speed_scale = 1.0
STUB_TICK_SEC      = 0.1      # feedback publish + cancel poll period
STUB_MIN_SPEED     = 0.01     # avoid divide-by-zero on speed_scale = 0


class ScenarioPlayerNode(Node):
    def __init__(self) -> None:
        super().__init__("scenario_player")

        self.declare_parameter("scenario_search_path", "")
        search_path = self.get_parameter("scenario_search_path").value or ""
        self._repo = ScenarioRepo(search_path)
        if search_path:
            self.get_logger().info(f"scenario_search_path = {search_path}")
        else:
            self.get_logger().warn("scenario_search_path is empty — list will be empty")

        self._goal_lock = threading.Lock()
        self._active = False

        cb_group = ReentrantCallbackGroup()

        self._status = StatusPublisher(self, topic="/scenario_player/status")
        self._status.idle()

        self._motion = MotionBackendStub(self)

        self._list_srv = self.create_service(
            ListScenarios, "scenario_player/list",
            self._on_list_scenarios, callback_group=cb_group,
        )
        self._action_srv = ActionServer(
            self, PlayScenario, "scenario_player/play",
            execute_callback=self._execute_play,
            goal_callback=self._on_goal_request,
            cancel_callback=self._on_cancel_request,
            callback_group=cb_group,
        )

    # ------------------------------------------------------------------
    # ListScenarios
    # ------------------------------------------------------------------

    def _on_list_scenarios(self, request, response):
        scenarios = self._repo.scan()
        response.scenarios = [self._to_scenario_info(s) for s in scenarios]
        response.names = [s.name for s in scenarios]
        response.descriptions = [s.description for s in scenarios]
        response.sub_counts = [len(s.sub_names) for s in scenarios]
        response.sub_names = [",".join(s.sub_names) for s in scenarios]
        return response

    @staticmethod
    def _to_scenario_info(s: Scenario) -> ScenarioInfo:
        info = ScenarioInfo()
        info.name = s.name
        info.description = s.description
        info.sub_count = len(s.sub_names)
        info.sub_names = list(s.sub_names)
        info.path = s.path
        return info

    # ------------------------------------------------------------------
    # PlayScenario goal lifecycle
    # ------------------------------------------------------------------

    def _on_goal_request(self, _goal_request) -> GoalResponse:
        with self._goal_lock:
            if self._active:
                return GoalResponse.REJECT
            self._active = True
            return GoalResponse.ACCEPT

    def _on_cancel_request(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute_play(self, goal_handle):
        t0 = time.monotonic()
        result = PlayScenario.Result()
        goal = goal_handle.request

        raw_speed = float(goal.speed_scale) if goal.speed_scale else 1.0
        speed = max(STUB_MIN_SPEED, raw_speed)
        dry_run = bool(goal.dry_run)

        try:
            scenario = self._repo.resolve(goal.scenario_path)
            if scenario is None:
                return self._abort(goal_handle, result,
                                   f"Scenario not found: {goal.scenario_path!r}")
            self._repo.load_subs(scenario)

            start_sub = max(1, int(goal.start_sub) or 1)
            end_sub = int(goal.end_sub)
            if end_sub < 0 or end_sub > len(scenario.subs):
                end_sub = len(scenario.subs)
            if start_sub > end_sub or end_sub == 0:
                return self._abort(goal_handle, result,
                                   f"Empty sub range [{start_sub},{end_sub}]")

            selected = scenario.subs[start_sub - 1:end_sub]
            total_subs = len(selected)
            total_steps = sum(max(1, len(s.steps)) for s in selected)

            self._status.started(scenario.name, total_subs, total_steps)

            steps_completed = 0
            for sub_idx, sub in enumerate(selected, start=1):
                if self._tick_boundary(goal_handle, sub_idx, total_subs, sub,
                                       0, "<start_sub>", "start_sub",
                                       0.0, steps_completed, total_steps, t0):
                    return self._cancel(goal_handle, result, sub.name, "<start_sub>",
                                        "start_sub", steps_completed, total_subs,
                                        total_steps, t0)

                sub_steps = sub.steps if sub.steps else [{"name": sub.name}]
                for step_idx, step_def in enumerate(sub_steps):
                    step_name = self._step_name(step_def, step_idx)
                    base_dur = self._step_duration(step_def)
                    scaled = base_dur / speed

                    self._status.step(sub_idx, sub.name, step_idx, step_name, scaled)

                    # === Phase 1 replacement point ===========================
                    # The sleep loop below stands in for real motion. In Phase 1,
                    # MotionBackendStub.execute_step issues FollowJointTrajectory
                    # (or Pilz LIN via MoveIt) and the wait becomes await-result.
                    # =========================================================
                    if not dry_run:
                        self._motion.execute_step(step_def)

                    elapsed = 0.0
                    while elapsed < scaled:
                        if goal_handle.is_cancel_requested:
                            return self._cancel(goal_handle, result, sub.name, step_name,
                                                "running", steps_completed, total_subs,
                                                total_steps, t0)
                        step_progress = min(1.0, elapsed / scaled) if scaled > 0 else 1.0
                        overall_progress = (steps_completed + step_progress) / max(1, total_steps)
                        self._publish_feedback(goal_handle, sub_idx, total_subs, sub.name,
                                               step_idx, len(sub_steps), step_name,
                                               "running", step_progress, overall_progress,
                                               time.monotonic() - t0)
                        time.sleep(STUB_TICK_SEC)
                        elapsed += STUB_TICK_SEC
                    steps_completed += 1

                if self._tick_boundary(goal_handle, sub_idx, total_subs, sub,
                                       max(0, len(sub_steps) - 1), "<end_sub>", "end_sub",
                                       1.0, steps_completed, total_steps, t0):
                    return self._cancel(goal_handle, result, sub.name, "<end_sub>",
                                        "end_sub", steps_completed, total_subs,
                                        total_steps, t0)

            duration = time.monotonic() - t0
            self._status.completed(scenario.name, total_subs, total_subs,
                                   steps_completed, duration)
            self._status.idle()
            goal_handle.succeed()
            result.success = True
            result.message = "OK"
            result.subs_completed = total_subs
            result.subs_total = total_subs
            result.steps_completed = steps_completed
            result.total_duration_sec = float(duration)
            return result
        finally:
            with self._goal_lock:
                self._active = False

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _step_name(step_def, idx: int) -> str:
        if isinstance(step_def, dict):
            return str(step_def.get("name") or f"step_{idx + 1}")
        return str(step_def)

    @staticmethod
    def _step_duration(step_def) -> float:
        if isinstance(step_def, dict):
            try:
                return float(step_def.get("duration_sec", STUB_STEP_BASE_SEC))
            except (TypeError, ValueError):
                return STUB_STEP_BASE_SEC
        return STUB_STEP_BASE_SEC

    def _publish_feedback(self, goal_handle, current_sub, total_subs, sub_name,
                          current_step, total_steps_in_sub, step_name, phase,
                          step_progress, overall_progress, elapsed_sec) -> None:
        fb = PlayScenario.Feedback()
        fb.current_sub = int(current_sub)
        fb.total_subs = int(total_subs)
        fb.sub_name = str(sub_name)
        fb.current_step = int(current_step)
        fb.total_steps = int(total_steps_in_sub)
        fb.step_name = str(step_name)
        fb.phase = str(phase)
        fb.step_progress = float(step_progress)
        fb.overall_progress = float(overall_progress)
        fb.elapsed_sec = float(elapsed_sec)
        goal_handle.publish_feedback(fb)

    def _tick_boundary(self, goal_handle, sub_idx, total_subs, sub,
                       step_idx, step_name, phase, step_progress,
                       steps_completed, total_steps, t0) -> bool:
        if goal_handle.is_cancel_requested:
            return True
        overall = steps_completed / max(1, total_steps)
        self._publish_feedback(goal_handle, sub_idx, total_subs, sub.name,
                               step_idx, max(1, len(sub.steps)), step_name,
                               phase, step_progress, overall,
                               time.monotonic() - t0)
        return False

    def _cancel(self, goal_handle, result, sub_name, step_name, phase,
                steps_completed, subs_total, total_steps, t0):
        self._status.canceled(sub_name, step_name, phase)
        self._status.idle()
        goal_handle.canceled()
        result.success = False
        result.message = "canceled"
        result.subs_completed = 0
        result.subs_total = subs_total
        result.steps_completed = steps_completed
        result.total_duration_sec = float(time.monotonic() - t0)
        return result

    def _abort(self, goal_handle, result, message: str):
        self._status.failed(step_name="", reason="not_found", message=message)
        self._status.idle()
        goal_handle.abort()
        result.success = False
        result.message = message
        result.subs_completed = 0
        result.subs_total = 0
        result.steps_completed = 0
        result.total_duration_sec = 0.0
        return result


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = ScenarioPlayerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
