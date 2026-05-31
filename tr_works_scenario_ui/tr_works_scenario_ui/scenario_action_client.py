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
import threading
from typing import Optional

import rclpy
from PyQt5.QtCore import QObject, pyqtSignal
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

from tr_works_scenario_player.action import PlayScenario
from tr_works_scenario_player.srv import ListScenarios


class ScenarioRosBridge(QObject):
    sig_status_event = pyqtSignal(dict)
    sig_feedback = pyqtSignal(dict)
    sig_result = pyqtSignal(dict)
    sig_error = pyqtSignal(str)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        rclpy.init(args=None)
        self._node: Node = rclpy.create_node("tr_works_scenario_ui")
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
