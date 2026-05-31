"""Status publisher for openarmx_scenario_player.

Publishes JSON-encoded std_msgs/String events to /scenario_player/status with
QoS = RELIABLE + TRANSIENT_LOCAL (depth=10) so late-joining UIs latch the last
state. Schema (consumed by openarmx_scenario_ui):

  {'event': 'idle'}
  {'event': 'started',   'scenario','subs','total_steps'}
  {'event': 'step',      'sub','sub_name','step','step_name','duration_sec'}
  {'event': 'completed', 'scenario','subs_completed','subs_total',
                          'steps_completed','duration_sec'}
  {'event': 'canceled',  'sub_name','step_name','phase'}
  {'event': 'failed',    'step_name','reason','message'}

`duration_sec` is ALWAYS included in 'step'/'completed' — the UI's '%.1f' /
'%.2f' format calls have no None guard.
"""

from __future__ import annotations

import json

from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import String


class StatusPublisher:
    def __init__(self, node: Node, topic: str = "/scenario_player/status") -> None:
        self._node = node
        qos = QoSProfile(
            depth=10,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub = node.create_publisher(String, topic, qos)

    def _emit(self, payload: dict) -> None:
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._pub.publish(msg)

    def idle(self) -> None:
        self._emit({"event": "idle"})

    def started(self, scenario: str, subs: int, total_steps: int) -> None:
        self._emit({
            "event": "started",
            "scenario": scenario,
            "subs": int(subs),
            "total_steps": int(total_steps),
        })

    def step(self, sub: int, sub_name: str, step: int, step_name: str,
             duration_sec: float) -> None:
        self._emit({
            "event": "step",
            "sub": int(sub),
            "sub_name": sub_name,
            "step": int(step),
            "step_name": step_name,
            "duration_sec": float(duration_sec),
        })

    def completed(self, scenario: str, subs_completed: int, subs_total: int,
                  steps_completed: int, duration_sec: float) -> None:
        self._emit({
            "event": "completed",
            "scenario": scenario,
            "subs_completed": int(subs_completed),
            "subs_total": int(subs_total),
            "steps_completed": int(steps_completed),
            "duration_sec": float(duration_sec),
        })

    def canceled(self, sub_name: str, step_name: str, phase: str) -> None:
        self._emit({
            "event": "canceled",
            "sub_name": sub_name,
            "step_name": step_name,
            "phase": phase,
        })

    def failed(self, step_name: str, reason: str, message: str) -> None:
        self._emit({
            "event": "failed",
            "step_name": step_name,
            "reason": reason,
            "message": message,
        })
