"""ptp_pick_bridge.py — UI <-> 상주 픽 서버(좌/우) 브릿지.

rclpy 노드를 별도 스레드에서 spin(Qt 스레드는 안 막힘). 색을 /pick_color 토픽으로
양 서버에 알리고, 좌/우 상주 서버의 pick_once/auto_start/auto_stop 서비스를 호출하며,
각 서버의 ~/status 를 구독해 Qt 시그널로 UI 에 올린다. (서버 = ptp_pick_resident.py,
노드명 remap: ptp_pick_left / ptp_pick_right)

수동(Manual): 양측 pick_once 1회 — 각 서버가 자기 색·측면(중앙 Y=0 기준) 가장 가까운 박스
              1개를 집음(보통 한쪽만 박스가 있어 1개).
자동(Auto)  : 양측 auto_start/auto_stop — 각 서버가 자기 측면을 연속 picking.
"""
from __future__ import annotations

import threading

from PyQt5.QtCore import QObject, pyqtSignal
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String, Bool

# UI 콤보(한글) -> yolov8 프롬프트
KOR2PROMPT = {
    "빨강": "mini-box-red", "노랑": "mini-box-yellow", "녹색": "mini-box-green",
    "파랑": "mini-box-blue", "주황": "mini-box-orange",
    "자동(전체)": "auto", "전체(모든 색)": "auto",   # 색 무관(모든 색) — 자동 기본
}
SIDES = ("left", "right")


class PtpPickBridge(QObject):
    sig_status = pyqtSignal(str)     # 서버 상태 텍스트 -> UI 라벨

    def __init__(self, parent=None):
        super().__init__(parent)
        if not rclpy.ok():
            rclpy.init()
        self._node = Node("ptp_pick_ui_bridge")
        self._color_pub = self._node.create_publisher(String, "/pick_color", 10)
        self._dual_pub = self._node.create_publisher(Bool, "/allow_dual_arm", 10)
        self._pick, self._auto_on, self._auto_off = {}, {}, {}
        for s in SIDES:
            self._pick[s] = self._node.create_client(Trigger, f"/ptp_pick_{s}/pick_once")
            self._auto_on[s] = self._node.create_client(Trigger, f"/ptp_pick_{s}/auto_start")
            self._auto_off[s] = self._node.create_client(Trigger, f"/ptp_pick_{s}/auto_stop")
            self._node.create_subscription(
                String, f"/ptp_pick_{s}/status",
                lambda m, sd=s: self.sig_status.emit(f"[{sd}] {m.data}"), 10)
        self._exec = SingleThreadedExecutor()
        self._exec.add_node(self._node)
        self._thread = threading.Thread(target=self._exec.spin, daemon=True)
        self._thread.start()

    # ---- 명령 (Qt 스레드에서 호출; call_async 는 비차단, executor 스레드가 처리) ----
    def set_color(self, kor: str):
        self._color_pub.publish(String(data=KOR2PROMPT.get(kor, "mini-box-red")))

    def set_dual_arm(self, allow: bool):
        """양팔 동시 구동 허용 토글. True=동시, False=단일팔(충돌방지 뮤텍스)."""
        self._dual_pub.publish(Bool(data=bool(allow)))
        self.sig_status.emit("양팔 동시 구동 허용 ON" if allow else "단일팔(충돌방지) 모드")

    def _call_side(self, clients, side: str = "both") -> int:
        sides = SIDES if side == "both" else (side,)
        n = 0
        for s in sides:
            c = clients.get(s)
            if c is None:
                continue
            if c.service_is_ready() or c.wait_for_service(timeout_sec=0.5):
                c.call_async(Trigger.Request()); n += 1
        return n

    def _call_both(self, clients) -> int:
        return self._call_side(clients, "both")

    def manual_pick(self, kor: str, side: str = "both"):
        self.set_color(kor)
        n = self._call_side(self._pick, side)
        sn = {"left": "좌", "right": "우", "both": "양"}.get(side, side)
        self.sig_status.emit(
            f"수동 명령 전송 [{sn}팔] ({n}서버)" if n else "픽 서버 없음 — 서버를 띄우세요")

    def auto_start(self, kor: str):
        self.set_color(kor)
        n = self._call_both(self._auto_on)
        self.sig_status.emit(f"자동 시작 ({n}서버)" if n else "픽 서버 없음")

    def auto_stop(self):
        n = self._call_both(self._auto_off)
        self.sig_status.emit(f"자동 정지 ({n}서버)")

    def live_servers(self):
        return [s for s in SIDES if self._pick[s].service_is_ready()]

    def shutdown(self):
        try:
            self._exec.shutdown()
            self._node.destroy_node()
        except Exception:
            pass
