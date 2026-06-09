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
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy,
                       QoSHistoryPolicy)
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
        # /pick_color·/container_follow_active 공용 latched QoS. resident 구독이
        # TRANSIENT_LOCAL(LATCHED)이라 발행도 TRANSIENT_LOCAL 이어야 durability 호환 +
        # 늦게/재시작한 resident 도 마지막 색 즉시 수신(VOLATILE 발행 시 resident 미수신).
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             history=QoSHistoryPolicy.KEEP_LAST)
        self._color_pub = self._node.create_publisher(String, "/pick_color", latched)
        self._dual_pub = self._node.create_publisher(Bool, "/allow_dual_arm", 10)
        self._follow_pub = self._node.create_publisher(Bool, "/container_follow_active", latched)
        self._follow_pub.publish(Bool(data=False))   # 기본 비활성(수동/타 발행자에 /pick_color 양보)
        self._pick, self._auto_on, self._auto_off = {}, {}, {}
        for s in SIDES:
            self._pick[s] = self._node.create_client(Trigger, f"/ptp_pick_{s}/pick_once")
            self._auto_on[s] = self._node.create_client(Trigger, f"/ptp_pick_{s}/auto_start")
            self._auto_off[s] = self._node.create_client(Trigger, f"/ptp_pick_{s}/auto_stop")
            self._node.create_subscription(
                String, f"/ptp_pick_{s}/status",
                lambda m, sd=s: self.sig_status.emit(f"[{sd}] {m.data}"), 10)
        # 컨테이너 게이트 상태 -> UI 표시(수신만)
        self._node.create_subscription(
            String, "/container_pick_gate/status",
            lambda m: self.sig_status.emit(f"[컨테이너] {m.data}"), 10)
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

    def _set_follow(self, active: bool):
        """컨테이너 자동(게이트가 /pick_color 구동) 활성/비활성 신호. 활성이면 UI/브릿지는
        /pick_color 를 발행하지 않고(게이트에 양보), 비활성이면 게이트가 양보한다."""
        self._follow_pub.publish(Bool(data=bool(active)))

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
        self._set_follow(False)        # 수동: 게이트 양보, UI 가 색 지정
        self.set_color(kor)
        n = self._call_side(self._pick, side)
        sn = {"left": "좌", "right": "우", "both": "양"}.get(side, side)
        self.sig_status.emit(
            f"수동 명령 전송 [{sn}팔] ({n}서버)" if n else "픽 서버 없음 — 서버를 띄우세요")

    def auto_start(self, kor: str):
        """색 지정 자동(전체/특정 색) — UI 가 /pick_color 를 직접 지정. 게이트는 양보."""
        self._set_follow(False)
        self.set_color(kor)
        n = self._call_both(self._auto_on)
        self.sig_status.emit(f"자동 시작 ({n}서버)" if n else "픽 서버 없음")

    def auto_start_container(self):
        """컨테이너 색 추종 자동 — UI 는 /pick_color 미발행. container_pick_gate 가 거리 게이트+
        색 다수결로 /pick_color 를 몬다. 여기선 활성 신호만 켜고 resident 자동 루프를 켠다."""
        self._set_follow(True)
        n = self._call_both(self._auto_on)
        self.sig_status.emit(
            f"컨테이너 자동 시작 ({n}서버) — 게이트가 거리·색 판단" if n else "픽 서버 없음")

    def auto_stop(self):
        self._set_follow(False)        # 게이트 비활성
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
