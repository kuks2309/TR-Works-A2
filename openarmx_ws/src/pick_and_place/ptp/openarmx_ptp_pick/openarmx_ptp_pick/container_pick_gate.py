#!/usr/bin/env python3
"""컨테이너 게이트 노드 — /place_box/info(거리·색)로 '무엇을 집을지' 판단해 /pick_color 발행.

판단·발행은 이 노드가 하고, UI(User Interface)는 표시(수신)만 한다. box_detect_loop 와
좌/우 resident 는 기존대로 /pick_color 를 수신만 한다(변경 없음).

판단 로직:
  - 컨테이너 '있음' = 거리/기하로 안정 판정(color 신뢰도 낮으므로 거리로):
    JSON 의 ok && wall.ok && tof.present && dist_min<=front_distance<=dist_max 가
    presence_frames 연속이면 '안정 있음'. 한 번이라도 깨지면 리셋.
  - 모드(param mode):
      agnostic    : 있음이면 /pick_color="auto"(색무관, 가까운 미니박스), 없으면 hold.
      color_follow: 있음이면 색을 '시간 다수결'로 lock 해 /pick_color="mini-box-{색}".
                    아직 미확정이면 hold(오집기 방지). 단발 오분류는 다수결에서 묻힌다.
  - 시간 다수결(color_follow): 안정-있음 동안 confidence>=conf_min 인 color.name 만 최근
    window 프레임 모아, 최빈 색이 majority_frac 이상이면 그 색 lock. 미달이면 직전 lock 유지.
    컨테이너가 사라지면 투표 초기화(다음 컨테이너는 새로 투표).

발행:
  /pick_color (std_msgs/String): "auto" | "mini-box-{색}" | hold_value(없음/미확정)
  ~/status   (std_msgs/String) : 사람이 읽는 상태(UI 표시용)

사용: python3 container_pick_gate.py [--ros-args -p mode:=color_follow]
(스택 + place_box_detection_node 가 떠 /place_box/info 발행 중이어야 함)
"""
import json
from collections import Counter, deque

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy,
                       QoSHistoryPolicy)
from std_msgs.msg import String, Bool


class ContainerPickGate(Node):
    def __init__(self):
        super().__init__("container_pick_gate")
        # --- 파라미터(런타임 조정 가능) ---
        self.mode = self.declare_parameter("mode", "color_follow").value   # color_follow|agnostic
        self.dist_min = self.declare_parameter("dist_min", 0.10).value     # 컨테이너 거리 하한[m]
        self.dist_max = self.declare_parameter("dist_max", 1.00).value     # 상한[m]
        self.presence_frames = int(self.declare_parameter("presence_frames", 5).value)
        self.window = int(self.declare_parameter("window", 20).value)      # 색 다수결 윈도우
        self.conf_min = self.declare_parameter("conf_min", 0.60).value     # 투표 참여 최소 confidence
        self.majority_frac = self.declare_parameter("majority_frac", 0.70).value
        self.publish_hz = self.declare_parameter("publish_hz", 5.0).value
        self.hold_value = self.declare_parameter("hold_value", "none").value  # 없음/미확정 시 발행값
        self.color_prefix = self.declare_parameter("color_prefix", "mini-box-").value
        # gated=True: '컨테이너 자동' 활성(/container_follow_active=True)일 때만 /pick_color 발행
        # (수동·타 색 자동과 충돌 방지). gated=False: 항상 구동(단독 테스트용 기본).
        self._gated = bool(self.declare_parameter("gated", False).value)
        self._active = not self._gated   # gated 면 활성 신호 받기 전까지 비활성

        # --- 상태 ---
        self._present_run = 0          # 연속 '있음' 프레임 수
        self._stable = False           # 안정-있음 여부
        self._dist = None              # 최근 front_distance(표시용)
        self._votes = deque(maxlen=self.window)   # 최근 색 투표(conf>=conf_min)
        self._lock = None              # 확정된 색(bare, 예: 'green')
        self._lock_n = (0, 0)          # (최빈 표수, 총 표수) 표시용

        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(String, "/place_box/info", self._on_info, 10)
        self.create_subscription(Bool, "/container_follow_active", self._on_active, latched)
        # /pick_color 는 resident 가 LATCHED(TRANSIENT_LOCAL)로 구독 → 발행도 latched 여야 호환.
        self.pub_color = self.create_publisher(String, "/pick_color", latched)
        self.pub_status = self.create_publisher(String, "~/status", 10)
        self.create_timer(1.0 / max(1.0, self.publish_hz), self._tick)
        print(f"[container_gate] 시작 mode={self.mode} dist=[{self.dist_min},{self.dist_max}] "
              f"window={self.window} conf>={self.conf_min} majority>={self.majority_frac}", flush=True)

    # ---------------- /place_box/info 수신: 거리 게이트 + 색 투표 누적 ----------------
    def _on_info(self, msg):
        try:
            d = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        wall = d.get("wall") or {}
        tof = d.get("tof") or {}
        color = d.get("color") or {}
        fd = wall.get("front_distance")
        present_now = bool(
            d.get("ok") and wall.get("ok") and tof.get("present")
            and isinstance(fd, (int, float)) and self.dist_min <= fd <= self.dist_max)

        if present_now:
            self._present_run += 1
            self._dist = fd
            # 안정-있음 동안만 색 투표(confidence 임계 통과분만)
            if self._present_run >= self.presence_frames:
                self._stable = True
                name = color.get("name")
                conf = color.get("confidence") or 0.0
                if name and name not in ("?", "", "unknown") and conf >= self.conf_min:
                    self._votes.append(self._strip_prefix(name))
                self._update_lock()
        else:
            # 컨테이너 사라짐 -> 안정·투표·lock 초기화(다음 컨테이너는 새로 투표)
            self._present_run = 0
            self._stable = False
            self._dist = None
            self._votes.clear()
            self._lock = None
            self._lock_n = (0, 0)

    def _strip_prefix(self, name):
        """color.name 이 'green'/'mini-box-green' 어느 쪽이든 bare('green')로 정규화."""
        return name[len(self.color_prefix):] if name.startswith(self.color_prefix) else name

    def _update_lock(self):
        """최근 윈도우의 색 다수결: 최빈 색이 majority_frac 이상이면 lock(아니면 직전 유지)."""
        if not self._votes:
            return
        cnt = Counter(self._votes)
        top, n = cnt.most_common(1)[0]
        total = len(self._votes)
        if total >= self.window and (n / total) >= self.majority_frac:
            self._lock = top
            self._lock_n = (n, total)

    def _on_active(self, m):
        """브릿지(drive 계층)가 '컨테이너 자동' 활성/비활성을 알림. gated 일 때만 반영."""
        if self._gated:
            self._active = bool(m.data)

    # ---------------- 발행: 판단 결과를 /pick_color + 상태로 ----------------
    def _tick(self):
        if not self._active:
            # 비활성(수동/타 색 자동 등) — /pick_color 양보(발행 안 함), 상태만 표시.
            self.pub_status.publish(String(data="비활성 — 컨테이너 자동 모드 대기"))
            return
        if not self._stable:
            self._emit(self.hold_value, "컨테이너 없음 — 대기(hold)")
            return
        dist_s = f"{self._dist:.2f}m" if isinstance(self._dist, (int, float)) else "?"
        if self.mode == "agnostic":
            self._emit("auto", f"컨테이너 있음(d={dist_s}) — 색무관 auto")
            return
        # color_follow
        if self._lock:
            self._emit(self.color_prefix + self._lock,
                       f"컨테이너 있음(d={dist_s}) 색={self._lock} "
                       f"lock(투표 {self._lock_n[0]}/{self._lock_n[1]}) → {self.color_prefix}{self._lock}")
        else:
            self._emit(self.hold_value,
                       f"컨테이너 있음(d={dist_s}) — 색 확정 대기(투표 {len(self._votes)}/{self.window})")

    def _emit(self, color_value, status):
        self.pub_color.publish(String(data=str(color_value)))
        self.pub_status.publish(String(data=status))


def main():
    rclpy.init()
    node = ContainerPickGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
