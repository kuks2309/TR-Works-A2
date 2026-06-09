#!/usr/bin/env python3
"""검출 루프 노드 — 검출과 이동을 '분리'한다.

/pick_color 의 색을 yolov8 액션으로 '연속' 검출해 /detected_boxes 를 항상 최신으로 유지한다.
좌/우 resident(ptp_pick_resident.py)는 더 이상 픽마다 검출을 트리거하지 않고, 이 노드가
갱신한 최신 /detected_boxes 만 소비한다 -> 픽이 검출을 기다리지 않아 전환이 빠르고,
양팔이 각자 검출하던 경합(yolo 액션 직렬화)도 사라진다.

비차단(콜백 기반): 타이머가 직전 검출이 끝났을 때만 다음 goal 을 보낸다(쉼없이 연속).
사용: python3 box_detect_loop.py   (스택 + yolov8_node 가 떠 있어야 함)
"""
import fcntl
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from yolov8_detection_msgs.action import DetectBox

ALL_COLORS = ["mini-box-red", "mini-box-yellow", "mini-box-green",
              "mini-box-blue", "mini-box-orange"]
# 검출 게이팅: 어느 팔이든 모션 중이면(resident 가 SH, C++ Hover 가 EX 보유) 검출을 멈춘다.
# = GPU 절약 + 움직이는 팔이 카메라를 가려 생기는 오검출 방지(sense-plan-act).
MOTION_PATH_LOCK = "/tmp/openarmx_motion_path.lock"


class DetectLoop(Node):
    def __init__(self):
        super().__init__("box_detect_loop")
        self._color = "mini-box-red"           # /pick_color 로 UI 가 갱신
        self._busy = False
        self._n = 0
        self._skipped = 0                       # 모션 중이라 건너뛴 검출 틱 수
        try:                                    # 검출 게이팅용 경로 락 핸들
            self._path_fd = open(MOTION_PATH_LOCK, "w")
        except OSError as e:                    # /tmp 실패 → 게이팅 비활성(검출 계속; box_detect는 모션 아님이라 안전)
            self._path_fd = None
            print(f"[detect_loop] 경로 락 열기 실패({e}) — 게이팅 비활성", flush=True)
        self.create_subscription(String, "/pick_color", self._on_color, 10)
        self.cl = ActionClient(self, DetectBox, "/yolov8_node/detect")
        self.create_timer(0.05, self._tick)    # 직전 검출 끝나면 즉시 다음(연속)
        print("[detect_loop] 시작 — /pick_color 색을 연속 검출", flush=True)

    def _on_color(self, m):
        if m.data:
            self._color = m.data

    def _motion_in_progress(self):
        """경로 락(/tmp/openarmx_motion_path.lock)을 EX 비차단 시험 획득.
        성공=아무도 모션 안 함(즉시 해제하고 검출), 실패=어느 팔이 모션 중 → 검출 생략.
        resident 는 모션 동안 SH 를 잡으므로 모션 중이면 EX 시도가 실패한다. 락은 검출
        밖에서만 잠깐 잡았다 바로 풀어(굶주림 방지) resident 의 SH 획득을 막지 않는다."""
        if self._path_fd is None:               # 락 열기 실패 → 게이팅 불가(검출 계속)
            return False
        try:
            fcntl.flock(self._path_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.flock(self._path_fd, fcntl.LOCK_UN)
            return False
        except (BlockingIOError, OSError):
            return True

    def _tick(self):
        if self._busy or not self.cl.server_is_ready():
            return
        # [검출 게이팅] 어느 팔이든 모션 중이면 이번 틱 검출 생략(모션 끝나면 자동 재개).
        if self._motion_in_progress():
            self._skipped += 1
            if self._skipped % 100 == 0:
                print(f"[detect_loop] 모션 중 — 검출 게이팅(생략 {self._skipped})", flush=True)
            return
        self._busy = True
        prompt = ",".join(ALL_COLORS) if self._color == "auto" else self._color
        g = DetectBox.Goal()
        g.prompts = prompt
        g.confidence = 0.4
        g.publish_annotated = True
        self.cl.send_goal_async(g).add_done_callback(self._on_goal)

    def _on_goal(self, fut):
        gh = fut.result()
        if not gh or not gh.accepted:
            self._busy = False
            return
        gh.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, _fut):
        self._busy = False
        self._n += 1
        if self._n % 50 == 0:
            print(f"[detect_loop] 검출 {self._n}회 (색={self._color})", flush=True)


def main():
    rclpy.init()
    node = DetectLoop()
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
