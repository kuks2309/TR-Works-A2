#!/usr/bin/env python3
"""오른팔 pick 시퀀스 (붉은 박스) — ptp_pick_seq_v2_left.py 를 --side=right 로 구동.

왼팔과 동일 로직/단계(검출은 별도 트리거: 빨강), 측면 차이는 자동 처리:
  - INIT 자세: 우 j1=-50/j7=-50 (INIT_DEG 측면화)
  - 컨트롤러/프레임: right_* (SIDE f-string)
  - 중간자세: init_tcp_pose() 기반이라 INIT Y(-0.17) 자동 미러
  - 설정: pick_seq_config_right.yaml (오프셋/높이 별도 튜닝)
좌/우 별도 실행. (왼팔: ptp_pick_seq_v2_left.py, 오른팔: 이 스크립트)

검출은 이 스크립트 실행 전에 빨강으로 트리거:
  ros2 action send_goal /yolov8_node/detect yolov8_detection_msgs/action/DetectBox \
    "{prompts: 'mini-box-red', confidence: 0.5, publish_annotated: true}"
"""
import os
import sys

# 모듈 임포트 전에 --side=right 를 argv 에 주입 (모듈 레벨 SIDE 가 이를 읽음)
if "--side=right" not in sys.argv:
    sys.argv.insert(1, "--side=right")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import ptp_pick_seq_v2_left as core  # noqa: E402  (argv 주입 후 임포트)

if __name__ == "__main__":
    sys.exit(core.main())
