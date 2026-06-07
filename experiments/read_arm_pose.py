#!/usr/bin/env python3
"""현재(핸드 가이드로 잡아 둔) 팔 자세 읽기 — 관절각 + hand_tcp 위치/RPY/tilt.

ptp_pick_seq_v2_left.py 의 모델/FK 로직을 그대로 재사용(SIDE f-string).
사용:
  python3 read_arm_pose.py --side=right   # 오른팔
  python3 read_arm_pose.py                 # 왼팔(기본)
"""
import os
import sys

import numpy as np

_side = "right" if "--side=right" in sys.argv else "left"
if f"--side={_side}" not in sys.argv:
    sys.argv.insert(1, f"--side={_side}")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy  # noqa: E402
import ptp_pick_seq_v2_left as core  # noqa: E402  (argv 주입 후 임포트)


def main():
    rclpy.init()
    node = core.PickV2()
    for _ in range(200):                       # /robot_description + /joint_states 대기
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.urdf and node.state:
            break
    if not node.urdf:
        print("ERROR: /robot_description 미수신"); node.destroy_node(); rclpy.shutdown(); return 1
    node.build_model()
    node.wait(0.5)                             # 관절각 안정화

    q_arm = node.arm_q()                       # 측정 7관절(rad) j1..j7
    qfull = node.seed_full()                   # 현재 측정값으로 채운 reduced 모델 q
    pos, rpy = node.grasp_pose_rpy(qfull)      # hand_tcp 위치[m] + RPY[deg]
    tilt = node.gripper_tilt_deg(qfull)        # 수직(-Z)에서 벗어난 각[deg]

    print(f"=== {core.SIDE} 팔 현재 자세 (핸드 가이드) ===")
    print(f"관절각(deg) j1..j7 = {[f'{np.degrees(a):+.1f}' for a in q_arm]}")
    print(f"hand_tcp 위치(m)  = ({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})")
    print(f"hand_tcp RPY(deg) = (R {rpy[0]:+.1f}, P {rpy[1]:+.1f}, Y {rpy[2]:+.1f})")
    print(f"그리퍼 기울기 tilt = {tilt:.1f}° (수직=0°; 적당히 비스듬이 파지에 유리)")
    node.destroy_node(); rclpy.shutdown(); return 0


if __name__ == "__main__":
    sys.exit(main())
