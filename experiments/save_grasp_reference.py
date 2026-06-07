#!/usr/bin/env python3
"""현재(핸드 가이드) 팔 자세 + 검출 박스를 기준 파지자세 데이터셋에 누적 저장.

far-right 등 위치별로 사용자가 손으로 시연한 '잡기 좋은' 접근자세를 모아
(박스 위치 -> 좋은 자세) 매핑 데이터를 만든다. 회전행렬까지 저장(RPY 모호성 회피).

사용:
  python3 save_grasp_reference.py --side=right --note "far-right tilt58"
데이터셋: experiments/<side>_grasp_reference_dataset.yaml  ('poses' 리스트에 append)
최초 실행 시 기존 단일 파일 <side>_grasp_reference_pose.yaml 을 #1 로 마이그레이션.
"""
import os
import sys

import numpy as np
import yaml

_side = "right" if "--side=right" in sys.argv else "left"
if f"--side={_side}" not in sys.argv:
    sys.argv.insert(1, f"--side={_side}")
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import rclpy  # noqa: E402
import pinocchio as pin  # noqa: E402
import ptp_pick_seq_v2_left as core  # noqa: E402


def _note():
    if "--note" in sys.argv:
        i = sys.argv.index("--note")
        if i + 1 < len(sys.argv):
            return sys.argv[i + 1]
    return ""


def main():
    rclpy.init()
    node = core.PickV2()
    for _ in range(200):
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.urdf and node.state:
            break
    if not node.urdf:
        print("ERROR: /robot_description 미수신"); node.destroy_node(); rclpy.shutdown(); return 1
    node.build_model()
    node.wait(0.5)

    q_arm = node.arm_q()
    qfull = node.seed_full()
    pin.forwardKinematics(node.model, node.data, qfull)
    pin.updateFramePlacement(node.model, node.data, node.fid)
    M = node.data.oMf[node.fid]
    pos, R = M.translation.copy(), M.rotation.copy()
    rpy = np.degrees(pin.rpy.matrixToRpy(R))
    tilt = node.gripper_tilt_deg(qfull)
    box = list(node.box) if node.box is not None else None

    entry = {
        "note": _note(),
        "joint_deg": [round(float(np.degrees(a)), 2) for a in q_arm],
        "hand_tcp_pos_m": [round(float(v), 4) for v in pos],
        "hand_tcp_rpy_deg": [round(float(v), 2) for v in rpy],
        "gripper_tilt_deg": round(float(tilt), 2),
        "detected_box_m": ([round(float(v), 4) for v in box] if box else None),
        "hand_tcp_R": [[round(float(R[r, c]), 6) for c in range(3)] for r in range(3)],
    }

    ds_path = os.path.join(HERE, f"{core.SIDE}_grasp_reference_dataset.yaml")
    data = {"side": core.SIDE, "poses": []}
    if os.path.exists(ds_path):
        data = yaml.safe_load(open(ds_path)) or data
    else:
        # 최초: 기존 단일 파일(#1)을 마이그레이션
        single = os.path.join(HERE, f"{core.SIDE}_grasp_reference_pose.yaml")
        if os.path.exists(single):
            s = yaml.safe_load(open(single)) or {}
            data["poses"].append({
                "note": "ref1 첫 시연(검출 박스 미기록)",
                "joint_deg": s.get("joint_deg"),
                "hand_tcp_pos_m": s.get("hand_tcp_pos_m"),
                "hand_tcp_rpy_deg": s.get("hand_tcp_rpy_deg"),
                "gripper_tilt_deg": s.get("gripper_tilt_deg"),
                "detected_box_m": None,
                "hand_tcp_R": s.get("hand_tcp_R"),
            })
    data.setdefault("poses", []).append(entry)
    with open(ds_path, "w") as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)

    print(f"저장 #{len(data['poses'])} (총 {len(data['poses'])}개) -> {ds_path}")
    print(yaml.safe_dump(entry, allow_unicode=True, sort_keys=False))
    node.destroy_node(); rclpy.shutdown(); return 0


if __name__ == "__main__":
    sys.exit(main())
