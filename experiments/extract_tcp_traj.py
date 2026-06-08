#!/usr/bin/env python3
"""rosbag 의 /joint_states 를 FK 해 hand_tcp(TCP) 궤적을 추출 -> CSV + 그래프.

움직인 팔을 자동 판별(관절 변화량 큰 쪽)하고, 그 팔의 hand_tcp 위치/자세(RPY)를
시간순으로 뽑는다. 모델은 ptp_pick_seq_v2_left.PickV2 (라이브 /robot_description) 재사용.

사용: python3 extract_tcp_traj.py [bag경로]   (생략 시 experiments/rosbags/ 최신 pick_*)
"""
import glob
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))


def latest_bag():
    cands = sorted(glob.glob(os.path.join(HERE, "rosbags", "pick_*")), key=os.path.getmtime)
    return cands[-1] if cands else None


def read_joint_states(bag):
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import JointState
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id="sqlite3"), ConverterOptions("", ""))
    out = []
    while r.has_next():
        topic, data, t = r.read_next()
        if topic == "/joint_states":
            m = deserialize_message(data, JointState)
            out.append((t * 1e-9, dict(zip(m.name, m.position))))
    return out


def main():
    bag = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith("--") else latest_bag()
    if not bag or not os.path.exists(bag):
        print("ERROR: bag 없음:", bag); return 1
    print("bag:", bag)
    js = read_joint_states(bag)
    if not js:
        print("ERROR: /joint_states 메시지 없음"); return 1
    print(f"/joint_states {len(js)}개 ({js[-1][0]-js[0][0]:.1f}s)")

    # 움직인 팔 판별(관절 변화량 최대)
    def jrange(side):
        names = [f"openarmx_{side}_joint{i+1}" for i in range(7)]
        arr = np.array([[d.get(n, 0.0) for n in names] for _, d in js])
        return float(np.max(np.ptp(arr, axis=0)))
    side = "right" if jrange("right") >= jrange("left") else "left"
    print(f"움직인 팔: {side} (좌 {np.degrees(jrange('left')):.0f}°, 우 {np.degrees(jrange('right')):.0f}°)")

    # 모델 빌드(라이브 /robot_description) 후 FK
    sys.argv = [sys.argv[0], f"--side={side}"]
    sys.path.insert(0, HERE)
    import rclpy
    import pinocchio as pin
    import ptp_pick_seq_v2_left as core
    rclpy.init()
    node = core.PickV2()
    for _ in range(200):
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.urdf and node.state:
            break
    if not node.urdf:
        print("ERROR: /robot_description 미수신(스택 켜져 있어야 함)"); return 1
    node.build_model()

    t0 = js[0][0]
    rows = []
    for t, d in js:
        q = pin.neutral(node.model)
        for k, qi in enumerate(node.qidx):
            q[qi] = d.get(node.jnames[k], 0.0)
        pin.forwardKinematics(node.model, node.data, q)
        pin.updateFramePlacement(node.model, node.data, node.fid)
        M = node.data.oMf[node.fid]
        p = M.translation
        rpy = np.degrees(pin.rpy.matrixToRpy(M.rotation))
        rows.append((t - t0, p[0], p[1], p[2], rpy[0], rpy[1], rpy[2]))
    rows = np.array(rows)

    csv = os.path.join(HERE, f"tcp_traj_{side}.csv")
    np.savetxt(csv, rows, delimiter=",", header="t,x,y,z,roll,pitch,yaw", comments="")
    print(f"CSV 저장: {csv} ({len(rows)}행)")
    print(f"  X {rows[:,1].min():.3f}~{rows[:,1].max():.3f}  Y {rows[:,2].min():+.3f}~{rows[:,2].max():+.3f}  "
          f"Z {rows[:,3].min():.3f}~{rows[:,3].max():.3f} m")
    print(f"  최저 z(하강 도달) = {rows[:,3].min():.3f} m  (t={rows[rows[:,3].argmin(),0]:.1f}s)")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
        ax[0].plot(rows[:, 0], rows[:, 1], label="X")
        ax[0].plot(rows[:, 0], rows[:, 2], label="Y")
        ax[0].plot(rows[:, 0], rows[:, 3], label="Z", lw=2)
        ax[0].set_ylabel("위치 [m]"); ax[0].legend(); ax[0].grid(True)
        ax[0].set_title(f"hand_tcp 궤적 ({side}) — bag {os.path.basename(bag)}")
        ax[1].plot(rows[:, 0], rows[:, 4], label="Roll")
        ax[1].plot(rows[:, 0], rows[:, 5], label="Pitch")
        ax[1].plot(rows[:, 0], rows[:, 6], label="Yaw")
        ax[1].set_ylabel("자세 [deg]"); ax[1].set_xlabel("시간 [s]"); ax[1].legend(); ax[1].grid(True)
        png = os.path.join(HERE, f"tcp_traj_{side}.png")
        fig.tight_layout(); fig.savefig(png, dpi=110)
        print(f"그래프 저장: {png}")
    except Exception as e:
        print("그래프 생략(matplotlib):", e)

    node.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
