#!/usr/bin/env python3
"""수동 pick '움직이기까지' 지연 측정 — 로봇 미동작(DRY, run=False).

첫 모션 전 비용은 run_pick() 의 IK(중첩 랜덤재시작) 지배 의심. 상주 서버와 동일 코드
(core.PickV2 + run_pick)로 IK 단계별 시간을 잰다(발행/모션 없음).

박스는 최신 /detected_boxes 가 있으면 그것, 없으면 --box=x,y,z(기본=검증된 우팔 박스).

사용: python3 measure_pick_ik_time.py --side=right [--box=0.345,-0.116,0.782]
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy  # noqa: E402
import ptp_pick_seq_v2_left as core  # noqa: E402
import ptp_pick_resident as res  # noqa: E402


def _arg_box():
    for a in sys.argv:
        if a.startswith("--box="):
            x, y, z = (float(v) for v in a.split("=", 1)[1].split(","))
            return (x, y, z)
    return None


def _t(label, fn):
    t0 = time.perf_counter()
    out = fn()
    dt = (time.perf_counter() - t0) * 1000.0
    print(f"  {label:34s} {dt:9.1f} ms")
    return out, dt


def main():
    rclpy.init()
    node = core.PickV2()
    print(f"[{core.SIDE}] 스택 대기(/robot_description + /joint_states)...", flush=True)
    waited = 0.0
    while rclpy.ok() and not (node.urdf and node.state):
        rclpy.spin_once(node, timeout_sec=0.1)
        waited += 0.1
        if waited > 30.0:
            print("스택 미수신 30s — 종료"); return 1
    node.build_model()
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)

    # --box 가 주어지면 강제(override) — 임의 위치에서 refmodel 실패/폴백 재현용.
    arg_box = _arg_box()
    box = arg_box or node.box or (0.345, -0.116, 0.782)
    src = "arg(override)" if arg_box else ("live /detected_boxes" if node.box else "synthetic")
    node.box = box
    rbx, rby, rbz = box
    print(f"[{core.SIDE}] box=({rbx:.3f},{rby:+.3f},{rbz:.3f})  [{src}]", flush=True)

    # 재시작/iter 오버라이드 (모듈 전역을 호출 시점에 읽으므로 즉시 반영)
    for a in sys.argv:
        if a.startswith("--restarts="):
            core.IK_RESTARTS = int(a.split("=", 1)[1])
        if a.startswith("--maxiter="):
            core.IK_MAX_ITER = int(a.split("=", 1)[1])

    ox, oy, oz = core.load_grasp_offset()
    bx, by = rbx + ox, rby + oy
    core.DESCEND_Z = max(core.DESCEND_FLOOR, rbz + oz - core.GRASP_DEPTH)
    seed = node.seed_full()

    print(f"---- IK 단계별 (IK_RESTARTS={core.IK_RESTARTS}, MAX_ITER={core.IK_MAX_ITER}, "
          f"MAX_TRIES={core.MAX_TRIES}, DESCEND_Z={core.DESCEND_Z:.3f}) ----")
    (ref_out), _ = _t("solve_pick_refmodel (run_pick 1순위)",
                      lambda: node.solve_pick_refmodel(bx, by, seed))
    print(f"      -> refmodel ok={ref_out[7]} info={ref_out[6]}")
    (opt_out), _ = _t("solve_pick_optimal (구 빠른 경로)",
                      lambda: node.solve_pick_optimal(bx, by, seed, core.N_CAND))
    print(f"      -> optimal  ok={opt_out[7]} info={opt_out[6]}")
    it_pos, it_R = node.init_tcp_pose()
    mx, my, mz = it_pos[0] + core.MID_X_FWD, it_pos[1], max(float(it_pos[2]), core.MID_MIN_Z)
    _t("solve_pose(중간자세)", lambda: node.solve_pose(mx, my, mz, it_R, seed))
    print("---- 전체 run_pick(run=False) = pick_once 후 첫 모션까지 IK 비용 ----")
    _t("run_pick DRY", lambda: res.run_pick(node, run=False))

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
