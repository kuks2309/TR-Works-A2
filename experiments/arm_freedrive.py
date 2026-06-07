#!/usr/bin/env python3
"""한 팔: 홈(INIT) 이동 / 중력보상 자유 구동 ON / 자유 구동 OFF(홈 복귀).

** 안전 정책: 자유 구동은 반드시 홈(INIT)에서 시작하고, 해제 시 반드시 홈으로 복귀한다. **

원리(MIT 모드): tau = kp*(pos_cmd-pos) + kd*(vel_cmd-vel) + tau_ff.
  - tau_ff(중력보상)= {side}_forward_effort_controller -> 팔이 안 떨어짐(자유 구동 중에도 유지).
  - kp(위치 강성)=0 + {side}_joint_trajectory_controller 비활성 -> 손으로 자유 이동.

모드:
  --home    : 홈(INIT)으로 이동. 필요 시 위치 제어 복원(JTC 활성+kp) 후 INIT 궤적 발행.
  --enable  : 자유 구동 ON. **홈에 있을 때만 허용**(아니면 중단; 먼저 --home).
              JTC 비활성 -> kp=0. (중력보상 effort active 필수)
  --restore : 자유 구동 OFF + **홈 복귀**. JTC 활성(현재자세 캡처) -> kp 복원 -> INIT 이동.

사용:
  python3 arm_freedrive.py --side right --home
  python3 arm_freedrive.py --side right --enable
  python3 arm_freedrive.py --side right --restore
"""
import math
import sys

import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import SwitchController, ListControllers
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration

KP_FACTORY = [50.0, 50.0, 50.0, 50.0, 10.0, 10.0, 10.0]   # j1..j7 복원값
KP_FREE = [0.0] * 7
HOME_TOL_DEG = 10.0     # 홈 도달/홈 여부 판정 허용 오차(무적분 droop 포함)
MOVE_T = 4.0            # 홈 이동 시간[s]


def parse(a):
    side = "right" if "--side=right" in a or ("--side" in a and a[a.index("--side") + 1] == "right") \
        else ("left" if "--side=left" in a or ("--side" in a and a[a.index("--side") + 1] == "left") else None)
    mode = next((m for m in ("home", "enable", "restore") if f"--{m}" in a), None)
    return side, mode


def call(node, client, req, timeout=5.0):
    if not client.wait_for_service(timeout_sec=timeout):
        raise RuntimeError(f"service 없음: {client.srv_name}")
    fut = client.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    return fut.result()


def ctrl_state(node, name):
    res = call(node, node.create_client(ListControllers, "/controller_manager/list_controllers"),
               ListControllers.Request())
    for c in (res.controller if res else []):
        if c.name == name:
            return c.state
    return None


def switch(node, activate=None, deactivate=None):
    req = SwitchController.Request()
    req.activate_controllers = activate or []
    req.deactivate_controllers = deactivate or []
    req.strictness = SwitchController.Request.STRICT
    req.activate_asap = True
    res = call(node, node.create_client(SwitchController, "/controller_manager/switch_controller"), req)
    return bool(res and res.ok)


def set_kp(node, side, vals):
    req = SetParameters.Request()
    req.parameters = [Parameter(name=f"kp_joint{i+1}",
                                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(v)))
                      for i, v in enumerate(vals)]
    res = call(node, node.create_client(SetParameters, f"/openarmx_{side}_hardware_params/set_parameters"), req)
    return bool(res and all(r.successful for r in res.results))


def read_joints(node, st, side, spins=40):
    for _ in range(spins):
        rclpy.spin_once(node, timeout_sec=0.05)
    return [math.degrees(st.get(f"openarmx_{side}_joint{i+1}", 0.0)) for i in range(7)]


def goto_home(node, side, init_deg, st, t=MOVE_T):
    pub = node.create_publisher(JointTrajectory, f"/{side}_joint_trajectory_controller/joint_trajectory", 10)
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)   # 퍼블리셔 매칭
    jt = JointTrajectory()
    jt.joint_names = [f"openarmx_{side}_joint{i+1}" for i in range(7)]
    pt = JointTrajectoryPoint()
    pt.positions = [math.radians(v) for v in init_deg]
    pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
    jt.points = [pt]
    pub.publish(jt)
    for _ in range(int((t + 1.5) / 0.05)):
        rclpy.spin_once(node, timeout_sec=0.05)
    cur = read_joints(node, st, side, spins=10)
    err = max(abs(c - h) for c, h in zip(cur, init_deg))
    return cur, err


def main():
    side, mode = parse(sys.argv)
    if side is None or mode is None:
        print("사용: --side {left|right} {--home|--enable|--restore}"); return 2

    # INIT_DEG(SSOT, 측면화)를 pick 스크립트에서 가져옴
    sys.argv = [sys.argv[0], f"--side={side}"]
    import os
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import ptp_pick_seq_v2_left as core
    INIT_DEG = core.INIT_DEG

    jtc = f"{side}_joint_trajectory_controller"
    eff = f"{side}_forward_effort_controller"
    rclpy.init(); n = Node("arm_freedrive")
    st = {}
    n.create_subscription(JointState, "/joint_states",
                          lambda m: [st.__setitem__(k, p) for k, p in zip(m.name, m.position)], 10)
    try:
        if mode == "home":
            print(f"[{side}] 홈(INIT) 이동")
            if ctrl_state(n, jtc) != "active":
                print(f"  - {jtc} 활성:", "ok" if switch(n, activate=[jtc]) else "FAIL")
            print(f"  - kp 복원:", "ok" if set_kp(n, side, KP_FACTORY) else "FAIL")
            cur, err = goto_home(n, side, INIT_DEG, st)
            print(f"  -> 도달 {[round(c,1) for c in cur]}  목표 {INIT_DEG}  최대오차 {err:.1f}°")
            print("  " + ("홈 도달 OK" if err <= HOME_TOL_DEG else f"!! 홈 미도달(오차 {err:.1f}°>{HOME_TOL_DEG})"))

        elif mode == "enable":
            if ctrl_state(n, eff) != "active":
                print(f"!! 중단: {eff}(중력보상) active 아님 -> kp=0 시 추락. 중력보상 먼저 켜세요."); return 1
            cur = read_joints(n, st, side)
            err = max(abs(c - h) for c, h in zip(cur, INIT_DEG))
            if err > HOME_TOL_DEG:
                print(f"!! 중단: 홈이 아님(최대오차 {err:.1f}°>{HOME_TOL_DEG}). "
                      f"자유 구동은 반드시 홈에서 시작 -> 먼저 `--home`."); return 1
            print(f"[{side}] 자유 구동 ON (홈 확인 오차 {err:.1f}°)")
            print(f"  1) {jtc} 비활성:", "ok" if switch(n, deactivate=[jtc]) else "FAIL(이미 비활성?)")
            print(f"  2) kp_joint1..7=0:", "ok" if set_kp(n, side, KP_FREE) else "FAIL")
            print("  -> 손으로 자유 이동. 중력보상이 받쳐 줌(약간 처짐). 끝나면 `--restore`(홈 복귀).")

        else:  # restore
            print(f"[{side}] 자유 구동 OFF + 홈 복귀")
            print(f"  1) {jtc} 활성(현재자세 캡처):", "ok" if switch(n, activate=[jtc]) else "FAIL")
            print(f"  2) kp 복원 {KP_FACTORY}:", "ok" if set_kp(n, side, KP_FACTORY) else "FAIL")
            cur, err = goto_home(n, side, INIT_DEG, st)
            print(f"  3) 홈 이동 -> 도달 {[round(c,1) for c in cur]}  최대오차 {err:.1f}°")
            print("  " + ("홈 복귀 OK" if err <= HOME_TOL_DEG else f"!! 홈 미도달(오차 {err:.1f}°)"))
        return 0
    finally:
        n.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
