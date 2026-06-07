#!/usr/bin/env python3
"""왼팔 원위치(INIT) 복귀 — action server 없이 JTC 토픽 발행. 그리퍼 미관여(현재 상태 유지)."""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import String  # noqa

SIDE = "left"
JNAMES = [f"openarmx_{SIDE}_joint{i}" for i in range(1, 8)]
INIT_DEG = [50.0, 0.0, 0.0, 100.0, 0.0, 0.0, 50.0]   # joint_data.INIT_POSITION_DEG (left)
MOVE_TIME = 6.0


def main():
    rclpy.init()
    node = rclpy.create_node("go_init_left")
    pub = node.create_publisher(
        JointTrajectory, f"/{SIDE}_joint_trajectory_controller/joint_trajectory", 10)

    def wait(sec):
        end = node.get_clock().now().nanoseconds + int(sec * 1e9)
        while node.get_clock().now().nanoseconds < end:
            rclpy.spin_once(node, timeout_sec=0.05)

    # 컨트롤러 구독 대기
    for _ in range(50):
        if pub.get_subscription_count() > 0:
            break
        wait(0.1)

    traj = JointTrajectory()
    traj.joint_names = JNAMES
    pt = JointTrajectoryPoint()
    pt.positions = [math.radians(d) for d in INIT_DEG]
    pt.time_from_start = Duration(sec=int(MOVE_TIME), nanosec=0)
    traj.points = [pt]
    pub.publish(traj)
    print(f"[init] published INIT (subs={pub.get_subscription_count()}): "
          f"{[f'{d:+.0f}' for d in INIT_DEG]} deg, {MOVE_TIME}s")
    wait(MOVE_TIME + 0.8)

    # 검증: 현재 joint_states
    from sensor_msgs.msg import JointState
    state = {}

    def cb(msg):
        for n, p in zip(msg.name, msg.position):
            state[n] = p
    node.create_subscription(JointState, "/joint_states", cb, 10)
    for _ in range(60):
        rclpy.spin_once(node, timeout_sec=0.1)
        if all(j in state for j in JNAMES):
            break
    print("[verify] 왼팔 INIT 도달:")
    for i, j in enumerate(JNAMES):
        deg = math.degrees(state.get(j, float('nan')))
        print(f"   j{i+1}: {deg:+6.1f} deg (target {INIT_DEG[i]:+.0f})")
    g = state.get(f"openarmx_{SIDE}_finger_joint1", float('nan'))
    print(f"[verify] gripper: {g:.4f} m -> {(1-g/0.044)*100:.0f}% 닫힘 (미관여)")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
