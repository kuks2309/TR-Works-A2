#!/usr/bin/env python3
"""
cyclo MoveL time_from_start 검증 테스트.

가설:
  - cyclo MoveL msg의 time_from_start은 cubic 보간 horizon
  - 짧은 horizon → 빠른 속도 (= start-goal / duration 평균)
  - 긴 horizon → 느린 속도

절차:
  1. before EE pose (body_link0 → openarmx_right_hand_tcp) 측정
  2. target = before + (0, 0, +0.05) (Z +50mm)
  3. publish MoveL with time_from_start = T
  4. T + 1.5초 대기 (도달 + 여유)
  5. after EE pose 측정
  6. ΔZ 출력 → 도달했는지 검증

사용법:
  python3 test_cyclo_movel_velocity.py [duration_sec=2.0]
"""
import sys
import time
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from openarmx_scenario_player_msgs.msg import MoveL


ARM = "left"
BASE_FRAME = "openarmx_body_link0"
TCP_FRAME = f"openarmx_{ARM}_hand_tcp"
TOPIC = f"/openarmx/{ARM}/movel"
# defaults: 20mm Z+
DEFAULT_AXIS = "z"
DEFAULT_SIGN = +1
DEFAULT_MAG = 0.02  # 20 mm


class CycloMoveLProbe(Node):
    def __init__(self):
        super().__init__("test_cyclo_movel_velocity")
        self._tf_buf = Buffer()
        self._tf_listen = TransformListener(self._tf_buf, self)
        self._pub = self.create_publisher(MoveL, TOPIC, 10)

    def lookup_ee(self) -> dict | None:
        for _ in range(50):
            try:
                t = self._tf_buf.lookup_transform(BASE_FRAME, TCP_FRAME, Time())
                return {
                    "x": t.transform.translation.x,
                    "y": t.transform.translation.y,
                    "z": t.transform.translation.z,
                    "qx": t.transform.rotation.x,
                    "qy": t.transform.rotation.y,
                    "qz": t.transform.rotation.z,
                    "qw": t.transform.rotation.w,
                }
            except Exception:
                rclpy.spin_once(self, timeout_sec=0.1)
        return None

    def publish_movel(self, target: dict, duration_sec: float) -> None:
        msg = MoveL()
        msg.pose.header.frame_id = BASE_FRAME
        msg.pose.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(target["x"])
        msg.pose.pose.position.y = float(target["y"])
        msg.pose.pose.position.z = float(target["z"])
        msg.pose.pose.orientation.x = float(target["qx"])
        msg.pose.pose.orientation.y = float(target["qy"])
        msg.pose.pose.orientation.z = float(target["qz"])
        msg.pose.pose.orientation.w = float(target["qw"])
        sec = int(duration_sec)
        msg.time_from_start.sec = sec
        msg.time_from_start.nanosec = int(round((duration_sec - sec) * 1e9))
        self._pub.publish(msg)


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
    axis = sys.argv[2].lower() if len(sys.argv) > 2 else DEFAULT_AXIS
    sign = int(sys.argv[3]) if len(sys.argv) > 3 else DEFAULT_SIGN
    mag = float(sys.argv[4]) if len(sys.argv) > 4 else DEFAULT_MAG
    delta_signed = sign * mag
    print(f"=== test_cyclo_movel_velocity: duration={duration:.3f}s, "
          f"Δ{axis.upper()}={delta_signed*1000:+.0f}mm ===")

    rclpy.init()
    node = CycloMoveLProbe()

    # warmup spin + wait for cyclo subscriber discovery
    print("Waiting for cyclo subscriber on", TOPIC, "...")
    t0 = time.time()
    while time.time() - t0 < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        n_sub = node._pub.get_subscription_count()
        if n_sub >= 1:
            print(f"OK: {n_sub} subscriber(s) discovered after {time.time()-t0:.2f}s")
            break
    else:
        print(f"WARN: no subscriber discovered in 5s (count={node._pub.get_subscription_count()})")

    before = node.lookup_ee()
    if before is None:
        print(f"ERROR: cannot lookup TF {BASE_FRAME} -> {TCP_FRAME}")
        rclpy.shutdown()
        return
    print(f"BEFORE  EE  = ({before['x']:+.4f}, {before['y']:+.4f}, {before['z']:+.4f})")

    target = dict(before)
    target[axis] = before[axis] + delta_signed
    print(f"TARGET      = ({target['x']:+.4f}, {target['y']:+.4f}, {target['z']:+.4f})")
    print(f"EXPECTED v  = {abs(delta_signed)/duration*1000:.1f} mm/s  (= |Δ|/duration)")

    print(f"PUBLISH MoveL once on {TOPIC}")
    node.publish_movel(target, duration)

    # spin while waiting
    t0 = time.time()
    wait = duration + 1.5
    while time.time() - t0 < wait:
        rclpy.spin_once(node, timeout_sec=0.05)

    after = node.lookup_ee()
    if after is None:
        print(f"ERROR: cannot lookup TF after")
        rclpy.shutdown()
        return
    print(f"AFTER   EE  = ({after['x']:+.4f}, {after['y']:+.4f}, {after['z']:+.4f})")

    dx = after["x"] - before["x"]
    dy = after["y"] - before["y"]
    dz = after["z"] - before["z"]
    print(f"DELTA       = ({dx*1000:+.1f}, {dy*1000:+.1f}, {dz*1000:+.1f}) mm")
    d_along = {"x": dx, "y": dy, "z": dz}[axis]
    progress = d_along / delta_signed * 100.0 if delta_signed else 0.0
    print(f"{axis.upper()} progress  = {progress:+.1f}% of commanded {delta_signed*1000:+.0f}mm")
    if abs(d_along - delta_signed) < 0.005:
        print("RESULT: REACHED")
    elif abs(d_along) < 0.001:
        print("RESULT: NO MOTION")
    else:
        print("RESULT: PARTIAL")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
