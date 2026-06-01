#!/usr/bin/env python3
"""Standalone test for ik_check.LinearReachabilityChecker.

Loads the right-arm solver URDF, looks up the current EE pose via TF and the
current joint state via /joint_states, then runs the linear-path check for the
target (current + ΔZ).
"""
import sys
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from ament_index_python.packages import get_package_share_directory
from tf2_ros import Buffer, TransformListener
from sensor_msgs.msg import JointState

sys.path.insert(0, "/home/openarmx/TR-Works/kkw/China/openarmx_ws/src/openarmx_ros2/openarmx_scenario_ui")
from openarmx_scenario_ui.ik_check import LinearReachabilityChecker

ARM = "left"
JOINT_ORDER = [f"openarmx_{ARM}_joint{i}" for i in range(1, 8)]
TCP_FRAME = f"openarmx_{ARM}_hand_tcp"
BASE_FRAME = "openarmx_body_link0"


class Probe(Node):
    def __init__(self):
        super().__init__("ik_check_probe")
        self.tf_buf = Buffer()
        self.tf_listen = TransformListener(self.tf_buf, self)
        self.q = {}
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)

    def _on_js(self, msg):
        for n, p in zip(msg.name, msg.position):
            if n in JOINT_ORDER:
                self.q[n] = float(p)

    def get_ee(self):
        try:
            t = self.tf_buf.lookup_transform(BASE_FRAME, TCP_FRAME, Time())
        except Exception:
            return None
        return dict(
            x=t.transform.translation.x, y=t.transform.translation.y,
            z=t.transform.translation.z,
            qx=t.transform.rotation.x, qy=t.transform.rotation.y,
            qz=t.transform.rotation.z, qw=t.transform.rotation.w)

    def get_q(self):
        if not all(j in self.q for j in JOINT_ORDER):
            return None
        return np.array([self.q[j] for j in JOINT_ORDER])


def main():
    dz = float(sys.argv[1]) if len(sys.argv) > 1 else 0.02
    n_steps = int(sys.argv[2]) if len(sys.argv) > 2 else 10
    axis = sys.argv[3] if len(sys.argv) > 3 else "z"

    share = get_package_share_directory("openarmx_pick")
    urdf = f"{share}/urdf/openarmx_{ARM}_solver.urdf"
    print(f"URDF: {urdf}")

    checker = LinearReachabilityChecker(urdf, TCP_FRAME)
    print(f"model.nq={checker.nq}  q_lo={checker.q_lo}  q_hi={checker.q_hi}")

    rclpy.init()
    node = Probe()
    t0 = time.time()
    while time.time() - t0 < 3.0:
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.get_ee() is not None and node.get_q() is not None:
            break

    ee = node.get_ee()
    q_seed = node.get_q()
    if ee is None or q_seed is None:
        print(f"ERROR ee={ee} q={q_seed}")
        rclpy.shutdown()
        return
    print(f"q_seed = {q_seed}")
    print(f"START  = ({ee['x']:+.4f}, {ee['y']:+.4f}, {ee['z']:+.4f})")

    # Sanity: IK at current pose should converge immediately.
    sanity = checker.check_linear_path(ee, ee, q_seed, n_steps=1)
    print(f"SANITY (start==goal): reachable={sanity.reachable}  reason={sanity.reason}")

    # Real test: ΔZ (or chosen axis).
    goal = dict(ee)
    goal[axis] = ee[axis] + dz
    print(f"GOAL   = ({goal['x']:+.4f}, {goal['y']:+.4f}, {goal['z']:+.4f})  "
          f"(Δ{axis}={dz*1000:+.0f}mm, n_steps={n_steps})")
    res = checker.check_linear_path(ee, goal, q_seed, n_steps=n_steps)
    print(f"RESULT: reachable={res.reachable}  reason={res.reason}  "
          f"failed_at_waypoint={res.waypoint_index}/{res.n_waypoints}")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
