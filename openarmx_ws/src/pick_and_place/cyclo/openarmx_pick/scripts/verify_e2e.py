#!/usr/bin/env python3
"""End-to-end capstone: synthetic box -> grasp_pose_node -> MoveL -> QP solver.

Run alongside the solver + grasp_pose_node(auto_send:=true). This script:
  * simulates the robot: publishes /joint_states (home), then echoes each
    commanded joint_trajectory point back as the next measured state, and
  * publishes a synthetic box-top cloud on /box_plane/cloud (base frame), and
  * reads the solver's current EE pose (/openarmx/left_ee_pose) and reports how
    close it converges to the published grasp pose (/openarmx/grasp_pose,
    lifted by the pre-grasp height since auto_send sends the PRE-grasp pose).

Verdict PASS when the commanded EE reaches within tolerance of the pre-grasp XY.
"""
import struct

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, PointCloud2, PointField
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory

LEFT = [f"openarmx_left_joint{i}" for i in range(1, 8)]
BASE = "openarmx_body_link0"
CX, CY, CZ = 0.28, 0.14, 0.10
PRE_H = 0.10


def make_cloud(stamp):
    xs = np.arange(-0.04, 0.04, 0.004)
    ys = np.arange(-0.08, 0.08, 0.004)
    pts = [(CX + x, CY + y, CZ) for x in xs for y in ys]
    m = PointCloud2()
    m.header = Header(frame_id=BASE); m.header.stamp = stamp
    m.height = 1; m.width = len(pts)
    m.fields = [PointField(name=n, offset=o, datatype=PointField.FLOAT32, count=1)
                for n, o in (("x", 0), ("y", 4), ("z", 8))]
    m.is_bigendian = False; m.point_step = 12; m.row_step = 12 * len(pts)
    m.data = b"".join(struct.pack("<fff", *p) for p in pts); m.is_dense = True
    return m


class E2E(Node):
    def __init__(self):
        super().__init__("e2e_checker")
        self.q = [0.0] * 7
        self.ee = None
        self.grasp = None
        self.js = self.create_publisher(JointState, "/joint_states", 10)
        self.cloud = self.create_publisher(PointCloud2, "/box_plane/cloud", 5)
        self.create_subscription(JointTrajectory, "/openarmx/left_arm/joint_trajectory",
                                 self._on_cmd, 10)
        self.create_subscription(PoseStamped, "/openarmx/left_ee_pose", self._on_ee, 10)
        self.create_subscription(PoseStamped, "/openarmx/grasp_pose", self._on_grasp, 5)
        self.create_timer(0.01, self._js_tick)
        self.create_timer(0.2, self._cloud_tick)
        self.create_timer(1.0, self._report)
        self._t = 0

    def _js_tick(self):
        m = JointState(); m.header.stamp = self.get_clock().now().to_msg()
        m.name = list(LEFT); m.position = list(self.q); m.velocity = [0.0] * 7
        self.js.publish(m)

    def _cloud_tick(self):
        self.cloud.publish(make_cloud(self.get_clock().now().to_msg()))

    def _on_cmd(self, msg: JointTrajectory):
        if not msg.points:
            return
        idx = {n: i for i, n in enumerate(msg.joint_names)}
        for j, n in enumerate(LEFT):
            if n in idx:
                self.q[j] = msg.points[0].positions[idx[n]]

    def _on_ee(self, p):
        self.ee = np.array([p.pose.position.x, p.pose.position.y, p.pose.position.z])

    def _on_grasp(self, p):
        # grasp_pose is the GRASP point; auto_send commands PRE-grasp = +PRE_H in z.
        self.grasp = np.array([p.pose.position.x, p.pose.position.y,
                               p.pose.position.z + PRE_H])

    def _report(self):
        self._t += 1
        if self.ee is None or self.grasp is None:
            self.get_logger().info(f"[{self._t}s] waiting (ee={self.ee is not None}, "
                                   f"grasp={self.grasp is not None})")
            return
        err = float(np.linalg.norm(self.ee - self.grasp))
        exy = float(np.linalg.norm(self.ee[:2] - self.grasp[:2]))
        self.get_logger().info(
            f"[{self._t}s] EE=({self.ee[0]:+.3f},{self.ee[1]:+.3f},{self.ee[2]:+.3f}) "
            f"target_pregrasp=({self.grasp[0]:+.3f},{self.grasp[1]:+.3f},{self.grasp[2]:+.3f}) "
            f"|err|={err:.3f} xy={exy:.3f}")
        if exy < 0.03:
            self.get_logger().info(f">>> E2E verdict: PASS (EE reached pre-grasp XY, err={err:.3f}m)")


def main():
    rclpy.init()
    n = E2E()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
