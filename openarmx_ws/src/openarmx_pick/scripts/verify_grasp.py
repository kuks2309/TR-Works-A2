#!/usr/bin/env python3
"""Synthetic Stage-B check for grasp_pose_node (no camera/robot needed).

Publishes a fake box-top inlier cloud on ``/box_plane/cloud`` directly in the
base frame ``openarmx_body_link0`` (so the node's TF lookup is identity) and
prints the grasp PoseStamped the node emits on ``/openarmx/grasp_pose``.

Synthetic box top: a flat 0.16 m (long, +Y) x 0.08 m (short, +X) patch at
z = 0.20, centred at (0.50, 0.10). Expected grasp:
  * position ~ (0.50, 0.10, 0.195)   (centroid minus grasp_depth)
  * approach pointing straight down  (tool +z -> -z_base)
  * gripper opening spanning the SHORT axis (+X)
"""
import math
import struct

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped

BASE = "openarmx_body_link0"
CX, CY, CZ = 0.50, 0.10, 0.20
LONG_Y, SHORT_X = 0.16, 0.08


def make_cloud(stamp) -> PointCloud2:
    xs = np.arange(-SHORT_X / 2, SHORT_X / 2, 0.004)
    ys = np.arange(-LONG_Y / 2, LONG_Y / 2, 0.004)
    pts = [(CX + x, CY + y, CZ) for x in xs for y in ys]
    data = b"".join(struct.pack("<fff", *p) for p in pts)
    msg = PointCloud2()
    msg.header = Header(frame_id=BASE)
    msg.header.stamp = stamp
    msg.height = 1
    msg.width = len(pts)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(pts)
    msg.data = data
    msg.is_dense = True
    return msg


class GraspChecker(Node):
    def __init__(self):
        super().__init__("grasp_checker")
        self.pub = self.create_publisher(PointCloud2, "/box_plane/cloud", 5)
        self.create_subscription(PoseStamped, "/openarmx/grasp_pose", self._on_pose, 5)
        self.create_timer(0.2, self._tick)
        self._n = 0
        self._got = False

    def _tick(self):
        self.pub.publish(make_cloud(self.get_clock().now().to_msg()))
        self._n += 1

    def _on_pose(self, p: PoseStamped):
        if self._got:
            return
        self._got = True
        q = p.pose.orientation
        # Tool +z axis in base frame = 3rd column of R(q) -> the approach direction.
        x, y, z, w = q.x, q.y, q.z, q.w
        approach = np.array([2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)])
        opening = np.array([1 - 2 * (y * y + z * z), 2 * (x * y + z * w), 2 * (x * z - y * w)])
        self.get_logger().info("=" * 56)
        self.get_logger().info(f"grasp frame   : {p.header.frame_id}")
        self.get_logger().info(
            f"grasp position: ({p.pose.position.x:+.3f}, {p.pose.position.y:+.3f}, "
            f"{p.pose.position.z:+.3f})  [expect ~(+0.500, +0.100, +0.195)]")
        self.get_logger().info(
            f"approach axis : ({approach[0]:+.2f}, {approach[1]:+.2f}, {approach[2]:+.2f})  "
            f"[expect ~(0, 0, -1) straight down]")
        self.get_logger().info(
            f"opening axis  : ({opening[0]:+.2f}, {opening[1]:+.2f}, {opening[2]:+.2f})  "
            f"[expect ~(+/-1, 0, 0) short axis]")
        ok_pos = (abs(p.pose.position.x - CX) < 0.02 and abs(p.pose.position.y - CY) < 0.02)
        ok_down = approach[2] < -0.9
        ok_open = abs(opening[0]) > 0.9
        verdict = "PASS" if (ok_pos and ok_down and ok_open) else "CHECK"
        self.get_logger().info(f">>> Stage-B synthetic verdict: {verdict} "
                               f"(pos={ok_pos}, down={ok_down}, open={ok_open})")
        self.get_logger().info("=" * 56)


def main():
    rclpy.init()
    node = GraspChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
