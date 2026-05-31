#!/usr/bin/env python3
"""ROS node that publishes ground / box-top plane markers (computed by
analyze_planes.analyze_bag) on /plane_markers, latched.

Run alongside `ros2 bag play <bag>` to see the planes overlaid on the
PointCloud2 in RViz.
"""
import argparse
import math
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2, PointField
import struct

from box_detection.analyze_planes import analyze_bag


def make_colored_pointcloud2(points_xyz, rgb, frame_id, stamp):
    """Pack (N,3) xyz + 24-bit rgb int into PointCloud2 (rviz xyzrgb format)."""
    n = points_xyz.shape[0]
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.height = 1
    msg.width = n
    msg.is_dense = False
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = msg.point_step * n
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    rgb_packed = (int(rgb[0]) << 16) | (int(rgb[1]) << 8) | int(rgb[2])
    rgb_float = struct.unpack("f", struct.pack("I", rgb_packed))[0]
    arr = np.zeros((n, 4), dtype=np.float32)
    arr[:, 0:3] = points_xyz.astype(np.float32)
    arr[:, 3] = rgb_float
    msg.data = arr.tobytes()
    return msg


def quat_from_two_vectors(a, b):
    """Quaternion that rotates unit vector a onto unit vector b."""
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    c = float(np.dot(a, b))
    if c > 0.999999:
        return (0.0, 0.0, 0.0, 1.0)
    if c < -0.999999:
        # 180-deg around any axis perp to a
        axis = np.cross(a, [1.0, 0.0, 0.0])
        if np.linalg.norm(axis) < 1e-6:
            axis = np.cross(a, [0.0, 1.0, 0.0])
        axis /= np.linalg.norm(axis)
        return (axis[0], axis[1], axis[2], 0.0)
    v = np.cross(a, b)
    s = math.sqrt((1.0 + c) * 2.0)
    inv = 1.0 / s
    return (v[0] * inv, v[1] * inv, v[2] * inv, s * 0.5)


def make_plane_marker(coef, frame_id, ns, marker_id, color, size=0.8,
                      thickness=0.003):
    """Thin oriented box centered on the plane, sized `size` x `size`.
    Place center at the closest point on the plane to the camera origin."""
    a, b, c, d = coef
    n = np.array([a, b, c], dtype=float)
    n_norm = np.linalg.norm(n)
    n /= n_norm
    # closest point on plane to origin: -d*n (after normalizing equation
    # so that n is unit; plane becomes n.x + d/|n_orig| = 0)
    d_eff = d / n_norm
    center = -d_eff * n  # point on plane closest to origin

    qx, qy, qz, qw = quat_from_two_vectors(np.array([0.0, 0.0, 1.0]), n)

    m = Marker()
    m.header.frame_id = frame_id
    m.ns = ns
    m.id = marker_id
    m.type = Marker.CUBE
    m.action = Marker.ADD
    m.pose.position.x = float(center[0])
    m.pose.position.y = float(center[1])
    m.pose.position.z = float(center[2])
    m.pose.orientation.x = qx
    m.pose.orientation.y = qy
    m.pose.orientation.z = qz
    m.pose.orientation.w = qw
    m.scale.x = size
    m.scale.y = size
    m.scale.z = thickness
    m.color.r = float(color[0])
    m.color.g = float(color[1])
    m.color.b = float(color[2])
    m.color.a = 0.45
    return m


def make_normal_arrow(coef, frame_id, ns, marker_id, color, length=0.25):
    a, b, c, d = coef
    n = np.array([a, b, c], dtype=float)
    n /= np.linalg.norm(n)
    d_eff = d / np.linalg.norm([a, b, c])
    base = -d_eff * n
    tip = base + n * length
    m = Marker()
    m.header.frame_id = frame_id
    m.ns = ns + "_normal"
    m.id = marker_id
    m.type = Marker.ARROW
    m.action = Marker.ADD
    m.scale.x = 0.01
    m.scale.y = 0.02
    m.scale.z = 0.0
    m.color.r = float(color[0])
    m.color.g = float(color[1])
    m.color.b = float(color[2])
    m.color.a = 0.9
    p0 = Point(); p0.x, p0.y, p0.z = float(base[0]), float(base[1]), float(base[2])
    p1 = Point(); p1.x, p1.y, p1.z = float(tip[0]), float(tip[1]), float(tip[2])
    m.points = [p0, p1]
    return m


class PlanePublisher(Node):
    def __init__(self, results):
        super().__init__("plane_visualizer")
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.pub_marker = self.create_publisher(MarkerArray, "/plane_markers", qos)
        self.pub_ground = self.create_publisher(
            PointCloud2, "/plane_inliers/ground", qos)
        self.pub_box = self.create_publisher(
            PointCloud2, "/plane_inliers/box_top", qos)
        self.timer = self.create_timer(1.0, self._tick)
        self.array = MarkerArray()
        self.ground_clouds = []  # list[(frame, np.ndarray)]
        self.box_clouds = []

        idx = 0
        for cam, info in results.items():
            if info is None:
                continue
            frame = info["frame"]
            g = info["ground"]
            b = info["box_top"]
            self.array.markers.append(make_plane_marker(
                g["coef"], frame, f"ground_{cam}", idx, (0.2, 0.9, 0.2)))
            idx += 1
            self.array.markers.append(make_normal_arrow(
                g["coef"], frame, f"ground_{cam}", idx, (0.2, 0.9, 0.2)))
            idx += 1
            self.ground_clouds.append((frame, g["inlier_pts"]))
            if b is not None:
                self.array.markers.append(make_plane_marker(
                    b["coef"], frame, f"boxtop_{cam}", idx, (0.95, 0.2, 0.2),
                    size=0.5))
                idx += 1
                self.array.markers.append(make_normal_arrow(
                    b["coef"], frame, f"boxtop_{cam}", idx, (0.95, 0.2, 0.2)))
                idx += 1
                self.box_clouds.append((frame, b["inlier_pts"]))
        n_g = sum(c.shape[0] for _, c in self.ground_clouds)
        n_b = sum(c.shape[0] for _, c in self.box_clouds)
        self.get_logger().info(
            f"markers={len(self.array.markers)}  ground_inliers={n_g}  "
            f"box_top_inliers={n_b}")

    def _tick(self):
        now = self.get_clock().now().to_msg()
        for m in self.array.markers:
            m.header.stamp = now
        self.pub_marker.publish(self.array)
        # Publish first cam's inliers (typically center) — Fixed Frame match.
        if self.ground_clouds:
            frame, pts = self.ground_clouds[0]
            self.pub_ground.publish(make_colored_pointcloud2(
                pts, (60, 230, 60), frame, now))
        if self.box_clouds:
            frame, pts = self.box_clouds[0]
            self.pub_box.publish(make_colored_pointcloud2(
                pts, (240, 60, 60), frame, now))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True,
                    help="bag dir name under bags/20260506/")
    ap.add_argument("--root", default="bags/20260506")
    args, _ = ap.parse_known_args()

    bag_dir = Path(args.root) / args.bag
    print(f"Analyzing {bag_dir} ...")
    results = analyze_bag(bag_dir)
    if not results:
        print("No results.")
        return

    rclpy.init()
    node = PlanePublisher(results)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
