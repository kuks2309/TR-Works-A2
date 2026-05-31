#!/usr/bin/env python3
"""Realtime plane detector that subscribes to two D435 PointCloud2 topics,
runs ground / box-top RANSAC every refresh interval, and republishes
plane markers + colored inlier clouds for RViz overlay.

Assumes camera TFs (d435_center_*, d435_center_upper_*) are published by
an external robot_state_publisher (URDF in another workspace).
"""
import math
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy,
                       QoSHistoryPolicy)
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from visualization_msgs.msg import MarkerArray

from box_detection.analyze_planes import analyze_cloud
from box_detection.visualize_planes import (make_plane_marker,
                                             make_normal_arrow,
                                             make_colored_pointcloud2)


CAMERAS = [
    ("center", "/d435_center/d435_center/depth/color/points"),
    ("upper",  "/d435_center_upper/d435_center_upper/depth/color/points"),
]


class LivePlaneDetector(Node):
    def __init__(self):
        super().__init__("live_plane_detector")
        self.declare_parameter("refresh_rate", 1.0)   # Hz
        self.declare_parameter("frame_buffer", 5)
        self.declare_parameter("min_points", 20000)
        refresh = self.get_parameter("refresh_rate").value
        self.buffer_size = int(self.get_parameter("frame_buffer").value)
        self.min_points = int(self.get_parameter("min_points").value)

        sub_qos = QoSProfile(depth=5)
        sub_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        sub_qos.durability = QoSDurabilityPolicy.VOLATILE
        sub_qos.history = QoSHistoryPolicy.KEEP_LAST

        pub_qos = QoSProfile(depth=1)
        pub_qos.reliability = QoSReliabilityPolicy.RELIABLE
        pub_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.buffers = {label: deque(maxlen=self.buffer_size)
                        for label, _ in CAMERAS}
        self.frame_id = {label: None for label, _ in CAMERAS}

        for label, topic in CAMERAS:
            self.create_subscription(
                PointCloud2, topic,
                lambda msg, lab=label: self._on_cloud(lab, msg),
                qos_profile=sub_qos)

        self.pub_marker = self.create_publisher(
            MarkerArray, "/plane_markers", pub_qos)
        self.pub_ground = self.create_publisher(
            PointCloud2, "/plane_inliers/ground", pub_qos)
        self.pub_box = self.create_publisher(
            PointCloud2, "/plane_inliers/box_top", pub_qos)

        self.create_timer(1.0 / float(refresh), self._tick)
        self.get_logger().info(
            f"Subscribed to {len(CAMERAS)} cameras, refresh={refresh:.1f} Hz, "
            f"buffer={self.buffer_size} frames")

    def _on_cloud(self, label, msg: PointCloud2):
        self.frame_id[label] = msg.header.frame_id.lstrip("/")
        s = pc2.read_points(msg, field_names=["x", "y", "z"], skip_nans=True)
        pts = np.stack([np.asarray(s["x"], dtype=np.float64),
                        np.asarray(s["y"], dtype=np.float64),
                        np.asarray(s["z"], dtype=np.float64)], axis=1)
        self.buffers[label].append(pts)

    def _tick(self):
        markers = MarkerArray()
        idx = 0
        ground_pts = None
        ground_frame = None
        box_pts = None
        box_frame = None

        for label, _ in CAMERAS:
            buf = self.buffers[label]
            if not buf:
                continue
            pts = np.vstack(list(buf))
            if pts.shape[0] < self.min_points:
                continue
            try:
                info = analyze_cloud(label, self.frame_id[label], pts)
            except Exception as e:
                self.get_logger().warn(f"[{label}] analyze failed: {e}")
                continue
            if info is None:
                continue
            frame = info["frame"]
            g = info["ground"]
            b = info["box_top"]
            markers.markers.append(make_plane_marker(
                g["coef"], frame, f"ground_{label}", idx, (0.2, 0.9, 0.2)))
            idx += 1
            markers.markers.append(make_normal_arrow(
                g["coef"], frame, f"ground_{label}", idx, (0.2, 0.9, 0.2)))
            idx += 1
            if ground_pts is None:
                ground_pts = g["inlier_pts"]
                ground_frame = frame
            if b is not None:
                markers.markers.append(make_plane_marker(
                    b["coef"], frame, f"boxtop_{label}", idx,
                    (0.95, 0.2, 0.2), size=0.5))
                idx += 1
                markers.markers.append(make_normal_arrow(
                    b["coef"], frame, f"boxtop_{label}", idx,
                    (0.95, 0.2, 0.2)))
                idx += 1
                if box_pts is None:
                    box_pts = b["inlier_pts"]
                    box_frame = frame

        if not markers.markers:
            return
        now = self.get_clock().now().to_msg()
        for m in markers.markers:
            m.header.stamp = now
        self.pub_marker.publish(markers)
        if ground_pts is not None:
            self.pub_ground.publish(make_colored_pointcloud2(
                ground_pts, (60, 230, 60), ground_frame, now))
        if box_pts is not None:
            self.pub_box.publish(make_colored_pointcloud2(
                box_pts, (240, 60, 60), box_frame, now))


def main():
    rclpy.init()
    node = LivePlaneDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
