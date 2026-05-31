#!/usr/bin/env python3
"""Live ChArUco detection node.

Subscribes:
  /d435_center/d435_center/color/image_raw
  /d435_center/d435_center/color/camera_info  (intrinsics for pose)

Publishes:
  /charuco/image_debug            sensor_msgs/Image  (annotated, RGB)
  /tf                              charuco_board frame (parent: camera_optical)
  /charuco/board_marker            visualization_msgs/Marker  (board plane CUBE)

Run:
    python3 live_detect.py
View in RViz:
    Image display → /charuco/image_debug
    Marker → /charuco/board_marker
    TF → enable charuco_board frame
"""
import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker

DICTIONARY = cv2.aruco.DICT_5X5_50
SQUARES_X, SQUARES_Y = 5, 7
SQUARE_LEN_M, MARKER_LEN_M = 0.040, 0.030
BOARD_W_M = SQUARES_X * SQUARE_LEN_M     # 0.200
BOARD_H_M = SQUARES_Y * SQUARE_LEN_M     # 0.280

IMG_TOPIC = "/d435_center/d435_center/color/image_raw"
INFO_TOPIC = "/d435_center/d435_center/color/camera_info"


def rvec_tvec_to_quat(rvec):
    R, _ = cv2.Rodrigues(rvec)
    # rotation matrix → quaternion (x,y,z,w)
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


class LiveDetector(Node):
    def __init__(self):
        super().__init__("charuco_live_detect")
        self.bridge = CvBridge()
        self.K = None
        self.D = None
        self.cam_frame = None

        dictionary = cv2.aruco.getPredefinedDictionary(DICTIONARY)
        self.board = cv2.aruco.CharucoBoard(
            (SQUARES_X, SQUARES_Y), SQUARE_LEN_M, MARKER_LEN_M, dictionary)
        self.detector = cv2.aruco.CharucoDetector(self.board)
        self.obj_points_all = self.board.getChessboardCorners()  # (N,3) m

        qos = QoSProfile(depth=2)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.create_subscription(Image, IMG_TOPIC, self._on_img, qos)
        self.create_subscription(CameraInfo, INFO_TOPIC, self._on_info, qos)

        self.pub_img = self.create_publisher(Image, "/charuco/image_debug", 5)
        self.pub_marker = self.create_publisher(Marker, "/charuco/board_marker", 5)
        self.tf_br = TransformBroadcaster(self)
        self.last_log = 0.0

    def _on_info(self, msg: CameraInfo):
        if self.K is None:
            self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.D = np.array(msg.d, dtype=np.float64)
            self.cam_frame = msg.header.frame_id
            self.get_logger().info(
                f"intrinsics ready: fx={self.K[0,0]:.1f} cam_frame={self.cam_frame}")

    def _on_img(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ch_corners, ch_ids, m_corners, m_ids = self.detector.detectBoard(gray)

        n_m = 0 if m_ids is None else len(m_ids)
        n_c = 0 if ch_ids is None else len(ch_ids)

        vis = img.copy()
        if n_m > 0:
            cv2.aruco.drawDetectedMarkers(vis, m_corners, m_ids)
        if n_c > 0:
            cv2.aruco.drawDetectedCornersCharuco(vis, ch_corners, ch_ids,
                                                 (0, 0, 255))

        pose_text = ""
        if n_c >= 6 and self.K is not None:
            obj_pts = self.obj_points_all[ch_ids.flatten()].astype(np.float32)
            img_pts = ch_corners.reshape(-1, 2).astype(np.float32)
            ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, self.K, self.D,
                                          flags=cv2.SOLVEPNP_ITERATIVE)
            if ok:
                pose_text = (f"d={np.linalg.norm(tvec)*1000:.0f}mm "
                             f"t=({tvec[0,0]*100:+.1f},{tvec[1,0]*100:+.1f},"
                             f"{tvec[2,0]*100:+.1f})cm")
                # draw axis
                cv2.drawFrameAxes(vis, self.K, self.D, rvec, tvec, 0.05, 3)
                self._publish_pose(rvec, tvec, msg.header.stamp)

        cv2.putText(vis, f"ArUco {n_m}/17  Corners {n_c}/24  {pose_text}",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (0, 255, 0) if n_c >= 12 else (0, 165, 255), 2, cv2.LINE_AA)

        out = self.bridge.cv2_to_imgmsg(vis, "bgr8")
        out.header = msg.header
        self.pub_img.publish(out)

        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_log > 1.0:
            self.last_log = now
            self.get_logger().info(
                f"ArUco {n_m}/17  Corners {n_c}/24  {pose_text}")

    def _publish_pose(self, rvec, tvec, stamp):
        x, y, z, w = rvec_tvec_to_quat(rvec)
        # TF
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.cam_frame or "d435_center_color_optical_frame"
        t.child_frame_id = "charuco_board"
        t.transform.translation.x = float(tvec[0, 0])
        t.transform.translation.y = float(tvec[1, 0])
        t.transform.translation.z = float(tvec[2, 0])
        t.transform.rotation.x = x
        t.transform.rotation.y = y
        t.transform.rotation.z = z
        t.transform.rotation.w = w
        self.tf_br.sendTransform(t)

        # Marker (board plane CUBE in board frame, but we publish in cam frame)
        m = Marker()
        m.header = t.header
        m.ns = "charuco"
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD
        # board CUBE center: board frame origin is at (0,0,0) corner;
        # center is at (W/2, H/2, 0)
        # we transform that to cam frame using R, tvec
        R, _ = cv2.Rodrigues(rvec)
        center_board = np.array([BOARD_W_M / 2, BOARD_H_M / 2, 0.0])
        center_cam = R @ center_board + tvec.flatten()
        m.pose.position.x = float(center_cam[0])
        m.pose.position.y = float(center_cam[1])
        m.pose.position.z = float(center_cam[2])
        m.pose.orientation.x = x
        m.pose.orientation.y = y
        m.pose.orientation.z = z
        m.pose.orientation.w = w
        m.scale.x = BOARD_W_M
        m.scale.y = BOARD_H_M
        m.scale.z = 0.002
        m.color.r = 0.2; m.color.g = 0.6; m.color.b = 1.0; m.color.a = 0.5
        m.lifetime.sec = 1
        self.pub_marker.publish(m)


def main():
    rclpy.init()
    n = LiveDetector()
    try:
        rclpy.spin(n)
    finally:
        n.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
