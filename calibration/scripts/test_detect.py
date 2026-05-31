#!/usr/bin/env python3
"""Quick ChArUco detection test on D435 RGB stream.

Subscribes one frame from /d435_center/d435_center/color/image_raw,
runs ArUco + ChArUco corner interpolation, and reports counts +
saves a debug visualization.

Run while the RealSense node is up:
    python3 test_detect.py
"""
from pathlib import Path
import sys, time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

DICTIONARY = cv2.aruco.DICT_5X5_50
SQUARES_X, SQUARES_Y = 5, 7
SQUARE_LEN_M, MARKER_LEN_M = 0.040, 0.030
TOPIC = "/d435_center/d435_center/color/image_raw"
OUT = Path(__file__).resolve().parent.parent / "boards" / "detect_test.png"


class Probe(Node):
    def __init__(self):
        super().__init__("charuco_probe")
        self.bridge = CvBridge()
        self.frame = None
        self.create_subscription(Image, TOPIC, self._on_img, 5)

    def _on_img(self, msg):
        if self.frame is None:
            self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")


def main():
    rclpy.init()
    n = Probe()
    t0 = time.time()
    while n.frame is None and time.time() - t0 < 8.0:
        rclpy.spin_once(n, timeout_sec=0.2)
    if n.frame is None:
        print(f"FAIL: no image on {TOPIC} within 8s")
        rclpy.shutdown(); sys.exit(1)

    img = n.frame
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    dictionary = cv2.aruco.getPredefinedDictionary(DICTIONARY)
    board = cv2.aruco.CharucoBoard(
        (SQUARES_X, SQUARES_Y), SQUARE_LEN_M, MARKER_LEN_M, dictionary)
    ch_detector = cv2.aruco.CharucoDetector(board)
    ch_corners, ch_ids, m_corners, m_ids = ch_detector.detectBoard(gray)

    n_markers = 0 if m_ids is None else len(m_ids)
    n_corners = 0 if ch_ids is None else len(ch_ids)
    vis = img.copy()
    if n_markers > 0:
        cv2.aruco.drawDetectedMarkers(vis, m_corners, m_ids)
    if n_corners > 0:
        cv2.aruco.drawDetectedCornersCharuco(vis, ch_corners, ch_ids,
                                             (0, 0, 255))

    cv2.imwrite(str(OUT), vis)
    h, w = img.shape[:2]
    print(f"Image: {w}x{h}")
    print(f"ArUco markers detected: {n_markers} / {SQUARES_X*SQUARES_Y // 2 + 1} expected")
    print(f"ChArUco corners detected: {n_corners} / {(SQUARES_X-1)*(SQUARES_Y-1)} expected")
    print(f"Visualization saved: {OUT}")
    if n_markers == 0:
        print("\n결과: 검출 실패 — 보드가 카메라 시야에 있는지, 인쇄 100% 인지 확인")
    elif n_corners < (SQUARES_X-1)*(SQUARES_Y-1) * 0.5:
        print("\n결과: 일부만 검출 — 보드 일부 가림 또는 각도 문제")
    else:
        print("\n결과: 정상 검출 ✓ 캘리브레이션 진행 가능")

    n.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
