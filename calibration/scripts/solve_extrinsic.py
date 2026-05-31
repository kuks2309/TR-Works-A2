#!/usr/bin/env python3
"""Compute T_base_camera from ChArUco board observation + known board pose.

Workflow:
  1. Place ChArUco board flat on the table at a known location.
  2. Measure the board's bottom-left chessboard corner position in base_link
     frame (x, y, z) and the board's yaw rotation around base_link Z.
  3. Run this script — it averages N frames of solvePnP for T_camera_board,
     combines with the known T_base_board, and outputs T_base_camera +
     a ready-to-run static_transform_publisher command.

Board frame convention (OpenCV ChArUco):
  - Origin: bottom-left corner of the chessboard pattern.
  - X-axis: along the bottom edge (toward the right of the pattern).
  - Y-axis: along the left edge (toward the top of the pattern).
  - Z-axis: out of the board surface (away from the printed side).

Default placement assumed (override with --pitch / --roll if different):
  Board lies flat on the table, pattern facing UP. Board Z axis = base_link +Z.
  Board yaw=0 means board X axis aligns with base_link +X (i.e. board's
  short edge points toward the robot front).

CLI:
  python3 solve_extrinsic.py --bx 0.40 --by 0.00 --bz 0.70 --yaw 0 \
      --samples 20 --parent base_link --child d435_center_link
"""
import argparse
import math
import sys
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

DICTIONARY = cv2.aruco.DICT_5X5_50
SQUARES_X, SQUARES_Y = 5, 7
SQUARE_LEN_M, MARKER_LEN_M = 0.040, 0.030

IMG_TOPIC = "/d435_center/d435_center/color/image_raw"
INFO_TOPIC = "/d435_center/d435_center/color/camera_info"


def rpy_to_R(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def R_to_rpy(R):
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0
    return roll, pitch, yaw


def R_to_quat(R):
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
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


class Capturer(Node):
    def __init__(self):
        super().__init__("extrinsic_solver")
        self.bridge = CvBridge()
        self.K = None
        self.D = None
        self.cam_frame = None
        self.frames = []

        dictionary = cv2.aruco.getPredefinedDictionary(DICTIONARY)
        self.board = cv2.aruco.CharucoBoard(
            (SQUARES_X, SQUARES_Y), SQUARE_LEN_M, MARKER_LEN_M, dictionary)
        self.detector = cv2.aruco.CharucoDetector(self.board)
        self.obj_all = self.board.getChessboardCorners()

        qos = QoSProfile(depth=2)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.create_subscription(Image, IMG_TOPIC, self._on_img, qos)
        self.create_subscription(CameraInfo, INFO_TOPIC, self._on_info, qos)

    def _on_info(self, msg):
        if self.K is None:
            self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.D = np.array(msg.d, dtype=np.float64)
            self.cam_frame = msg.header.frame_id

    def _on_img(self, msg):
        if self.K is None:
            return
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ch_corners, ch_ids, _, _ = self.detector.detectBoard(gray)
        if ch_ids is None or len(ch_ids) < 6:
            return
        obj_pts = self.obj_all[ch_ids.flatten()].astype(np.float32)
        img_pts = ch_corners.reshape(-1, 2).astype(np.float32)
        ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, self.K, self.D,
                                      flags=cv2.SOLVEPNP_ITERATIVE)
        if ok:
            R, _ = cv2.Rodrigues(rvec)
            self.frames.append((R, tvec.flatten(), len(ch_ids)))


def average_pose(frames):
    Ts = np.array([f[1] for f in frames])
    t_mean = Ts.mean(axis=0)
    t_std = Ts.std(axis=0)
    # average rotation via quaternion: take first as reference, weight by inliers
    qs = []
    for R, _, n in frames:
        q = np.array(R_to_quat(R))
        qs.append((q, n))
    q_ref = qs[0][0]
    acc = np.zeros(4)
    wsum = 0.0
    for q, w in qs:
        if np.dot(q, q_ref) < 0:
            q = -q
        acc += w * q
        wsum += w
    q_mean = acc / wsum
    q_mean /= np.linalg.norm(q_mean)
    # back to R
    x, y, z, w = q_mean
    R_mean = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    return R_mean, t_mean, t_std


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bx", type=float, required=True,
                    help="board origin x in base_link (m)")
    ap.add_argument("--by", type=float, required=True,
                    help="board origin y in base_link (m)")
    ap.add_argument("--bz", type=float, required=True,
                    help="board origin z in base_link (m) — usually table top height")
    ap.add_argument("--yaw", type=float, default=0.0,
                    help="board yaw around base_link Z (deg, default 0)")
    ap.add_argument("--pitch", type=float, default=0.0,
                    help="board pitch (deg, default 0 = board flat)")
    ap.add_argument("--roll", type=float, default=0.0,
                    help="board roll (deg, default 0)")
    ap.add_argument("--samples", type=int, default=20,
                    help="frames to average (default 20)")
    ap.add_argument("--parent", default="base_link")
    ap.add_argument("--child", default="d435_center_link",
                    help="usually d435_center_link (URDF camera link, NOT optical)")
    ap.add_argument("--cam-frame-is-optical", action="store_true",
                    help="set if you want T_base→optical frame directly (rare)")
    args = ap.parse_args()

    rclpy.init()
    n = Capturer()
    print(f"Collecting {args.samples} frames of PnP …")
    t0 = time.time()
    while len(n.frames) < args.samples and time.time() - t0 < 30.0:
        rclpy.spin_once(n, timeout_sec=0.1)
        if len(n.frames) > 0 and len(n.frames) % 5 == 0:
            print(f"  {len(n.frames)}/{args.samples}")

    if len(n.frames) < 5:
        print(f"FAIL: only {len(n.frames)} valid frames collected")
        n.destroy_node(); rclpy.shutdown(); sys.exit(1)

    cam_frame = n.cam_frame
    n.destroy_node(); rclpy.shutdown()

    R_cam_board, t_cam_board, t_std = average_pose(n.frames)
    print(f"\nT_camera_board ({cam_frame} → board), averaged over {len(n.frames)} frames:")
    print(f"  translation: ({t_cam_board[0]:+.4f}, {t_cam_board[1]:+.4f}, "
          f"{t_cam_board[2]:+.4f}) m   ± ({t_std[0]*1000:.1f}, "
          f"{t_std[1]*1000:.1f}, {t_std[2]*1000:.1f}) mm")
    rpy_cb = R_to_rpy(R_cam_board)
    print(f"  rpy:         ({math.degrees(rpy_cb[0]):+.2f}, "
          f"{math.degrees(rpy_cb[1]):+.2f}, {math.degrees(rpy_cb[2]):+.2f}) deg")

    # T_base_board from user input
    R_base_board = rpy_to_R(math.radians(args.roll), math.radians(args.pitch),
                            math.radians(args.yaw))
    t_base_board = np.array([args.bx, args.by, args.bz])

    # T_base_camera = T_base_board * (T_camera_board)^-1
    R_board_cam = R_cam_board.T
    t_board_cam = -R_cam_board.T @ t_cam_board

    R_base_cam = R_base_board @ R_board_cam
    t_base_cam = R_base_board @ t_board_cam + t_base_board

    rpy = R_to_rpy(R_base_cam)
    qx, qy, qz, qw = R_to_quat(R_base_cam)

    print(f"\n=== RESULT: T_base_camera ({args.parent} → {cam_frame}) ===")
    print(f"  translation (m): x={t_base_cam[0]:+.4f}  y={t_base_cam[1]:+.4f}  "
          f"z={t_base_cam[2]:+.4f}")
    print(f"  rotation (rpy deg): r={math.degrees(rpy[0]):+.2f}  "
          f"p={math.degrees(rpy[1]):+.2f}  y={math.degrees(rpy[2]):+.2f}")
    print(f"  quaternion (xyzw): {qx:+.6f} {qy:+.6f} {qz:+.6f} {qw:+.6f}")

    print(f"\nWARNING: above is base_link → {cam_frame} (the OPTICAL frame).")
    print(f"To publish base_link → {args.child} (URDF camera_link), you need the")
    print(f"static optical→link offset from realsense2_description.")
    print(f"For now publishing the OPTICAL frame transform directly:")

    print("\n=== STATIC TRANSFORM PUBLISHER COMMAND ===")
    print(
        f"ros2 run tf2_ros static_transform_publisher \\\n"
        f"    --x {t_base_cam[0]:.4f} --y {t_base_cam[1]:.4f} --z {t_base_cam[2]:.4f} \\\n"
        f"    --qx {qx:.6f} --qy {qy:.6f} --qz {qz:.6f} --qw {qw:.6f} \\\n"
        f"    --frame-id {args.parent} --child-frame-id {cam_frame}"
    )


if __name__ == "__main__":
    main()
