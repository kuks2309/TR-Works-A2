#!/usr/bin/env python3
"""Square-pattern extrinsic calibration.

User moves the ChArUco board in a 4-corner square (parallel sides aligned
with base_link X/Y axes via a desk-edge guide). Captures one averaged pose
per corner, then solves T_base_camera by averaging 4 estimates.

Assumed board placement:
  - Board lies flat on desk (z=0 in base_link).
  - All 4 corners have IDENTICAL board orientation (pure translation between them).
  - Board's own X-axis is parallel to base_link +X (user's responsibility to
    align via desk guide). Override with --board-yaw if otherwise.

Corner traversal order (when looking down):
    pos4 ●─────● pos3       +Y
         │     │             ↑
         │     │             │
    pos1 ●─────● pos2        └──→ +X
    pos1 = (bx, by, 0) in base_link
    pos2 = pos1 + (side, 0, 0)
    pos3 = pos1 + (side, side, 0)
    pos4 = pos1 + (0, side, 0)

CLI:
  python3 solve_extrinsic_square.py --side 0.10 --bx 0.55 --by -0.10 \\
      --frames-per-corner 30 --parent base_link
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
    if sy < 1e-6:
        return math.atan2(-R[1, 2], R[1, 1]), math.atan2(-R[2, 0], sy), 0.0
    return (math.atan2(R[2, 1], R[2, 2]),
            math.atan2(-R[2, 0], sy),
            math.atan2(R[1, 0], R[0, 0]))


def R_to_quat(R):
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2.0
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        return (0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s,
                (R[2, 1] - R[1, 2]) / s)
    if R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        return ((R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s,
                (R[0, 2] - R[2, 0]) / s)
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
    return ((R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s,
            (R[1, 0] - R[0, 1]) / s)


def avg_quat(quats, weights=None):
    quats = np.asarray(quats)
    if weights is None:
        weights = np.ones(len(quats))
    ref = quats[0]
    acc = np.zeros(4)
    for q, w in zip(quats, weights):
        if np.dot(q, ref) < 0:
            q = -q
        acc += w * q
    acc /= np.linalg.norm(acc)
    return acc


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Capturer(Node):
    def __init__(self):
        super().__init__("square_extrinsic")
        self.bridge = CvBridge()
        self.K = None
        self.D = None
        self.cam_frame = None
        self.collecting = False
        self.collected = []
        self.target = 0

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
        if not self.collecting or self.K is None:
            return
        if len(self.collected) >= self.target:
            return
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ch_corners, ch_ids, _, _ = self.detector.detectBoard(gray)
        if ch_ids is None or len(ch_ids) < 8:
            return
        obj_pts = self.obj_all[ch_ids.flatten()].astype(np.float32)
        img_pts = ch_corners.reshape(-1, 2).astype(np.float32)
        ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, self.K, self.D,
                                      flags=cv2.SOLVEPNP_ITERATIVE)
        if ok:
            R, _ = cv2.Rodrigues(rvec)
            self.collected.append((R, tvec.flatten(), len(ch_ids)))

    def collect(self, n_frames, label):
        self.target = n_frames
        self.collected = []
        self.collecting = True
        print(f"  collecting {n_frames} frames for {label} …", end="", flush=True)
        t0 = time.time()
        while len(self.collected) < n_frames and time.time() - t0 < 20.0:
            rclpy.spin_once(self, timeout_sec=0.05)
            if len(self.collected) % 10 == 0 and len(self.collected) > 0:
                print(".", end="", flush=True)
        self.collecting = False
        print(f" got {len(self.collected)}")
        if len(self.collected) < 5:
            raise RuntimeError(f"only {len(self.collected)} frames for {label}")
        # average
        ts = np.array([f[1] for f in self.collected])
        t_mean = ts.mean(axis=0)
        t_std = ts.std(axis=0)
        qs = [R_to_quat(R) for R, _, _ in self.collected]
        ws = [n for _, _, n in self.collected]
        q_mean = avg_quat(qs, ws)
        R_mean = quat_to_R(q_mean)
        return R_mean, t_mean, t_std


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--side", type=float, required=True,
                    help="square side length (m), e.g. 0.10 for 10cm")
    ap.add_argument("--bx", type=float, required=True,
                    help="pos1 board origin x in base_link (m)")
    ap.add_argument("--by", type=float, required=True,
                    help="pos1 board origin y in base_link (m)")
    ap.add_argument("--bz", type=float, default=0.0,
                    help="desk top z in base_link (default 0, equal to base)")
    ap.add_argument("--board-yaw", type=float, default=0.0,
                    help="board X-axis yaw vs base +X (deg, default 0)")
    ap.add_argument("--frames-per-corner", type=int, default=30)
    ap.add_argument("--parent", default="base_link")
    args = ap.parse_args()

    rclpy.init()
    cap = Capturer()
    # wait for intrinsics
    t0 = time.time()
    while cap.K is None and time.time() - t0 < 5.0:
        rclpy.spin_once(cap, timeout_sec=0.1)
    if cap.K is None:
        print("FAIL: no camera_info"); rclpy.shutdown(); sys.exit(1)

    # square corners in base_link (board ORIGIN at each corner)
    R_b_board = rpy_to_R(0, 0, math.radians(args.board_yaw))
    s = args.side
    corner_offsets = [(0, 0), (s, 0), (s, s), (0, s)]
    base_positions = []
    for dx, dy in corner_offsets:
        # offset applied in board-local frame, but since board doesn't rotate
        # between corners, equivalent to base offset rotated by board_yaw
        local = np.array([dx, dy, 0.0])
        world = R_b_board @ local
        base_positions.append(np.array([args.bx, args.by, args.bz]) + world)

    print(f"\nSquare side={s*100:.1f} cm  pos1=({args.bx:.3f},{args.by:.3f},{args.bz:.3f}) m")
    print(f"Board yaw vs base +X: {args.board_yaw:.1f} deg")
    print(f"Expected positions in base_link:")
    for i, p in enumerate(base_positions, 1):
        print(f"  pos{i}: ({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f}) m")
    print()

    estimates = []   # list of (R_base_cam, t_base_cam, t_std)
    cam_t_per_pos = []  # for side-length verification
    for i, p in enumerate(base_positions, 1):
        input(f"Place board at pos{i} (slide only — do not rotate). Press ENTER when ready ...")
        R_cb, t_cb, t_std = cap.collect(args.frames_per_corner, f"pos{i}")
        cam_t_per_pos.append(t_cb.copy())
        # T_base_camera_i = T_base_board_i * inv(T_camera_board_i)
        R_bc = R_b_board @ R_cb.T
        t_bc = p - R_bc @ t_cb
        estimates.append((R_bc, t_bc, t_std))
        rpy = R_to_rpy(R_bc)
        print(f"  pos{i} estimate: t=({t_bc[0]:+.4f},{t_bc[1]:+.4f},{t_bc[2]:+.4f}) "
              f"rpy=({math.degrees(rpy[0]):+.2f},{math.degrees(rpy[1]):+.2f},"
              f"{math.degrees(rpy[2]):+.2f}) deg  "
              f"PnP_std=({t_std[0]*1000:.1f},{t_std[1]*1000:.1f},{t_std[2]*1000:.1f})mm")

    # average final
    Ts = np.array([e[1] for e in estimates])
    t_final = Ts.mean(axis=0)
    t_spread = Ts.std(axis=0)
    qs = [R_to_quat(e[0]) for e in estimates]
    q_final = avg_quat(qs)
    R_final = quat_to_R(q_final)
    rpy = R_to_rpy(R_final)
    qx, qy, qz, qw = q_final

    # side-length verification (camera-frame distances should be `side`)
    sides_measured = []
    for i in range(4):
        j = (i + 1) % 4
        d = np.linalg.norm(cam_t_per_pos[j] - cam_t_per_pos[i])
        sides_measured.append(d)
    diag1 = np.linalg.norm(cam_t_per_pos[2] - cam_t_per_pos[0])
    diag2 = np.linalg.norm(cam_t_per_pos[3] - cam_t_per_pos[1])
    expected_side = s
    expected_diag = s * math.sqrt(2)

    print("\n=== Scale / planarity verification ===")
    for i, d in enumerate(sides_measured):
        err = (d - expected_side) * 1000
        print(f"  side{i+1}->{(i+1)%4+1}: {d*1000:.1f} mm  (expected {expected_side*1000:.1f}, err {err:+.1f} mm)")
    print(f"  diag1-3:   {diag1*1000:.1f} mm  (expected {expected_diag*1000:.1f}, err {(diag1-expected_diag)*1000:+.1f} mm)")
    print(f"  diag2-4:   {diag2*1000:.1f} mm  (expected {expected_diag*1000:.1f}, err {(diag2-expected_diag)*1000:+.1f} mm)")

    print(f"\n=== Per-corner T_base_camera spread (should be small) ===")
    print(f"  t spread: ({t_spread[0]*1000:.2f}, {t_spread[1]*1000:.2f}, "
          f"{t_spread[2]*1000:.2f}) mm   ← if >5mm, accuracy may be limited")

    print(f"\n=== FINAL T_base_camera ({args.parent} → {cap.cam_frame}) ===")
    print(f"  translation (m):   x={t_final[0]:+.4f}  y={t_final[1]:+.4f}  z={t_final[2]:+.4f}")
    print(f"  rpy (deg):         r={math.degrees(rpy[0]):+.3f}  "
          f"p={math.degrees(rpy[1]):+.3f}  y={math.degrees(rpy[2]):+.3f}")
    print(f"  quaternion (xyzw): {qx:+.6f} {qy:+.6f} {qz:+.6f} {qw:+.6f}")

    print(f"\n=== STATIC TRANSFORM PUBLISHER COMMAND ===")
    print(
        f"ros2 run tf2_ros static_transform_publisher \\\n"
        f"    --x {t_final[0]:.4f} --y {t_final[1]:.4f} --z {t_final[2]:.4f} \\\n"
        f"    --qx {qx:.6f} --qy {qy:.6f} --qz {qz:.6f} --qw {qw:.6f} \\\n"
        f"    --frame-id {args.parent} --child-frame-id {cap.cam_frame}"
    )

    cap.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
