#!/usr/bin/env python3
"""Extract a mid-time PointCloud2 frame from each rosbag, transform into
base_link using /tf_static, then run RANSAC twice:
  1) ground plane (largest, horizontal, lowest)
  2) box-top plane (horizontal, above ground, second-largest)
"""
import sys
import math
import argparse
from pathlib import Path

import numpy as np
import open3d as o3d

import rclpy.serialization as ser
import rosbag2_py
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped


def quat_to_mat(qx, qy, qz, qw):
    n = qx*qx + qy*qy + qz*qz + qw*qw
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = qx*qx*s, qy*qy*s, qz*qz*s
    xy, xz, yz = qx*qy*s, qx*qz*s, qy*qz*s
    wx, wy, wz = qw*qx*s, qw*qy*s, qw*qz*s
    return np.array([
        [1.0 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1.0 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1.0 - (xx + yy)],
    ])


def tf_to_mat(t: TransformStamped):
    M = np.eye(4)
    M[:3, :3] = quat_to_mat(t.transform.rotation.x, t.transform.rotation.y,
                             t.transform.rotation.z, t.transform.rotation.w)
    M[:3, 3] = (t.transform.translation.x, t.transform.translation.y,
                t.transform.translation.z)
    return M


def chain_transform(parents, transforms, src, dst):
    """Return 4x4 matrix that maps points expressed in `src` frame to `dst` frame.
    transforms[child] = (parent, T_parent_child)  where T maps child->parent.
    """
    # build src -> root chain
    def to_root(frame):
        chain = []
        cur = frame
        while cur in transforms:
            par, T = transforms[cur]
            chain.append((par, T))
            cur = par
        return chain, cur  # chain of (parent, T_parent_cur), root frame

    src_chain, src_root = to_root(src)
    dst_chain, dst_root = to_root(dst)
    if src_root != dst_root:
        return None
    # T_root_src
    T_root_src = np.eye(4)
    for _, T in src_chain:
        T_root_src = T @ T_root_src
    T_root_dst = np.eye(4)
    for _, T in dst_chain:
        T_root_dst = T @ T_root_dst
    T_dst_src = np.linalg.inv(T_root_dst) @ T_root_src
    return T_dst_src


def read_bag(bag_dir: Path):
    so = rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3")
    co = rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                     output_serialization_format="cdr")
    reader = rosbag2_py.SequentialReader()
    reader.open(so, co)
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    return reader, type_map


def collect_static_tf(bag_dir: Path):
    reader, _ = read_bag(bag_dir)
    storage_filter = rosbag2_py.StorageFilter(topics=["/tf_static", "/tf"])
    reader.set_filter(storage_filter)
    transforms = {}  # child -> (parent, T)
    while reader.has_next():
        topic, data, _ = reader.read_next()
        msg = ser.deserialize_message(data, TFMessage)
        for t in msg.transforms:
            child = t.child_frame_id.lstrip("/")
            parent = t.header.frame_id.lstrip("/")
            if child not in transforms:  # keep first (static), TF dynamic only fills missing
                transforms[child] = (parent, tf_to_mat(t))
    return transforms


def extract_sampled_cloud(bag_dir: Path, topic: str, n_frames: int = 5):
    """Pick `n_frames` evenly-spaced messages of `topic` and concatenate
    their points into one (N,3) array."""
    reader, _ = read_bag(bag_dir)
    storage_filter = rosbag2_py.StorageFilter(topics=[topic])
    reader.set_filter(storage_filter)
    msgs = []
    while reader.has_next():
        _, data, ts = reader.read_next()
        msgs.append((ts, data))
    if not msgs:
        return None, None
    msgs.sort(key=lambda x: x[0])
    if len(msgs) <= n_frames:
        picks = msgs
    else:
        idxs = np.linspace(0, len(msgs) - 1, n_frames).astype(int)
        picks = [msgs[i] for i in idxs]
    parts = []
    frame = None
    for _, raw in picks:
        cloud = ser.deserialize_message(raw, PointCloud2)
        if frame is None:
            frame = cloud.header.frame_id.lstrip("/")
        s = pc2.read_points(cloud, field_names=["x", "y", "z"], skip_nans=True)
        parts.append(np.stack([np.asarray(s["x"], dtype=np.float64),
                               np.asarray(s["y"], dtype=np.float64),
                               np.asarray(s["z"], dtype=np.float64)], axis=1))
    return frame, np.vstack(parts) if parts else None


def transform_points(P, T):
    if P.size == 0:
        return P
    H = np.hstack([P, np.ones((P.shape[0], 1))])
    return (T @ H.T).T[:, :3]


def fit_plane(points, distance_threshold=0.01, ransac_n=3, num_iter=2000):
    if points.shape[0] < ransac_n:
        return None
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    plane, inliers = pcd.segment_plane(distance_threshold=distance_threshold,
                                       ransac_n=ransac_n,
                                       num_iterations=num_iter)
    return np.array(plane), np.asarray(inliers)


def normalize_plane_z_up(coef):
    # Force the plane normal to point +Z so a,b,c,d are deterministic.
    a, b, c, d = coef
    if c < 0:
        a, b, c, d = -a, -b, -c, -d
    return np.array([a, b, c, d])


def angle_to_z(coef):
    a, b, c, _ = coef
    n = np.array([a, b, c]) / max(np.linalg.norm([a, b, c]), 1e-9)
    cos = abs(n[2])
    return math.degrees(math.acos(min(max(cos, -1.0), 1.0)))


def height_at_origin(coef):
    # plane: a x + b y + c z + d = 0  ->  z@(0,0) = -d/c
    a, b, c, d = coef
    if abs(c) < 1e-9:
        return None
    return -d / c


def fmt_plane(coef):
    a, b, c, d = coef
    return f"{a:+.4f} x  {b:+.4f} y  {c:+.4f} z  {d:+.4f} = 0"


def analyze_cloud(label, frame, pts):
    """Analyze a single camera cloud in its own optical frame.

    Returns dict with ground / box_top plane results (in `frame` coords)."""
    # crop to camera-forward sector: z=forward, depth in 0.2..2.5 m,
    # |x|<1.5 (right), |y|<1.5 (down). Filters out NaNs and far walls.
    m = (pts[:, 2] > 0.2) & (pts[:, 2] < 2.5) \
        & (np.abs(pts[:, 0]) < 1.5) & (np.abs(pts[:, 1]) < 1.5)
    pts = pts[m]
    print(f"  [{label}] frame={frame}, points after crop = {pts.shape[0]}")
    if pts.shape[0] < 1000:
        print(f"  [{label}] not enough points")
        return None

    candidates = []
    remaining = pts.copy()
    for _ in range(4):
        if remaining.shape[0] < 500:
            break
        coef, inl = fit_plane(remaining, distance_threshold=0.01)
        if coef is None:
            break
        # canonicalize: flip so plane equation has positive d (origin offset)
        if coef[3] < 0:
            coef = -coef
        candidates.append({
            "coef": coef,
            "n_inliers": int(inl.size),
            "inlier_pts": remaining[inl],
            "centroid": remaining[inl].mean(axis=0),
        })
        keep = np.ones(remaining.shape[0], bool)
        keep[inl] = False
        remaining = remaining[keep]

    if not candidates:
        print(f"  [{label}] no plane")
        return None

    # In the RealSense optical frame, +y = down. Flip every plane equation
    # so its normal points to +y (b >= 0). Then the plane farther below the
    # camera origin has a more-negative d (since plane: a x + b y + c z + d = 0
    # → at camera origin the value is d, and a +y-pointing normal means a
    # plane below the origin yields d<0 with |d|=distance).
    for cand in candidates:
        a, b, c, d = cand["coef"]
        if b < 0:
            cand["coef"] = np.array([-a, -b, -c, -d])

    # Use the largest plane as reference for "horizontal"-ness.
    largest = max(candidates, key=lambda c: c["n_inliers"])
    ref_n = largest["coef"][:3] / np.linalg.norm(largest["coef"][:3])

    parallel = []
    for cand in candidates:
        n = cand["coef"][:3] / np.linalg.norm(cand["coef"][:3])
        ang = math.degrees(math.acos(min(abs(np.dot(n, ref_n)), 1.0)))
        cand["angle_to_ref_deg"] = ang
        print(f"  [{label}]   cand: inliers={cand['n_inliers']:6d}  "
              f"angle_to_ref={ang:5.2f} deg  d={cand['coef'][3]:+.3f}  "
              f"cy={cand['centroid'][1]:+.3f}  plane={fmt_plane(cand['coef'])}")
        if ang < 15.0 and cand["n_inliers"] >= 1500:
            parallel.append(cand)

    if not parallel:
        return None

    # Ground = plane whose `d` (with normal flipped to +y) is most negative,
    # i.e. the plane farthest below the camera origin.
    parallel.sort(key=lambda c: c["coef"][3])
    ground = parallel[0]

    g_n = ground["coef"][:3] / np.linalg.norm(ground["coef"][:3])
    def signed_dist_to_ground(p):
        a, b, c, d = ground["coef"]
        x, y, z = p
        return (a * x + b * y + c * z + d) / np.linalg.norm([a, b, c])

    # Box top: planes above ground (d > ground.d + 3 cm in canonical form);
    # pick the one with the most inliers.
    above = [c for c in parallel[1:] if c["coef"][3] > ground["coef"][3] + 0.03]
    box_top = max(above, key=lambda c: c["n_inliers"]) if above else None
    if box_top is not None:
        box_top["gap_to_ground"] = signed_dist_to_ground(box_top["centroid"])
        box_top["angle_to_ground_deg"] = box_top["angle_to_ref_deg"]
        box_top["height_above_ground_d"] = (box_top["coef"][3]
                                            - ground["coef"][3])

    print(f"  [{label}] GROUND : {fmt_plane(ground['coef'])} | inliers={ground['n_inliers']}")
    if box_top:
        print(f"  [{label}] BOX TOP: {fmt_plane(box_top['coef'])} | inliers={box_top['n_inliers']}, "
              f"parallel_angle={box_top['angle_to_ground_deg']:.2f} deg, "
              f"box_top_above_ground={box_top['gap_to_ground']:.4f} m (along camera-Y)")
    else:
        print(f"  [{label}] BOX TOP: not detected")
    return {"frame": frame, "ground": ground, "box_top": box_top}


def analyze_bag(bag_dir: Path):
    print(f"\n========= {bag_dir.name} =========")
    out = {}
    for cam_topic, label in [
        ("/d435_center/d435_center/depth/color/points", "center"),
        ("/d435_center_upper/d435_center_upper/depth/color/points", "upper"),
    ]:
        frame, pts = extract_sampled_cloud(bag_dir, cam_topic, n_frames=5)
        if pts is None or pts.size == 0:
            print(f"  [{label}] empty / missing")
            continue
        out[label] = analyze_cloud(label, frame, pts)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default="bags/20260506")
    ap.add_argument("--bag", default=None,
                    help="single bag dir name; default = all bags under --root")
    args = ap.parse_args()

    root = Path(args.root)
    bag_dirs = ([root / args.bag] if args.bag
                else sorted(p for p in root.iterdir() if p.is_dir()))

    results = {}
    for bd in bag_dirs:
        if not (bd / "metadata.yaml").exists():
            continue
        try:
            results[bd.name] = analyze_bag(bd)
        except Exception as e:
            print(f"  ERROR on {bd.name}: {e}")
            import traceback; traceback.print_exc()

    print("\n========= SUMMARY =========")
    for name, r in results.items():
        if not r:
            print(f"  {name}: (no result)")
            continue
        for cam, info in r.items():
            if info is None:
                print(f"  {name}/{cam}: (no result)")
                continue
            g = info["ground"]; b = info["box_top"]
            line = f"  {name}/{cam} (frame={info['frame']}):"
            line += f"\n    ground : {fmt_plane(g['coef'])}"
            if b is not None:
                line += (f"\n    box_top: {fmt_plane(b['coef'])}"
                         f"   gap={b['gap_to_ground']:.4f} m, "
                         f"parallel={b['angle_to_ground_deg']:.2f} deg")
            else:
                line += "\n    box_top: not detected"
            print(line)


if __name__ == "__main__":
    main()
