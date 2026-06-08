#!/usr/bin/env python3
"""On-site colour calibration helper for the place-box HSV classifier.

Measures the colour signature of the currently-shown box wall and appends it to
a calibration table, so the HSV hue boundaries in src/color_classifier.cpp can be
re-tuned to the actual camera/lighting at the deployment site.

USAGE (one colour at a time; place that colour's box in front, then):
    python3 color_calib.py <label>        # label = true colour, e.g. red/orange/...
    # writes/append /tmp/color_calib.csv

PREREQUISITES (verify they are actually publishing, not just "running"):
    - D435 cloud : /camera/camera/depth/color/points
    - TOF range  : /tof/range            (openarmx_tof_driver tof_serial_driver)
    - body TF    : openarmx_body_link0 <- camera_depth_optical_frame
                   (needs robot_state_publisher, e.g. 하드웨어 SIL)

It detects the wall by GEOMETRY (TOF ROI in x + above-desk vertical-plane RANSAC,
vertical-constrained), so the colour is read from the wall only — not the desk or
the small boxes. Falls back to all-above-desk if the TOF ROI is empty.

After collecting every colour, read the H_med column and set each adjacent
boundary between the two medians (mind overlapping p05/p95 tails), then update
src/color_classifier.cpp hsvToColor() and rebuild.
"""
import sys, time, csv, os
import numpy as np
import rclpy
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy,
                       QoSHistoryPolicy)
from sensor_msgs.msg import PointCloud2, Range
from sensor_msgs_py import point_cloud2 as pc2
import tf2_ros
from rclpy.duration import Duration

LABEL = sys.argv[1] if len(sys.argv) > 1 else "unknown"
CSV = os.environ.get("COLOR_CALIB_CSV", "/tmp/color_calib.csv")
BODY = "openarmx_body_link0"
CAM = "camera_depth_optical_frame"
ROI_TOL = 0.10   # ROI half-width in x (m) around the TOF standoff


def q2R(a, b, c, d):
    n = a*a + b*b + c*c + d*d
    s = 2/n
    return np.array([[1-(b*b+c*c)*s, (a*b-c*d)*s, (a*c+b*d)*s],
                     [(a*b+c*d)*s, 1-(a*a+c*c)*s, (b*c-a*d)*s],
                     [(a*c-b*d)*s, (b*c+a*d)*s, 1-(a*a+b*b)*s]])


def ransac_vertical(pts, thr, it=600, max_nz=0.20, seed=1):
    """Largest plane parallel to +z (vertical), like PCL SACMODEL_PARALLEL_PLANE."""
    if pts.shape[0] < 3:
        return None
    rng = np.random.default_rng(seed)
    bm, bc = None, -1
    for _ in range(it):
        i = rng.choice(pts.shape[0], 3, replace=False)
        p0, p1, p2 = pts[i]
        nv = np.cross(p1-p0, p2-p0)
        L = np.linalg.norm(nv)
        if L < 1e-9 or abs((nv/L)[2]) > max_nz:
            continue
        nv /= L
        d = -nv @ p0
        m = np.abs(pts @ nv + d) < thr
        c = int(m.sum())
        if c > bc:
            bc, bm = c, m
    return bm


def main():
    q = QoSProfile(depth=5)
    q.reliability = QoSReliabilityPolicy.BEST_EFFORT
    q.durability = QoSDurabilityPolicy.VOLATILE
    q.history = QoSHistoryPolicy.KEEP_LAST
    rclpy.init()
    nd = rclpy.create_node("color_calib")
    st = {"cloud": None, "tof": []}
    nd.create_subscription(PointCloud2, "/camera/camera/depth/color/points",
                           lambda m: st.__setitem__("cloud", m), q)
    nd.create_subscription(Range, "/tof/range",
                           lambda m: st["tof"].append(float(m.range)), q)
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, nd)

    t = time.time()
    while time.time()-t < 15 and st["cloud"] is None:
        rclpy.spin_once(nd, timeout_sec=0.05)
    for _ in range(60):
        rclpy.spin_once(nd, timeout_sec=0.05)
    if st["cloud"] is None:
        print(f"[{LABEL}] NO CLOUD in 15s — verify camera/TF are publishing")
        return 1

    d_tof = float(np.median(st["tof"][-15:])) if st["tof"] else 0.5
    sp = pc2.read_points(st["cloud"], field_names=["x", "y", "z", "rgb"], skip_nans=True)
    cam = np.stack([sp["x"], sp["y"], sp["z"]], 1).astype(np.float64)
    u = np.asarray(sp["rgb"]).view(np.uint32)
    R = ((u >> 16) & 255).astype(float)
    G = ((u >> 8) & 255).astype(float)
    B = (u & 255).astype(float)
    ts = buf.lookup_transform(BODY, CAM, rclpy.time.Time(), timeout=Duration(seconds=3.0))
    tr = ts.transform.translation
    rq = ts.transform.rotation
    body = cam @ q2R(rq.x, rq.y, rq.z, rq.w).T + np.array([tr.x, tr.y, tr.z])
    z, x = body[:, 2], body[:, 0]
    dz = np.median(z[(z > 0.65) & (z < 0.80)])

    roi = (z > dz+0.03) & (x > d_tof-ROI_TOL) & (x < d_tof+ROI_TOL)
    if int(roi.sum()) < 200:
        print(f"[{LABEL}] TOF ROI sparse ({int(roi.sum())}pt @ {d_tof:.3f}m) -> fallback all above-desk")
        roi = (z > dz+0.03)
    P, Rr, Gr, Br = body[roi], R[roi], G[roi], B[roi]
    mk = ransac_vertical(P, 0.008)
    if mk is None or mk.sum() < 100:
        print(f"[{LABEL}] no vertical wall (above-desk pts={P.shape[0]})")
        return 1
    Ri, Gi, Bi = Rr[mk], Gr[mk], Br[mk]

    mx = np.maximum(np.maximum(Ri, Gi), Bi)
    mn = np.minimum(np.minimum(Ri, Gi), Bi)
    df = mx - mn
    sat = np.where(mx <= 0, 0, df/np.maximum(mx, 1e-9))
    val = mx/255.0
    h = np.zeros_like(mx)
    nz = df > 0
    rmax = (mx == Ri) & nz
    gmax = (mx == Gi) & nz & ~rmax
    bmax = (mx == Bi) & nz & ~rmax & ~gmax
    h[rmax] = 60*(((Gi[rmax]-Bi[rmax])/df[rmax]) % 6)
    h[gmax] = 60*((Bi[gmax]-Ri[gmax])/df[gmax]+2)
    h[bmax] = 60*((Ri[bmax]-Gi[bmax])/df[bmax]+4)
    ok = (sat >= 0.25) & (val >= 0.15)
    H = h[ok]

    sig = dict(label=LABEL, n=int(mk.sum()),
               R=round(Ri.mean(), 1), G=round(Gi.mean(), 1), B=round(Bi.mean(), 1),
               H_med=round(float(np.median(H)), 1),
               H_p05=round(float(np.percentile(H, 5)), 1),
               H_p95=round(float(np.percentile(H, 95)), 1),
               GR=round(Gi.mean()/max(Ri.mean(), 1e-6), 3),
               BR=round(Bi.mean()/max(Ri.mean(), 1e-6), 3),
               sat=round(float(sat[ok].mean()), 2),
               val=round(float(val[ok].mean()), 2), dist=round(d_tof, 3))
    print(f"[{LABEL}] RGB=({sig['R']},{sig['G']},{sig['B']}) "
          f"H_med={sig['H_med']} (p05={sig['H_p05']},p95={sig['H_p95']}) "
          f"G/R={sig['GR']} B/R={sig['BR']} sat={sig['sat']} val={sig['val']} "
          f"n={sig['n']} @ {sig['dist']}m")
    new = not os.path.exists(CSV)
    with open(CSV, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(sig.keys()))
        if new:
            w.writeheader()
        w.writerow(sig)
    print(f"  -> appended to {CSV}")
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
