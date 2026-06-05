#!/usr/bin/env python3
"""LIVE end-to-end latency measurement of the box-detection pipeline, stages 1-5,
measured on the SAME live D435 frames (camera owned directly by pyrealsense2).

  [1] camera capture    = D435 wait_for_frames + depth->color align (host)
  [2a] JPEG encode      = robot PC cv2.imencode (q80)
  [2b+3+4] POST RTT     = upload + Pi YOLOv8-seg infer + JSON response
       [3] Pi infer     = server-reported infer_ms
  [5] box position calc = mask centroid -> 3D back-projection (REAL depth+intrinsics)

Production-matched config: color 640x480 bgr8 30fps, aligned_depth_to_color,
intrinsics read from the live color stream. Stage 6 (RViz render) is measured
separately (needs the ROS stack).
"""
from __future__ import annotations

import argparse
import json
import statistics
import time
import urllib.request
from urllib.parse import urlencode

import cv2
import numpy as np
import pyrealsense2 as rs

W, H, FPS = 640, 480, 30


def st(xs):
    xs = list(xs)
    return dict(mean=statistics.mean(xs), median=statistics.median(xs),
               min=min(xs), max=max(xs),
               std=statistics.pstdev(xs) if len(xs) > 1 else 0.0, n=len(xs))


def fmt(name, s):
    return (f"  {name:<36} mean {s['mean']:7.2f}  median {s['median']:7.2f}  "
            f"min {s['min']:7.2f}  max {s['max']:7.2f}  std {s['std']:6.2f} ms (n={s['n']})")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--server", default="http://10.42.0.2:8080/detect")
    ap.add_argument("--conf", type=float, default=0.35)
    ap.add_argument("--iou", type=float, default=0.5)
    ap.add_argument("--jpeg-quality", type=int, default=80)
    ap.add_argument("-n", "--iters", type=int, default=30)
    ap.add_argument("--warmup", type=int, default=10)
    args = ap.parse_args()

    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, W, H, rs.format.bgr8, FPS)
    cfg.enable_stream(rs.stream.depth, W, H, rs.format.z16, FPS)
    profile = pipe.start(cfg)
    align = rs.align(rs.stream.color)

    # real color intrinsics (production reads these from camera_info)
    cprof = profile.get_stream(rs.stream.color).as_video_stream_profile()
    intr = cprof.get_intrinsics()
    fx, fy, cx, cy = intr.fx, intr.fy, intr.ppx, intr.ppy
    dscale = profile.get_device().first_depth_sensor().get_depth_scale()
    print(f"D435 live: {W}x{H}@{FPS}  intr fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}  "
          f"depth_scale={dscale}")
    print(f"server={args.server} jpeg_q={args.jpeg_quality} iters={args.iters} "
          f"(warmup {args.warmup})\n")

    url = f"{args.server}?{urlencode({'conf': f'{args.conf:.4f}', 'iou': f'{args.iou:.4f}'})}"

    cap_ms, enc_ms, rtt_ms, infer_ms, proj_ms = [], [], [], [], []
    ndets, jbytes = [], []

    def project(u, v, depth):
        ui, vi = int(round(u)), int(round(v))
        if not (0 <= ui < W and 0 <= vi < H):
            return None
        z = float(depth[vi, ui]) * dscale
        if z <= 0:
            return None
        return [(u - cx) * z / fx, (v - cy) * z / fy, z]

    total = args.warmup + args.iters
    for i in range(total):
        # --- [1] capture: wait_for_frames + align depth->color ---
        t = time.monotonic()
        frames = pipe.wait_for_frames()
        frames = align.process(frames)
        color = frames.get_color_frame()
        depth = frames.get_depth_frame()
        if not color or not depth:
            continue
        bgr = np.asanyarray(color.get_data())
        depth_np = np.asanyarray(depth.get_data())
        cap = (time.monotonic() - t) * 1e3

        # --- [2a] JPEG encode ---
        t = time.monotonic()
        ok, buf = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), args.jpeg_quality])
        enc = (time.monotonic() - t) * 1e3
        if not ok:
            continue
        jpeg = bytes(buf)

        # --- [2b+3+4] POST round trip ---
        req = urllib.request.Request(
            url, data=jpeg, method="POST",
            headers={"Content-Type": "image/jpeg", "Content-Length": str(len(jpeg))})
        t = time.monotonic()
        with urllib.request.urlopen(req, timeout=5.0) as r:
            raw = r.read()
        rtt = (time.monotonic() - t) * 1e3
        resp = json.loads(raw.decode("utf-8"))
        dets = resp.get("detections", [])
        srv = float(resp.get("infer_ms", 0.0))

        # --- [5] 3D back-projection (real depth) ---
        t = time.monotonic()
        for d in dets:
            c = d.get("centroid_px") or [0.5 * (d["bbox_xyxy"][0] + d["bbox_xyxy"][2]),
                                         0.5 * (d["bbox_xyxy"][1] + d["bbox_xyxy"][3])]
            project(c[0], c[1], depth_np)
        proj = (time.monotonic() - t) * 1e3

        if i >= args.warmup:
            cap_ms.append(cap); enc_ms.append(enc); rtt_ms.append(rtt)
            infer_ms.append(srv); proj_ms.append(proj)
            ndets.append(len(dets)); jbytes.append(len(jpeg))

    pipe.stop()

    overhead = [r - inf for r, inf in zip(rtt_ms, infer_ms)]
    e2e = [c + e + r + p for c, e, r, p in zip(cap_ms, enc_ms, rtt_ms, proj_ms)]
    print(f"detections/frame: mean {statistics.mean(ndets):.1f} (min {min(ndets)} max {max(ndets)})   "
          f"jpeg bytes: mean {int(statistics.mean(jbytes))}\n")
    print("LIVE STAGE BREAKDOWN (single detection goal):")
    print(fmt("[1] D435 capture + align", st(cap_ms)))
    print(fmt("[2a] JPEG encode (robot PC)", st(enc_ms)))
    print(fmt("[2b+3+4] POST round-trip (RTT)", st(rtt_ms)))
    print(fmt("   |- [3] Pi YOLOv8-seg infer (NPU)", st(infer_ms)))
    print(fmt("   |- [2b+4] net+Pi-decode+Flask ovh", st(overhead)))
    print(fmt("[5] 3D back-projection (all dets)", st(proj_ms)))
    print(fmt("[1+2+3+4+5] E2E (excl. RViz render)", st(e2e)))


if __name__ == "__main__":
    main()
