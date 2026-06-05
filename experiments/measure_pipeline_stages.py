#!/usr/bin/env python3
"""Step-by-step latency measurement of the remote-Hailo box detection pipeline.

Measures the stages that do NOT require a live D435 (the camera is currently
unplugged), driving the real Pi5+Hailo-8 server with a saved 640x480 D435 frame:

  [2] image transmission   = JPEG encode (robot PC) + network upload
  [3] Pi YOLOv8 compute     = server-reported infer_ms (letterbox+NPU+NMS+centroid)
  [4] result transmission   = JSON response download (part of net overhead)
  [5] box position calc     = 2D centroid -> 3D back-projection (synthetic depth)

Stages [1] camera capture and [6] RViz display need the physical camera + live
stack and are reported separately.

Usage: python3 measure_pipeline_stages.py [--server URL] [--image PATH] [-n N]
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


def stats(xs):
    xs = list(xs)
    return {
        "mean": statistics.mean(xs),
        "median": statistics.median(xs),
        "min": min(xs),
        "max": max(xs),
        "std": statistics.pstdev(xs) if len(xs) > 1 else 0.0,
        "n": len(xs),
    }


def fmt(name, s, unit="ms"):
    return (f"  {name:<34} mean {s['mean']:7.2f}  median {s['median']:7.2f}  "
            f"min {s['min']:7.2f}  max {s['max']:7.2f}  std {s['std']:6.2f} {unit} (n={s['n']})")


def health_rtt(base_url, n=20):
    """Pure HTTP+Flask round-trip baseline (negligible body)."""
    url = base_url.replace("/detect", "/health")
    out = []
    for _ in range(n):
        t0 = time.monotonic()
        with urllib.request.urlopen(url, timeout=5.0) as r:
            r.read()
        out.append((time.monotonic() - t0) * 1e3)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--server", default="http://10.42.0.2:8080/detect")
    ap.add_argument("--image", default="AI/mini-box-segment/cmp_frame.jpg")
    ap.add_argument("--conf", type=float, default=0.35)
    ap.add_argument("--iou", type=float, default=0.5)
    ap.add_argument("--jpeg-quality", type=int, default=80)
    ap.add_argument("-n", "--iters", type=int, default=30)
    ap.add_argument("--warmup", type=int, default=5)
    args = ap.parse_args()

    img = cv2.imread(args.image)
    if img is None:
        raise SystemExit(f"cannot read {args.image}")
    h, w = img.shape[:2]
    print(f"image: {args.image}  {w}x{h}  jpeg_q={args.jpeg_quality}  "
          f"server={args.server}  iters={args.iters} (warmup {args.warmup})\n")

    url = f"{args.server}?{urlencode({'conf': f'{args.conf:.4f}', 'iou': f'{args.iou:.4f}'})}"

    enc_ms, rtt_ms, infer_ms, parse_ms, proj_ms = [], [], [], [], []
    ndet = 0
    jpeg_sizes = []

    # synthetic aligned-depth frame (uint16 mm) + intrinsics ~ D435 640x480
    depth = np.full((h, w), 600, dtype=np.uint16)  # 0.6 m everywhere
    fx = fy = 615.0
    cx, cy = w / 2.0, h / 2.0

    def project(u, v):
        ui, vi = int(round(u)), int(round(v))
        if not (0 <= ui < w and 0 <= vi < h):
            return None
        z = float(depth[vi, ui]) * 1e-3
        if z <= 0:
            return None
        return [(u - cx) * z / fx, (v - cy) * z / fy, z]

    total = args.warmup + args.iters
    for i in range(total):
        # --- [2a] JPEG encode (robot PC) ---
        t = time.monotonic()
        ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), args.jpeg_quality])
        enc = (time.monotonic() - t) * 1e3
        jpeg = bytes(buf)

        # --- [2b+3+4] POST round trip (upload + Pi infer + response) ---
        req = urllib.request.Request(
            url, data=jpeg, method="POST",
            headers={"Content-Type": "image/jpeg", "Content-Length": str(len(jpeg))})
        t = time.monotonic()
        with urllib.request.urlopen(req, timeout=5.0) as r:
            raw = r.read()
        rtt = (time.monotonic() - t) * 1e3

        # --- [4b] client JSON parse ---
        t = time.monotonic()
        resp = json.loads(raw.decode("utf-8"))
        parse = (time.monotonic() - t) * 1e3

        dets = resp.get("detections", [])
        srv_infer = float(resp.get("infer_ms", 0.0))

        # --- [5] 3D back-projection of every detection's centroid ---
        t = time.monotonic()
        for d in dets:
            c = d.get("centroid_px") or [
                0.5 * (d["bbox_xyxy"][0] + d["bbox_xyxy"][2]),
                0.5 * (d["bbox_xyxy"][1] + d["bbox_xyxy"][3])]
            project(c[0], c[1])
        proj = (time.monotonic() - t) * 1e3

        if i >= args.warmup:
            enc_ms.append(enc)
            rtt_ms.append(rtt)
            infer_ms.append(srv_infer)
            parse_ms.append(parse)
            proj_ms.append(proj)
            jpeg_sizes.append(len(jpeg))
            ndet = len(dets)

    overhead = [r - inf for r, inf in zip(rtt_ms, infer_ms)]  # net + Pi decode + Flask
    hb = health_rtt(args.server, n=20)

    print(f"detections per frame: {ndet}   jpeg bytes: {int(statistics.mean(jpeg_sizes))} (mean)\n")
    print("STAGE BREAKDOWN (per detection cycle / single goal):")
    print(fmt("[2a] JPEG encode (robot PC)", stats(enc_ms)))
    print(fmt("[2b+3+4] POST round-trip (RTT)", stats(rtt_ms)))
    print(fmt("   |- [3] Pi YOLOv8 infer (NPU)", stats(infer_ms)))
    print(fmt("   |- [2b+4] net+decode+Flask ovh", stats(overhead)))
    print(fmt("[4b] client JSON parse", stats(parse_ms)))
    print(fmt("[5] 3D back-projection (all dets)", stats(proj_ms)))
    print(fmt("[ref] HTTP /health baseline RTT", stats(hb)))

    e2e = stats([e + r + p + pr for e, r, p, pr in zip(enc_ms, rtt_ms, parse_ms, proj_ms)])
    print("\nMEASURED SUBTOTAL [2a+RTT+parse+5] (excludes camera capture & RViz):")
    print(fmt("end-to-end measurable", e2e))


if __name__ == "__main__":
    main()
