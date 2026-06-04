#!/usr/bin/env python3
"""Live mini-box segmentation viewer: D435 (robot PC) -> remote Hailo seg server.

Grabs the RealSense D435 color stream on the robot PC, POSTs each frame to the
remote segmentation server (pi_yolo_server on the Raspberry Pi 5 + Hailo-8),
and overlays the returned masks (polygon) / boxes / class labels / mask centroid
live. non-ROS2.

The server must run with TASK=seg; this viewer requests ?masks=1 so polygons are
returned for the overlay (the picking path normally omits them).

keys: q/ESC quit, s snapshot
"""

from __future__ import annotations

import argparse
import json
import time
import urllib.error
import urllib.request
from urllib.parse import urlencode

import cv2
import numpy as np

W, H, FPS = 640, 480, 30
# BGR colours matching the class names (mini-box-blue/green/orange/red/yellow)
CLASS_COLORS = {0: (255, 0, 0), 1: (0, 200, 0), 2: (0, 165, 255),
                3: (0, 0, 255), 4: (0, 255, 255)}


def request_remote(url: str, jpeg: bytes, timeout: float) -> dict:
    req = urllib.request.Request(
        url, data=jpeg, method="POST", headers={"Content-Type": "image/jpeg"})
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read().decode("utf-8"))


def draw(frame: np.ndarray, dets: list) -> np.ndarray:
    out = frame.copy()
    overlay = out.copy()
    for d in dets:                                   # filled masks first (translucent)
        color = CLASS_COLORS.get(int(d.get("class_id", -1)), (200, 200, 200))
        poly = d.get("polygon")
        if poly and len(poly) >= 3:
            cv2.fillPoly(overlay, [np.array(poly, np.int32)], color)
    out = cv2.addWeighted(overlay, 0.45, out, 0.55, 0)
    for d in dets:                                   # boxes + centroid + labels
        color = CLASS_COLORS.get(int(d.get("class_id", -1)), (200, 200, 200))
        x1, y1, x2, y2 = [int(v) for v in d["bbox_xyxy"]]
        cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
        cen = d.get("centroid_px", [(x1 + x2) / 2, (y1 + y2) / 2])
        cx, cy = int(cen[0]), int(cen[1])
        cv2.circle(out, (cx, cy), 4, (255, 255, 255), -1)
        cv2.circle(out, (cx, cy), 5, color, 2)
        label = f"{d.get('class_name', '?')} {d.get('confidence', 0.0):.2f}"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        yt = max(0, y1 - th - 6)
        cv2.rectangle(out, (x1, yt), (x1 + tw + 2, yt + th + 6), color, -1)
        tc = (0, 0, 0) if sum(color) > 400 else (255, 255, 255)
        cv2.putText(out, label, (x1 + 1, yt + th + 1),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, tc, 1)
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description="Live remote Hailo-8 seg viewer (D435)")
    ap.add_argument("--server", default="http://10.42.0.2:8080/detect")
    ap.add_argument("--conf", type=float, default=0.3)
    ap.add_argument("--jpeg-quality", type=int, default=80)
    ap.add_argument("--timeout", type=float, default=5.0)
    args = ap.parse_args()

    import pyrealsense2 as rs
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, W, H, rs.format.bgr8, FPS)
    pipe.start(cfg)

    url = f"{args.server}?{urlencode({'conf': args.conf, 'masks': 1})}"
    win = "D435 -> Hailo-8 seg (remote)  [q/ESC quit, s snapshot]"
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
    print(f"[viewer] {W}x{H} -> {url}")

    t0, n, fps = time.time(), 0, 0.0
    try:
        for _ in range(10):
            pipe.wait_for_frames()
        while True:
            fr = pipe.wait_for_frames().get_color_frame()
            if not fr:
                continue
            frame = np.asanyarray(fr.get_data())
            ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), args.jpeg_quality])
            dets = []
            err = None
            if ok:
                try:
                    dets = request_remote(url, bytes(buf), args.timeout).get("detections", [])
                except (urllib.error.URLError, OSError, ValueError) as exc:
                    err = str(exc)

            out = draw(frame, dets)
            n += 1
            if time.time() - t0 >= 1.0:
                fps, t0, n = n / (time.time() - t0), time.time(), 0
            cv2.putText(out, f"FPS:{fps:4.1f}  det:{len(dets)}", (10, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            if err:
                cv2.putText(out, f"server: {err}", (10, H - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            cv2.imshow(win, out)
            k = cv2.waitKey(1) & 0xFF
            if k in (ord("q"), 27):
                break
            if k == ord("s"):
                cv2.imwrite("remote_seg_snapshot.jpg", out)
                print("[viewer] saved remote_seg_snapshot.jpg")
    finally:
        pipe.stop()
        cv2.destroyAllWindows()
        print("[viewer] done")


if __name__ == "__main__":
    main()
