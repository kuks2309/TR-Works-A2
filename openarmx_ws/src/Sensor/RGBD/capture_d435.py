#!/usr/bin/env python3
"""D435 color (+optional depth) capture-and-save tool for dataset collection (non-ROS2).

Captures the RealSense D435 color stream at 640x480 and saves frames to disk.
Optionally (--depth) also saves the aligned 16-bit depth image (3D) next to each
color frame, sharing the same timestamp so pairs stay matched.

Two explicit modes:

  * windowed  (default)      : live preview; press SPACE to save, 'q'/ESC to quit
  * headless  (--no-window)  : no display (e.g. Pi over SSH); auto-saves on the
                               --auto interval, optionally stopping after --count

Pure pyrealsense2 + OpenCV, no ROS2. Simplified from
kuks2309/forklift_ros2_ws ds435_stereo_view/stereo_viewer.py (color + optional
depth, fixed 640x480, dataset-oriented naming).

Each run saves into its own session subfolder under --out (created on first save):
    --out/<session>/{prefix}_{YYYYmmdd_HHMMSS_mmm}.{ext}        color (jpg/png)
    --out/<session>/{prefix}_{YYYYmmdd_HHMMSS_mmm}_depth.png    aligned depth, 16-bit mm (only with --depth)
where <session> defaults to the start time YYYYmmdd_HHMMSS (override with --session NAME).

Examples
--------
# Color only, windowed: SPACE=save, q/ESC=quit
python3 capture_d435.py --out dataset/box

# Color + depth, headless on the Pi over SSH: 200 pairs @ 0.5 s, then stop
python3 capture_d435.py --out dataset/box --depth --no-window --auto 0.5 --count 200
"""

from __future__ import annotations

import argparse
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np


class RealSenseD435:
    """RealSense D435 color (BGR8) + optional aligned depth (Z16), fixed res/fps."""

    def __init__(self, width: int, height: int, fps: int, with_depth: bool = False):
        import pyrealsense2 as rs
        self._pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        self._align = None
        if with_depth:
            config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
            self._align = rs.align(rs.stream.color)  # align depth to the color frame
        self._pipeline.start(config)
        # D435 auto-exposure/white-balance needs a few frames to settle.
        for _ in range(10):
            self._pipeline.wait_for_frames()

    def read(self):
        """Return (color_bgr, depth_uint16_or_None). depth is None unless enabled."""
        frames = self._pipeline.wait_for_frames()
        if self._align is not None:
            frames = self._align.process(frames)
        color = frames.get_color_frame()
        if not color:
            return None, None
        color_img = np.asanyarray(color.get_data())
        depth_img = None
        if self._align is not None:
            depth = frames.get_depth_frame()
            if depth:
                depth_img = np.asanyarray(depth.get_data())  # uint16, millimeters
        return color_img, depth_img

    def close(self):
        self._pipeline.stop()


def timestamp() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]  # millisecond precision


def save_pair(color, depth, out_dir: Path, prefix: str, ext: str, jpeg_quality: int):
    """Save color (and depth if given) sharing one timestamp. Returns (color_path, depth_path|None)."""
    ts = timestamp()
    color_path = out_dir / f"{prefix}_{ts}.{ext}"
    if ext == "jpg":
        cv2.imwrite(str(color_path), color, [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality])
    else:
        cv2.imwrite(str(color_path), color)
    depth_path = None
    if depth is not None:
        # 16-bit PNG preserves the raw uint16 millimeter depth (lossless 3D data).
        depth_path = out_dir / f"{prefix}_{ts}_depth.png"
        cv2.imwrite(str(depth_path), depth)
    return color_path, depth_path


def main() -> None:
    p = argparse.ArgumentParser(description="D435 color (+optional depth) capture+save (640x480, non-ROS2)")
    p.add_argument("--out", default="captures", help="output directory (created if missing)")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--ext", default="jpg", choices=["jpg", "png"], help="color image format")
    p.add_argument("--jpeg-quality", type=int, default=95, help="1..100 (jpg color only)")
    p.add_argument("--prefix", default="img", help="filename prefix")
    p.add_argument("--depth", action="store_true",
                   help="also save the aligned 16-bit depth image (3D) per frame")
    p.add_argument("--auto", type=float, default=0.0,
                   help="auto-save every N seconds (0 = manual 's' only)")
    p.add_argument("--count", type=int, default=0,
                   help="stop after N saved frames (0 = until quit/Ctrl+C)")
    p.add_argument("--no-window", action="store_true",
                   help="headless: no preview window (requires --auto)")
    p.add_argument("--session", default=None,
                   help="session subfolder name under --out (default: start time YYYYmmdd_HHMMSS)")
    args = p.parse_args()

    if args.no_window and args.auto <= 0.0:
        p.error("--no-window requires --auto > 0 (no 's' key without a window)")

    # Each run saves into its own session subfolder under --out. The folder is
    # created lazily on the first save so quitting without saving leaves nothing.
    session = args.session or datetime.now().strftime("%Y%m%d_%H%M%S")
    session_dir = Path(args.out) / session

    try:
        src = RealSenseD435(args.width, args.height, args.fps, with_depth=args.depth)
    except ImportError:
        print("[capture] pyrealsense2 not installed.", file=sys.stderr)
        sys.exit(2)
    except Exception as exc:  # noqa: BLE001 (no device / busy / unsupported profile)
        print(f"[capture] failed to start RealSense: {exc}", file=sys.stderr)
        sys.exit(2)

    windowed = not args.no_window
    win = "D435 capture (SPACE=save, q/ESC=quit)"
    if windowed:
        cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)

    print(f"[capture] D435 color {args.width}x{args.height}@{args.fps} "
          f"{'+depth ' if args.depth else ''}-> {session_dir.resolve()} "
          f"({args.ext}, auto={args.auto}s, count={args.count or 'inf'})")
    saved = 0
    last_auto = time.monotonic()
    try:
        while True:
            color, depth = src.read()
            if color is None:
                continue

            do_save = False
            now = time.monotonic()
            if args.auto > 0.0 and (now - last_auto) >= args.auto:
                do_save = True
                last_auto = now

            if windowed:
                view = color.copy()
                cv2.putText(view, f"saved: {saved}", (8, 24),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
                cv2.imshow(win, view)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):  # q or ESC
                    break
                if key in (ord(" "), ord("s")):  # SPACE (or s) = save
                    do_save = True

            if do_save:
                session_dir.mkdir(parents=True, exist_ok=True)  # lazy: only on first save
                color_path, depth_path = save_pair(
                    color, depth if args.depth else None,
                    session_dir, args.prefix, args.ext, args.jpeg_quality,
                )
                saved += 1
                extra = f" (+depth {depth_path.name})" if depth_path else ""
                print(f"[capture] saved {saved}: {color_path.name}{extra}")
                if args.count and saved >= args.count:
                    print(f"[capture] reached count={args.count}, stopping")
                    break
    except KeyboardInterrupt:
        print("\n[capture] interrupted")
    finally:
        src.close()
        if windowed:
            cv2.destroyAllWindows()
        if saved:
            print(f"[capture] done. {saved} frame(s) saved to {session_dir.resolve()}")
        else:
            print("[capture] done. 0 frames saved (no session folder created)")


if __name__ == "__main__":
    main()
