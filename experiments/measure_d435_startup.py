#!/usr/bin/env python3
"""D435 구동(콜드/웜 스타트) 시간 측정 — pyrealsense2 직접.

종료(stop) 후 다시 start 할 때 걸리는 시간을 분해:
  start_ms       : rs.pipeline().start(cfg) 호출 소요 (device open + USB 협상 + 스트림 구성)
  firstframe_ms  : start 직후 첫 wait_for_frames() 까지 (첫 프레임 도착)
  total          : start_ms + firstframe_ms (= "go" 부터 첫 사용가능 프레임까지)

trial 0 = 콜드(장치 유휴 후 첫 기동), trial 1+ = 같은 프로세스 내 재기동(웜).
production 구성(color+depth+align)을 기본으로 한다.
"""
from __future__ import annotations

import argparse
import time

import numpy as np
import pyrealsense2 as rs

W, H, FPS = 640, 480, 30


def one_trial(use_depth: bool, align_depth: bool):
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, W, H, rs.format.bgr8, FPS)
    if use_depth:
        cfg.enable_stream(rs.stream.depth, W, H, rs.format.z16, FPS)
    t0 = time.monotonic()
    profile = pipe.start(cfg)
    t_start = (time.monotonic() - t0) * 1e3

    align = rs.align(rs.stream.color) if (use_depth and align_depth) else None
    t1 = time.monotonic()
    frames = pipe.wait_for_frames()         # blocks until first frameset
    if align is not None:
        frames = align.process(frames)
    _ = np.asanyarray(frames.get_color_frame().get_data())
    t_first = (time.monotonic() - t1) * 1e3

    pipe.stop()
    return t_start, t_first


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-n", "--trials", type=int, default=5)
    ap.add_argument("--gap", type=float, default=2.0, help="seconds between stop and next start")
    ap.add_argument("--no-depth", action="store_true")
    ap.add_argument("--no-align", action="store_true")
    args = ap.parse_args()

    use_depth = not args.no_depth
    align_depth = not args.no_align
    print(f"D435 startup: {W}x{H}@{FPS}  color"
          f"{'+depth' if use_depth else ''}{'+align' if (use_depth and align_depth) else ''}  "
          f"trials={args.trials}  gap={args.gap}s\n")
    print(f"{'trial':<7}{'start_ms':>12}{'firstframe_ms':>16}{'total_ms':>12}")
    rows = []
    for i in range(args.trials):
        s, f = one_trial(use_depth, align_depth)
        tag = "cold" if i == 0 else f"warm{i}"
        print(f"{tag:<7}{s:12.1f}{f:16.1f}{s + f:12.1f}")
        rows.append((s, f))
        if i < args.trials - 1:
            time.sleep(args.gap)

    warm = rows[1:] if len(rows) > 1 else rows
    ws = sum(r[0] for r in warm) / len(warm)
    wf = sum(r[1] for r in warm) / len(warm)
    print(f"\ncold total : {rows[0][0] + rows[0][1]:.1f} ms  (start {rows[0][0]:.1f} + first {rows[0][1]:.1f})")
    print(f"warm mean  : {ws + wf:.1f} ms  (start {ws:.1f} + first {wf:.1f})  over {len(warm)} restart(s)")


if __name__ == "__main__":
    main()
