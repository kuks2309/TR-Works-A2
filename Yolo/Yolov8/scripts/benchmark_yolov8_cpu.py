#!/usr/bin/env python3
"""CPU inference-speed benchmark for the yolov8_detection node's YOLO models.

Mirrors the exact predict() call used in
3d_detect_ws/src/yolov8_detection/yolov8_detection/yolov8_node.py::_process:

    self._yolo.predict(source=cv_bgr, conf=..., iou=..., imgsz=...,
                       classes=..., device='cpu', verbose=False)

For YOLO-World models the run script (scripts/run_yolov8_ros.sh) sets
prompts="cardboard box,box,carton,package" and confidence=0.10, so we call
set_classes() before timing to reproduce production conditions.

Reports per-frame latency (mean/median/p95/min/max), FPS, and whether the
10 Hz (<=100 ms/frame) real-time target is met. Pure CPU; no ROS required.
"""

from __future__ import annotations

import argparse
import os
import statistics
import time
from typing import List, Optional

import numpy as np
from ultralytics import YOLO  # NOTE: importing ultralytics first sets OMP_NUM_THREADS=1
import torch


def make_frame(h: int, w: int, seed: int = 0) -> np.ndarray:
    """Deterministic, mildly structured BGR frame (uint8 HxWx3).

    Structured (gradient + blocks) rather than pure noise so the NMS/postprocess
    load is closer to a real scene than to a worst-case noise field.
    """
    rng = np.random.default_rng(seed)
    yy, xx = np.mgrid[0:h, 0:w]
    base = ((xx * 255 // max(w, 1)).astype(np.uint8))
    img = np.stack([base, (yy * 255 // max(h, 1)).astype(np.uint8),
                    ((xx + yy) % 256).astype(np.uint8)], axis=-1)
    # A few solid rectangles to give the detector something box-like.
    for _ in range(6):
        x1, y1 = rng.integers(0, w - 40), rng.integers(0, h - 40)
        x2, y2 = x1 + rng.integers(20, 40), y1 + rng.integers(20, 40)
        img[y1:y2, x1:x2] = rng.integers(0, 255, size=3, dtype=np.uint8)
    return np.ascontiguousarray(img)


def bench_model(model_path: str, prompts: Optional[List[str]], conf: float,
                iou: float, imgsz: int, frame: np.ndarray,
                warmup: int, iters: int) -> dict:
    model = YOLO(model_path)
    model.to("cpu")
    used_prompts = None
    if prompts and hasattr(model, "set_classes"):
        model.set_classes(prompts)
        used_prompts = prompts

    def run_once():
        return model.predict(source=frame, conf=conf, iou=iou, imgsz=imgsz,
                             device="cpu", verbose=False)

    for _ in range(warmup):
        run_once()

    wall: List[float] = []
    pre = inf = post = 0.0
    n_det_last = 0
    for _ in range(iters):
        t0 = time.perf_counter()
        res = run_once()
        wall.append((time.perf_counter() - t0) * 1000.0)
        sp = res[0].speed  # ms: {'preprocess','inference','postprocess'}
        pre += sp.get("preprocess", 0.0)
        inf += sp.get("inference", 0.0)
        post += sp.get("postprocess", 0.0)
        n_det_last = 0 if res[0].boxes is None else len(res[0].boxes)

    wall_sorted = sorted(wall)
    p95 = wall_sorted[min(len(wall_sorted) - 1, int(round(0.95 * (len(wall_sorted) - 1))))]
    mean = statistics.mean(wall)
    return {
        "model": model_path,
        "prompts": used_prompts,
        "imgsz": imgsz,
        "conf": conf,
        "iters": iters,
        "mean_ms": mean,
        "median_ms": statistics.median(wall),
        "p95_ms": p95,
        "min_ms": min(wall),
        "max_ms": max(wall),
        "fps": 1000.0 / mean if mean > 0 else float("inf"),
        "ul_preprocess_ms": pre / iters,
        "ul_inference_ms": inf / iters,
        "ul_postprocess_ms": post / iters,
        "n_det_last": n_det_last,
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--height", type=int, default=480, help="D435 color height")
    ap.add_argument("--width", type=int, default=640, help="D435 color width")
    ap.add_argument("--warmup", type=int, default=3)
    ap.add_argument("--iters", type=int, default=20)
    ap.add_argument("--conf", type=float, default=0.10)
    ap.add_argument("--iou", type=float, default=0.5)
    ap.add_argument("--prompts", type=str,
                    default="cardboard box,box,carton,package")
    ap.add_argument("--models", type=str, nargs="+", required=True)
    ap.add_argument("--threads", type=int, default=0,
                    help="torch intra-op threads (0 = leave as ultralytics default = 1)")
    args = ap.parse_args()

    if args.threads > 0:
        torch.set_num_threads(args.threads)

    prompts = [p.strip() for p in args.prompts.split(",") if p.strip()]
    frame = make_frame(args.height, args.width)

    try:
        load1, load5, load15 = os.getloadavg()
    except OSError:
        load1 = load5 = load15 = float("nan")

    print("=" * 78)
    print("YOLOv8 CPU inference benchmark")
    print(f"torch={torch.__version__}  cuda_available={torch.cuda.is_available()}  "
          f"threads={torch.get_num_threads()}  OMP_NUM_THREADS={os.environ.get('OMP_NUM_THREADS')}")
    print(f"loadavg(1/5/15)={load1:.2f}/{load5:.2f}/{load15:.2f}  cpus={os.cpu_count()}")
    print(f"frame={args.width}x{args.height}  imgsz={args.imgsz}  conf={args.conf}  "
          f"iou={args.iou}  warmup={args.warmup}  iters={args.iters}")
    print("=" * 78)

    rows = []
    for m in args.models:
        use_prompts = prompts if "world" in m.lower() else None
        r = bench_model(m, use_prompts, args.conf, args.iou, args.imgsz,
                        frame, args.warmup, args.iters)
        rows.append(r)
        target = "PASS" if r["mean_ms"] <= 100.0 else "FAIL"
        print(f"\n--- {m} ---")
        print(f"  prompts        : {r['prompts']}")
        print(f"  detections(last): {r['n_det_last']}")
        print(f"  mean   : {r['mean_ms']:8.1f} ms   ({r['fps']:.2f} FPS)")
        print(f"  median : {r['median_ms']:8.1f} ms")
        print(f"  p95    : {r['p95_ms']:8.1f} ms")
        print(f"  min/max: {r['min_ms']:.1f} / {r['max_ms']:.1f} ms")
        print(f"  ultralytics speed  pre={r['ul_preprocess_ms']:.1f}  "
              f"inf={r['ul_inference_ms']:.1f}  post={r['ul_postprocess_ms']:.1f} ms")
        print(f"  10Hz (<=100ms) -> {target}")

    print("\n" + "=" * 78)
    print(f"{'model':28} {'mean_ms':>9} {'fps':>7} {'10Hz':>6}")
    print("-" * 78)
    for r in rows:
        name = r["model"].split("/")[-1]
        print(f"{name:28} {r['mean_ms']:9.1f} {r['fps']:7.2f} "
              f"{'PASS' if r['mean_ms'] <= 100 else 'FAIL':>6}")
    try:
        print(f"loadavg after run (1/5/15) = "
              f"{os.getloadavg()[0]:.2f}/{os.getloadavg()[1]:.2f}/{os.getloadavg()[2]:.2f}")
    except OSError:
        pass
    print("=" * 78)


if __name__ == "__main__":
    main()
