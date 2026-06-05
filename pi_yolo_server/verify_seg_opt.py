#!/usr/bin/env python3
"""Verify the proto-scale centroid optimisation against the old 640-resize path.

Runs ONE NPU inference, then computes centroids/areas BOTH ways on the cached
tensors and reports (1) accuracy parity (centroid diff in original px) and
(2) isolated post-processing speed. Run with the systemd server STOPPED.
"""
import os, sys, time, statistics
import cv2, numpy as np

os.environ.setdefault("HAILO_MOCK", "0")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from hailo_seg import HailoYoloSeg, postprocess, make_mask, mask_centroids, load_labels, NET

IMG = sys.argv[1] if len(sys.argv) > 1 else "/tmp/cmp_frame.jpg"
CONF, IOU, ITERS = 0.35, 0.5, 30

det = HailoYoloSeg("yolov8s_seg_minibox.hef", labels=load_labels("labels.txt"), conf=CONF, iou=IOU)
bgr = cv2.imread(IMG)
oh, ow = bgr.shape[:2]
sx, sy = ow / NET, oh / NET

inp = np.expand_dims(cv2.cvtColor(cv2.resize(bgr, (NET, NET)), cv2.COLOR_BGR2RGB), 0).astype(np.uint8)
out = det._infer.infer({det._in_name: inp})
fb, sc, ci, co, proto = postprocess(out, CONF, IOU)
print(f"img {ow}x{oh}  detections={len(fb)}")

def old_path(fb, co, proto):
    res = []
    for box, coeff in zip(fb, co):
        mask = make_mask(proto, coeff, box)
        ys, xs = np.where(mask)
        if xs.size > 0:
            res.append((float(xs.mean()), float(ys.mean()), int(xs.size)))
        else:
            res.append((0.5*(box[0]+box[2]), 0.5*(box[1]+box[3]), 0))
    return res

old = old_path(fb, co, proto)
new = mask_centroids(proto, np.asarray(co, dtype=np.float32), fb)

# --- accuracy parity (original-pixel centroid difference) ---
diffs, adiffs = [], []
for (ox, oy, oa), n in zip(old, new):
    nx, ny, na = n if n is not None else (0.5*0, 0.5*0, 0)
    diffs.append(((ox-nx)*sx)**2 + ((oy-ny)*sy)**2)
    adiffs.append(abs(oa - na))
diffs = [d**0.5 for d in diffs]
print("\nACCURACY (old 640-resize centroid  vs  new 160-proto centroid):")
print(f"  centroid diff (orig px): mean {statistics.mean(diffs):.3f}  max {max(diffs):.3f}")
print(f"  mask-area px count diff (640-space): mean {statistics.mean(adiffs):.0f}  max {max(adiffs):.0f}")

# --- isolated post-processing speed (NPU output cached) ---
def t_old():
    t = time.monotonic(); old_path(fb, co, proto); return (time.monotonic()-t)*1e3
def t_new():
    coeffs = np.asarray(co, dtype=np.float32)
    t = time.monotonic(); mask_centroids(proto, coeffs, fb); return (time.monotonic()-t)*1e3
for _ in range(5): t_old(); t_new()
O = [t_old() for _ in range(ITERS)]
N = [t_new() for _ in range(ITERS)]
print(f"\nPOST-PROCESSING (n={ITERS}, NPU excluded):")
print(f"  OLD (per-det 640 mask): mean {statistics.mean(O):6.2f} ms  median {statistics.median(O):6.2f}")
print(f"  NEW (proto-160 batch) : mean {statistics.mean(N):6.2f} ms  median {statistics.median(N):6.2f}")
print(f"  speedup: {statistics.mean(O)/statistics.mean(N):.1f}x   saved ~{statistics.mean(O)-statistics.mean(N):.1f} ms/frame")
det.close()
