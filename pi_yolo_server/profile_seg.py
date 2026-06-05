#!/usr/bin/env python3
"""Sub-stage profiler for HailoYoloSeg.infer() on the Pi5+Hailo-8.

Breaks the server's single infer_ms into: preprocess (resize+cvtColor) /
NPU dataflow / postprocess (DFL decode + host NMS) / per-detection mask+centroid
loop. Run with the systemd server STOPPED (the VDevice is single-access).
"""
import os, sys, time, statistics
import cv2, numpy as np

os.environ.setdefault("HAILO_MOCK", "0")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from hailo_seg import HailoYoloSeg, postprocess, make_mask, load_labels, NET

IMG = sys.argv[1] if len(sys.argv) > 1 else "/tmp/cmp_frame.jpg"
CONF, IOU, ITERS, WARM = 0.35, 0.5, 30, 5

labels = load_labels("labels.txt")
det = HailoYoloSeg("yolov8s_seg_minibox.hef", labels=labels, conf=CONF, iou=IOU)
bgr = cv2.imread(IMG)
oh, ow = bgr.shape[:2]
sx, sy = ow / NET, oh / NET
print(f"img {ow}x{oh}  conf={CONF} iou={IOU}  iters={ITERS} (warm {WARM})")

def stat(xs):
    return (f"mean {statistics.mean(xs):6.2f}  median {statistics.median(xs):6.2f}  "
            f"min {min(xs):6.2f}  max {max(xs):6.2f}  std {statistics.pstdev(xs):5.2f} ms")

P=[]; N=[]; Q=[]; M=[]; nd=0
for it in range(WARM + ITERS):
    t=time.monotonic()
    inp = cv2.cvtColor(cv2.resize(bgr,(NET,NET)), cv2.COLOR_BGR2RGB)
    inp = np.expand_dims(inp,0).astype(np.uint8)
    p=(time.monotonic()-t)*1e3

    t=time.monotonic()
    out = det._infer.infer({det._in_name: inp})
    n=(time.monotonic()-t)*1e3

    t=time.monotonic()
    fb,sc,ci,co,proto = postprocess(out, CONF, IOU)
    q=(time.monotonic()-t)*1e3

    t=time.monotonic()
    cnt=0
    for box,score,cid,coeff in zip(fb,sc,ci,co):
        mask = make_mask(proto, coeff, box)
        ys,xs = np.where(mask)
        if xs.size>0:
            _cx,_cy = float(xs.mean()), float(ys.mean())
        cnt+=1
    m=(time.monotonic()-t)*1e3

    if it>=WARM:
        P.append(p); N.append(n); Q.append(q); M.append(m); nd=cnt

print(f"\ndetections: {nd}")
print(f"  [a] preprocess (resize+cvtColor) : {stat(P)}")
print(f"  [b] NPU dataflow (infer call)    : {stat(N)}")
print(f"  [c] postprocess (DFL decode+NMS) : {stat(Q)}")
print(f"  [d] mask+centroid loop ({nd} det) : {stat(M)}")
tot=[a+b+c+d for a,b,c,d in zip(P,N,Q,M)]
print(f"  SUM (== server infer_ms)         : {stat(tot)}")
if nd: print(f"  per-detection mask cost          : {statistics.mean(M)/nd:.2f} ms/det")
det.close()
