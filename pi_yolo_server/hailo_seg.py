"""Hailo-8 YOLOv8-seg inference + mask-centroid for the detection server.

Runs the mini-box segmentation HEF on the Raspberry Pi 5 (+ Hailo-8) and, per
instance, returns the bounding box AND the segmentation mask's centre of mass
(centroid) in ORIGINAL posted-image pixel coordinates:

    {"class_id": int, "class_name": str, "confidence": float,
     "bbox_xyxy": [x1,y1,x2,y2], "centroid_px": [cx,cy], "mask_area_px": int}

The centroid (computed from the actual mask pixels, not the bbox centre) is what
the robot PC back-projects to 3D for picking.

Post-processing (DFL decode + NMS + proto mask) is ported verbatim from the
project's proven hailo_camera_seg.py. The model was trained/exported with a PLAIN
resize (640x480 -> 640x640, no letterbox), so results are scaled back per-axis
(sx = orig_w/640, sy = orig_h/640).

Set HAILO_MOCK=1 to return one dummy instance without the NPU (plumbing tests).
"""

from __future__ import annotations

import contextlib
import os
from typing import List, Optional

import cv2
import numpy as np

NET = 640  # model input square


def sigmoid(x):
    return 1.0 / (1.0 + np.exp(-np.clip(x, -50, 50)))


def _find_prefix(outputs) -> str:
    for k in outputs:
        if k.endswith("conv44"):
            return k.rsplit("/conv44", 1)[0]
    return list(outputs)[0].rsplit("/", 1)[0]


def postprocess(outputs, conf, iou):
    """Hailo seg outputs (10 tensors) -> boxes(xyxy, 640 space), scores, class_ids,
    mask_coeffs, proto. Ported from hailo_camera_seg.py."""
    p = _find_prefix(outputs)
    scales = [
        (f"{p}/conv44", f"{p}/conv45", f"{p}/conv46", 8),
        (f"{p}/conv60", f"{p}/conv61", f"{p}/conv62", 16),
        (f"{p}/conv73", f"{p}/conv74", f"{p}/conv75", 32),
    ]
    proto = outputs[f"{p}/conv48"][0]  # (160,160,32)
    boxes, scores, cls_ids, coeffs = [], [], [], []
    grid16 = np.arange(16, dtype=np.float32)
    for bb_k, cl_k, mk_k, stride in scales:
        bbox = outputs[bb_k][0]   # (h,w,64)
        cls = outputs[cl_k][0]    # (h,w,nc) sigmoid baked in
        mask = outputs[mk_k][0]   # (h,w,32)
        mx = cls.max(-1)
        ids = cls.argmax(-1)
        ys, xs = np.where(mx > conf)
        for y, x in zip(ys, xs):
            raw = bbox[y, x].reshape(4, 16)
            e = np.exp(raw - raw.max(1, keepdims=True))
            dfl = (e / e.sum(1, keepdims=True) * grid16).sum(1)  # (4,) ltrb
            cx, cy = (x + 0.5) * stride, (y + 0.5) * stride
            x1, y1 = cx - dfl[0] * stride, cy - dfl[1] * stride
            x2, y2 = cx + dfl[2] * stride, cy + dfl[3] * stride
            boxes.append([x1, y1, x2 - x1, y2 - y1])
            scores.append(float(mx[y, x]))
            cls_ids.append(int(ids[y, x]))
            coeffs.append(mask[y, x])
    if not boxes:
        return [], [], [], [], proto
    idx = cv2.dnn.NMSBoxes(boxes, scores, conf, iou)
    if len(idx) == 0:
        return [], [], [], [], proto
    idx = np.array(idx).flatten()
    fb = [[boxes[i][0], boxes[i][1], boxes[i][0] + boxes[i][2], boxes[i][1] + boxes[i][3]] for i in idx]
    return fb, [scores[i] for i in idx], [cls_ids[i] for i in idx], [coeffs[i] for i in idx], proto


def make_mask(proto, coeff, box) -> np.ndarray:
    """proto(160,160,32) @ coeff(32,) -> sigmoid -> 640 mask, cropped to box, >0.5."""
    m = sigmoid(proto @ coeff)        # (160,160)
    m = cv2.resize(m, (NET, NET))     # 640x640
    x1, y1, x2, y2 = [int(c) for c in box]
    out = np.zeros((NET, NET), np.float32)
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(NET, x2), min(NET, y2)
    if x2 > x1 and y2 > y1:
        out[y1:y2, x1:x2] = m[y1:y2, x1:x2]
    return out > 0.5


def load_labels(path: Optional[str]) -> List[str]:
    if path and os.path.isfile(path):
        with open(path, "r", encoding="utf-8") as fh:
            return [ln.strip() for ln in fh
                    if ln.strip() and not ln.lstrip().startswith("#")]
    return []


class HailoYoloSeg:
    def __init__(
        self,
        hef_path: str,
        labels: Optional[List[str]] = None,
        input_size: int = 640,
        conf: float = 0.25,
        iou: float = 0.45,
        **_ignored,
    ) -> None:
        self.hef_path = hef_path
        self.labels = labels or []
        self.input_size = int(input_size)
        self.conf = float(conf)
        self.iou = float(iou)
        self.mock = os.environ.get("HAILO_MOCK", "0") == "1"

        if self.mock:
            print("[hailo_seg] HAILO_MOCK=1 -> dummy instance (no NPU)")
            self._infer = None
            return

        from hailo_platform import HEF, VDevice, FormatType
        from hailo_platform.pyhailort.pyhailort import (
            InferVStreams, InputVStreamParams, OutputVStreamParams)

        self._hef = HEF(self.hef_path)
        self._in_name = self._hef.get_input_vstream_infos()[0].name
        # Configure the device ONCE and keep the infer pipeline alive for the
        # server's lifetime (per-request infer() is then just a tensor call).
        self._stack = contextlib.ExitStack()
        target = self._stack.enter_context(VDevice())
        self._ng = target.configure(self._hef)[0]
        ivp = InputVStreamParams.make(self._ng, format_type=FormatType.UINT8)
        ovp = OutputVStreamParams.make(self._ng, format_type=FormatType.FLOAT32)
        self._infer = self._stack.enter_context(InferVStreams(self._ng, ivp, ovp))
        self._stack.enter_context(self._ng.activate())
        print(f"[hailo_seg] HEF loaded: {self.hef_path} input='{self._in_name}'")

    def _name(self, cid: int) -> str:
        return self.labels[cid] if 0 <= cid < len(self.labels) else str(cid)

    def close(self):
        if not self.mock and hasattr(self, "_stack"):
            self._stack.close()

    def infer(self, bgr: np.ndarray, conf: Optional[float] = None,
              return_polygon: bool = False) -> List[dict]:
        conf_thr = self.conf if conf is None else float(conf)
        oh, ow = bgr.shape[:2]
        sx, sy = ow / NET, oh / NET

        if self.mock:
            cx, cy = ow / 2.0, oh / 2.0
            bw, bh = ow * 0.3, oh * 0.3
            det = {
                "class_id": 0, "class_name": self._name(0), "confidence": 0.99,
                "bbox_xyxy": [cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2],
                "centroid_px": [cx, cy], "mask_area_px": int(bw * bh),
            }
            if return_polygon:
                det["polygon"] = [[cx - bw / 2, cy - bh / 2], [cx + bw / 2, cy - bh / 2],
                                  [cx + bw / 2, cy + bh / 2], [cx - bw / 2, cy + bh / 2]]
            return [det]

        inp = cv2.cvtColor(cv2.resize(bgr, (NET, NET)), cv2.COLOR_BGR2RGB)
        inp = np.expand_dims(inp, 0).astype(np.uint8)
        out = self._infer.infer({self._in_name: inp})
        fb, sc, ci, co, proto = postprocess(out, conf_thr, self.iou)

        dets: List[dict] = []
        for box, score, cid, coeff in zip(fb, sc, ci, co):
            mask = make_mask(proto, coeff, box)          # (640,640) bool, NET space
            ys, xs = np.where(mask)
            if xs.size > 0:
                cx_net, cy_net = float(xs.mean()), float(ys.mean())
                area = int(xs.size)
            else:  # mask empty -> fall back to bbox centre
                cx_net, cy_net = 0.5 * (box[0] + box[2]), 0.5 * (box[1] + box[3])
                area = 0
            det = {
                "class_id": int(cid),
                "class_name": self._name(int(cid)),
                "confidence": float(score),
                "bbox_xyxy": [box[0] * sx, box[1] * sy, box[2] * sx, box[3] * sy],
                "centroid_px": [cx_net * sx, cy_net * sy],
                "mask_area_px": int(area * sx * sy),
            }
            if return_polygon:
                cnts, _ = cv2.findContours(mask.astype(np.uint8),
                                           cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if cnts:
                    c = max(cnts, key=cv2.contourArea)
                    eps = 0.01 * cv2.arcLength(c, True)
                    poly = cv2.approxPolyDP(c, eps, True).reshape(-1, 2)
                    det["polygon"] = [[float(x * sx), float(y * sy)] for x, y in poly]
            dets.append(det)
        return dets
