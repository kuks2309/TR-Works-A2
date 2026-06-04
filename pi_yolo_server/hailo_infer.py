"""Hailo-8 YOLOv8s inference wrapper for the non-ROS2 detection server.

Runs on the Raspberry Pi 5 (+ Hailo-8 via the AI HAT+). Input is a BGR image
(OpenCV), output is a list of 2D detections in ORIGINAL image pixel coordinates:

    {"class_id": int, "class_name": str, "confidence": float,
     "bbox_xyxy": [x1, y1, x2, y2]}

The 3D back-projection (depth) is NOT done here — that stays on the robot PC.

HEF note
--------
The .hef itself is built elsewhere (Hailo Dataflow Compiler on an x86 box) and
copied to the Pi. Two common output flavours are supported:

  * "nms"  : HEF compiled WITH on-chip nms_postprocess (Hailo Model Zoo default
             for YOLOv8). The output stream yields, per class, an array of rows
             [y_min, x_min, y_max, x_max, score] normalized to the (letterboxed)
             input. This module un-letterboxes them back to original pixels.
  * "raw"  : HEF without NMS. Decoding raw YOLOv8 head tensors is model-specific;
             a host-side NMS helper is provided but the decode step is left as a
             clearly marked TODO for you to fill from your export config.

Set HAILO_MOCK=1 to bypass the NPU entirely and return one dummy centered box,
so the HTTP server + ROS2 bridge round-trip can be verified before the real HEF
(or before the Hailo hardware) is available.
"""

from __future__ import annotations

import os
from typing import List, Optional, Tuple

import cv2
import numpy as np


# --------------------------------------------------------------------- helpers

def letterbox(
    img: np.ndarray, new_size: int = 640, color: Tuple[int, int, int] = (114, 114, 114)
) -> Tuple[np.ndarray, float, Tuple[float, float]]:
    """Resize keeping aspect ratio and pad to a square `new_size` (YOLO style).

    Returns (padded_img, ratio, (pad_w, pad_h)). `ratio` is the single scale
    factor applied to the original image; pads are in padded-image pixels.
    """
    h, w = img.shape[:2]
    ratio = min(new_size / h, new_size / w)
    nh, nw = int(round(h * ratio)), int(round(w * ratio))
    resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    pad_w = (new_size - nw) / 2.0
    pad_h = (new_size - nh) / 2.0
    top, bottom = int(round(pad_h - 0.1)), int(round(pad_h + 0.1))
    left, right = int(round(pad_w - 0.1)), int(round(pad_w + 0.1))
    padded = cv2.copyMakeBorder(
        resized, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color
    )
    return padded, ratio, (left, top)


def unletterbox_xyxy(
    box: Tuple[float, float, float, float],
    ratio: float,
    pad: Tuple[float, float],
    orig_w: int,
    orig_h: int,
) -> List[float]:
    """Map a box from letterboxed-input pixels back to original image pixels."""
    x1, y1, x2, y2 = box
    pad_w, pad_h = pad
    x1 = (x1 - pad_w) / ratio
    y1 = (y1 - pad_h) / ratio
    x2 = (x2 - pad_w) / ratio
    y2 = (y2 - pad_h) / ratio
    x1 = float(np.clip(x1, 0, orig_w - 1))
    y1 = float(np.clip(y1, 0, orig_h - 1))
    x2 = float(np.clip(x2, 0, orig_w - 1))
    y2 = float(np.clip(y2, 0, orig_h - 1))
    return [x1, y1, x2, y2]


def nms_numpy(
    boxes: np.ndarray, scores: np.ndarray, iou_thr: float
) -> List[int]:
    """Plain NumPy NMS. boxes: (N,4) xyxy. Returns kept indices (host 'raw' path)."""
    if boxes.size == 0:
        return []
    x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    areas = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
    order = scores.argsort()[::-1]
    keep: List[int] = []
    while order.size > 0:
        i = order[0]
        keep.append(int(i))
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])
        w = np.maximum(0.0, xx2 - xx1)
        h = np.maximum(0.0, yy2 - yy1)
        inter = w * h
        iou = inter / (areas[i] + areas[order[1:]] - inter + 1e-9)
        order = order[1:][iou <= iou_thr]
    return keep


def load_labels(path: Optional[str]) -> List[str]:
    if path and os.path.isfile(path):
        with open(path, "r", encoding="utf-8") as fh:
            return [
                ln.strip() for ln in fh
                if ln.strip() and not ln.lstrip().startswith("#")
            ]
    return []


# ----------------------------------------------------------------- inferencer

class HailoYoloV8:
    def __init__(
        self,
        hef_path: str,
        labels: Optional[List[str]] = None,
        input_size: int = 640,
        conf: float = 0.35,
        iou: float = 0.5,
        output_format: str = "nms",
    ) -> None:
        self.hef_path = hef_path
        self.labels = labels or []
        self.input_size = int(input_size)
        self.conf = float(conf)
        self.iou = float(iou)
        self.output_format = output_format
        self.mock = os.environ.get("HAILO_MOCK", "0") == "1"

        if self.mock:
            print("[hailo_infer] HAILO_MOCK=1 -> returning dummy detections (no NPU)")
            self._infer_engine = None
            return

        # ---- HailoRT init -------------------------------------------------
        # `hailo_platform` ships with HailoRT (install on the Pi via the Hailo
        # apt/pip packages or `hailo-all` on Raspberry Pi OS).
        from hailo_platform import (  # noqa: F401  (imported lazily so mock works without HailoRT)
            HEF, VDevice, ConfigureParams, HailoStreamInterface,
            InferVStreams, InputVStreamParams, OutputVStreamParams, FormatType,
        )

        self._vdevice = VDevice()
        self._hef = HEF(self.hef_path)
        cfg = ConfigureParams.create_from_hef(
            self._hef, interface=HailoStreamInterface.PCIe
        )
        self._network_group = self._vdevice.configure(self._hef, cfg)[0]
        self._ng_params = self._network_group.create_params()
        self._in_info = self._hef.get_input_vstream_infos()[0]
        self._out_infos = self._hef.get_output_vstream_infos()
        self._in_params = InputVStreamParams.make(
            self._network_group, format_type=FormatType.UINT8
        )
        self._out_params = OutputVStreamParams.make(
            self._network_group, format_type=FormatType.FLOAT32
        )
        self._InferVStreams = InferVStreams
        print(f"[hailo_infer] HEF loaded: {self.hef_path} "
              f"input='{self._in_info.name}' outputs={len(self._out_infos)}")

    def _class_name(self, class_id: int) -> str:
        if 0 <= class_id < len(self.labels):
            return self.labels[class_id]
        return str(class_id)

    def infer(self, bgr: np.ndarray, conf: Optional[float] = None) -> List[dict]:
        conf_thr = self.conf if conf is None else float(conf)
        orig_h, orig_w = bgr.shape[:2]

        if self.mock:
            cx, cy = orig_w / 2.0, orig_h / 2.0
            bw, bh = orig_w * 0.3, orig_h * 0.3
            return [{
                "class_id": 0,
                "class_name": self._class_name(0),
                "confidence": 0.99,
                "bbox_xyxy": [cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2],
            }]

        padded, ratio, pad = letterbox(bgr, self.input_size)
        # Hailo expects RGB uint8, NHWC.
        rgb = cv2.cvtColor(padded, cv2.COLOR_BGR2RGB)
        net_input = np.expand_dims(rgb, axis=0).astype(np.uint8)

        with self._InferVStreams(self._network_group, self._in_params, self._out_params) as pipe:
            with self._network_group.activate(self._ng_params):
                results = pipe.infer({self._in_info.name: net_input})

        if self.output_format == "nms":
            return self._parse_nms(results, ratio, pad, orig_w, orig_h, conf_thr)
        return self._parse_raw(results, ratio, pad, orig_w, orig_h, conf_thr)

    def _parse_nms(self, results, ratio, pad, orig_w, orig_h, conf_thr) -> List[dict]:
        """Parse Hailo on-chip NMS output: per-class arrays of
        [y_min, x_min, y_max, x_max, score] normalized to the input square."""
        out = results[self._out_infos[0].name]
        # HailoRT returns this as a list-of-arrays (one per class), sometimes
        # wrapped in a length-1 batch list. Normalize to per-class arrays.
        per_class = out[0] if isinstance(out, (list, tuple)) and len(out) == 1 else out
        dets: List[dict] = []
        s = self.input_size
        for class_id, arr in enumerate(per_class):
            if arr is None or len(arr) == 0:
                continue
            for row in arr:
                ymin, xmin, ymax, xmax, score = (float(v) for v in row[:5])
                if score < conf_thr:
                    continue
                box = unletterbox_xyxy(
                    (xmin * s, ymin * s, xmax * s, ymax * s), ratio, pad, orig_w, orig_h
                )
                dets.append({
                    "class_id": class_id,
                    "class_name": self._class_name(class_id),
                    "confidence": score,
                    "bbox_xyxy": box,
                })
        return dets

    def _parse_raw(self, results, ratio, pad, orig_w, orig_h, conf_thr) -> List[dict]:
        """Decode + host-NMS path for HEFs WITHOUT on-chip NMS.

        TODO(you): YOLOv8 raw head decoding is specific to your export
        (anchors/strides/reg-max, output tensor names and layout). Fill the
        decode below to produce `boxes` (N,4 xyxy in INPUT pixels), `scores`
        (N,), `class_ids` (N,), then the generic NMS + un-letterbox runs.
        """
        raise NotImplementedError(
            "raw-output decoding not implemented; compile the HEF with "
            "nms_postprocess (output_format='nms') or fill in _parse_raw()."
        )
