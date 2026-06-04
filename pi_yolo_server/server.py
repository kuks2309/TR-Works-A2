"""Non-ROS2 YOLOv8s detection HTTP server for Raspberry Pi 5 + Hailo-8.

Endpoints
---------
POST /detect            body = raw JPEG bytes, query: ?conf=<f>&iou=<f>
    -> {"model": str, "infer_ms": float, "image_size": [H, W],
        "detections": [{"class_id": int, "class_name": str,
                        "confidence": float, "bbox_xyxy": [x1,y1,x2,y2]}]}
    bbox_xyxy is in ORIGINAL image pixel coordinates.
GET  /health            -> {"status": "ok", "mock": bool, "model": str}

The robot PC's ROS2 yolo_remote_node posts one color frame per DetectBox goal
and adds the 3D back-projection itself (depth never leaves the robot PC).

Config via environment variables:
    HEF_PATH        path to the compiled .hef            (default: yolov8s.hef)
    LABELS_PATH     newline-delimited class names file   (default: labels.txt)
    INPUT_SIZE      model input square size              (default: 640)
    CONF            default confidence threshold         (default: 0.35)
    IOU             default IoU threshold                (default: 0.5)
    OUTPUT_FORMAT   "nms" (on-chip NMS) or "raw"         (default: nms)
    HOST / PORT     bind address                         (default: 0.0.0.0:8080)
    HAILO_MOCK=1    skip the NPU, return a dummy box (for plumbing tests)
"""

from __future__ import annotations

import os
import threading
import time

import cv2
import numpy as np
from flask import Flask, jsonify, request

from hailo_infer import HailoYoloV8, load_labels

HEF_PATH = os.environ.get("HEF_PATH", "yolov8s.hef")
LABELS_PATH = os.environ.get("LABELS_PATH", "labels.txt")
INPUT_SIZE = int(os.environ.get("INPUT_SIZE", "640"))
CONF = float(os.environ.get("CONF", "0.35"))
IOU = float(os.environ.get("IOU", "0.5"))
OUTPUT_FORMAT = os.environ.get("OUTPUT_FORMAT", "nms")
HOST = os.environ.get("HOST", "0.0.0.0")
PORT = int(os.environ.get("PORT", "8080"))

app = Flask(__name__)

_labels = load_labels(LABELS_PATH)
_detector = HailoYoloV8(
    hef_path=HEF_PATH,
    labels=_labels,
    input_size=INPUT_SIZE,
    conf=CONF,
    iou=IOU,
    output_format=OUTPUT_FORMAT,
)
# The Hailo VDevice is single-access; serialize inference across requests.
_infer_lock = threading.Lock()


@app.get("/health")
def health():
    return jsonify({
        "status": "ok",
        "mock": _detector.mock,
        "model": os.path.basename(HEF_PATH),
        "num_labels": len(_labels),
    })


@app.post("/detect")
def detect():
    raw = request.get_data()
    if not raw:
        return jsonify({"error": "empty body; POST raw JPEG bytes"}), 400

    arr = np.frombuffer(raw, dtype=np.uint8)
    bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if bgr is None:
        return jsonify({"error": "could not decode JPEG"}), 400

    try:
        conf = float(request.args.get("conf", CONF))
    except (TypeError, ValueError):
        conf = CONF

    h, w = bgr.shape[:2]
    t0 = time.monotonic()
    with _infer_lock:
        detections = _detector.infer(bgr, conf=conf)
    infer_ms = (time.monotonic() - t0) * 1e3

    return jsonify({
        "model": os.path.basename(HEF_PATH),
        "infer_ms": round(infer_ms, 2),
        "image_size": [h, w],
        "detections": detections,
    })


def main() -> None:
    # threaded=False: keep a single worker so NPU access stays serialized even
    # without the lock; the lock is belt-and-suspenders for any future workers.
    print(f"[server] listening on {HOST}:{PORT} "
          f"(mock={_detector.mock}, model={os.path.basename(HEF_PATH)})")
    app.run(host=HOST, port=PORT, threaded=False)


if __name__ == "__main__":
    main()
