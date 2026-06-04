# pi_yolo_server — Hailo-8 YOLOv8s detection appliance (non-ROS2)

A tiny HTTP server that runs a **custom-trained YOLOv8s** on a **Raspberry Pi 5
+ Hailo-8** and returns 2D bounding boxes. It is the remote half of the
on-demand box-detection pipeline: the robot PC's ROS2 `yolo_remote_node` posts
one color frame per `DetectBox` goal and does the 3D back-projection itself
(depth never leaves the robot PC).

```
[robot PC : ROS2]                              [Raspberry Pi 5 : non-ROS2]
 RealSense D435  --buffer color/depth-->         pi_yolo_server (this)
 yolo_remote_node --POST jpeg (10.42.0.2:8080)--> HailoRT + yolov8s.hef
                  <----- detections JSON --------/
 (+ local depth -> 3D, ~/detections, box_plane)
```

## Network (direct link, no router)

- robot PC `enp1s0` : `10.42.0.1/24`
- Raspberry Pi 5 `eth0` : `10.42.0.2/24` (+ Wi-Fi for internet)

Server binds `0.0.0.0:8080` so it is reachable at `http://10.42.0.2:8080`.

## Install on the Pi

```bash
# 1) HailoRT + PCIe driver (Raspberry Pi OS, AI HAT+)
sudo apt update && sudo apt install -y hailo-all
hailortcli fw-control identify        # sanity check: Hailo-8 detected

# 2) Python deps for this server
cd ~/pi_yolo_server
python3 -m venv .venv --system-site-packages   # keep system hailo_platform visible
source .venv/bin/activate
pip install -r requirements.txt
```

> `--system-site-packages` lets the venv see the apt-installed `hailo_platform`
> (HailoRT). If you install HailoRT via pip instead, a plain venv is fine.

## Model + labels

- Copy your compiled `yolov8s.hef` next to `server.py` (or set `HEF_PATH`).
- Edit `labels.txt` so line N is the class name for `class_id == N`, matching
  your training order.
- Build the HEF on an **x86** machine with the Hailo Dataflow Compiler /
  Model Zoo, compiled **with `nms_postprocess`** (default `OUTPUT_FORMAT=nms`).
  Without on-chip NMS, set `OUTPUT_FORMAT=raw` and implement `_parse_raw()`.

## Run

```bash
# real inference
HEF_PATH=yolov8s.hef LABELS_PATH=labels.txt python3 server.py

# plumbing test WITHOUT the NPU / a real HEF (returns one dummy centered box)
HAILO_MOCK=1 python3 server.py
```

## Quick test (from the robot PC)

```bash
curl -s http://10.42.0.2:8080/health
curl -s --data-binary @frame.jpg -H 'Content-Type: image/jpeg' \
     'http://10.42.0.2:8080/detect?conf=0.4' | jq .
```

## Run as a service (autostart)

```bash
sudo cp yolo_server.service /etc/systemd/system/
# edit WorkingDirectory / ExecStart / User / Environment in the unit first
sudo systemctl daemon-reload
sudo systemctl enable --now yolo_server
journalctl -u yolo_server -f
```

## Deploy from the dev PC

```bash
# from China/pi_yolo_server/ on the robot/dev PC
./deploy.sh            # rsync this folder to pi@10.42.0.2:~/pi_yolo_server
```

## HTTP contract

`POST /detect` — body: raw JPEG bytes; query: `conf`, `iou` (optional).

```json
{
  "model": "yolov8s.hef",
  "infer_ms": 12.3,
  "image_size": [480, 640],
  "detections": [
    {"class_id": 0, "class_name": "box", "confidence": 0.91,
     "bbox_xyxy": [x1, y1, x2, y2]}
  ]
}
```

`bbox_xyxy` is in original (posted) image pixels. The robot PC pairs these boxes
with its own aligned-depth frame to recover 3D points, so this server stays a
pure 2D detector.
