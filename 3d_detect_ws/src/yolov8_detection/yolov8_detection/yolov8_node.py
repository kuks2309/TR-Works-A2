"""YOLOv8 (Ultralytics) inference node for RealSense D435 color stream.

Subscribes to a ROS 2 `sensor_msgs/Image` topic, runs Ultralytics YOLOv8, and
publishes both an annotated image and a JSON-encoded detection list. Optional
depth + camera_info subscription lets it back-project each detection center
into a 3D point in the camera optical frame.
"""

from __future__ import annotations

import json
import signal
from typing import List, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header, String
from ultralytics import YOLO

import message_filters


class Yolov8Node(Node):
    def __init__(self) -> None:
        super().__init__("yolov8_node")

        self.declare_parameter("model", "yolov8n.pt")
        self.declare_parameter("device", "cpu")
        self.declare_parameter("confidence", 0.35)
        self.declare_parameter("iou", 0.5)
        self.declare_parameter("image_size", 640)
        # Comma-separated class id allow-list. "" or "all" -> no filter.
        # We avoid an array param because rclpy can't infer the element type
        # from an empty YAML list, leaving the parameter uninitialized.
        self.declare_parameter("classes", "")
        # Open-vocabulary prompts for YOLO-World models (e.g. "yolov8s-world.pt").
        # Comma-separated free-form text. Empty -> keep the model's default vocab.
        self.declare_parameter("prompts", "")
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("use_depth", False)
        self.declare_parameter("sync_slop", 0.05)
        self.declare_parameter("publish_annotated", True)

        self._model_name = self.get_parameter("model").value
        self._device = self.get_parameter("device").value
        self._conf = float(self.get_parameter("confidence").value)
        self._iou = float(self.get_parameter("iou").value)
        self._imgsz = int(self.get_parameter("image_size").value)
        cls_param = str(self.get_parameter("classes").value or "").strip()
        if cls_param and cls_param.lower() != "all":
            self._classes: Optional[List[int]] = [int(c) for c in cls_param.split(",") if c.strip()]
        else:
            self._classes = None
        prompts_param = str(self.get_parameter("prompts").value or "").strip()
        self._prompts: Optional[List[str]] = (
            [p.strip() for p in prompts_param.split(",") if p.strip()]
            if prompts_param else None
        )
        self._image_topic = self.get_parameter("image_topic").value
        self._depth_topic = self.get_parameter("depth_topic").value
        self._info_topic = self.get_parameter("camera_info_topic").value
        self._use_depth = bool(self.get_parameter("use_depth").value)
        self._sync_slop = float(self.get_parameter("sync_slop").value)
        self._publish_annotated = bool(self.get_parameter("publish_annotated").value)

        self.get_logger().info(
            f"Loading YOLOv8 model='{self._model_name}' device='{self._device}'"
        )
        self._yolo = YOLO(self._model_name)
        try:
            self._yolo.to(self._device)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(
                f"Could not move model to '{self._device}', falling back to CPU: {exc}"
            )
            self._device = "cpu"
            self._yolo.to("cpu")

        if self._prompts:
            if hasattr(self._yolo, "set_classes"):
                self._yolo.set_classes(self._prompts)
                self.get_logger().info(
                    f"Open-vocabulary prompts active ({len(self._prompts)}): {self._prompts}"
                )
            else:
                self.get_logger().warning(
                    f"Model '{self._model_name}' has no set_classes() — "
                    "prompts ignored. Use a YOLO-World model (e.g. yolov8s-world.pt)."
                )
                self._prompts = None
        self._class_names = self._yolo.names if hasattr(self._yolo, "names") else {}

        self._bridge = CvBridge()
        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value

        self._pub_annotated = self.create_publisher(Image, "~/image_annotated", 10)
        self._pub_detections = self.create_publisher(String, "~/detections", 10)

        self._fx = self._fy = self._cx = self._cy = None
        self._info_sub = None

        if self._use_depth:
            self._info_sub = self.create_subscription(
                CameraInfo, self._info_topic, self._on_camera_info, 10
            )
            color_sub = message_filters.Subscriber(self, Image, self._image_topic, qos_profile=sensor_qos)
            depth_sub = message_filters.Subscriber(self, Image, self._depth_topic, qos_profile=sensor_qos)
            self._sync = message_filters.ApproximateTimeSynchronizer(
                [color_sub, depth_sub], queue_size=10, slop=self._sync_slop
            )
            self._sync.registerCallback(self._on_color_depth)
            self.get_logger().info(
                f"Subscribed (color+depth sync): {self._image_topic} + {self._depth_topic}"
            )
        else:
            self.create_subscription(
                Image, self._image_topic, self._on_color_only, sensor_qos
            )
            self.get_logger().info(f"Subscribed: {self._image_topic}")

        self.get_logger().info(
            f"Publishing: ~/image_annotated, ~/detections (conf>={self._conf}, iou={self._iou}, imgsz={self._imgsz})"
        )

    def _on_camera_info(self, msg: CameraInfo) -> None:
        if self._fx is None:
            k = msg.k
            self._fx, self._fy = float(k[0]), float(k[4])
            self._cx, self._cy = float(k[2]), float(k[5])
            self.get_logger().info(
                f"CameraInfo locked: fx={self._fx:.2f} fy={self._fy:.2f} cx={self._cx:.2f} cy={self._cy:.2f}"
            )

    def _on_color_only(self, color_msg: Image) -> None:
        self._process(color_msg, None)

    def _on_color_depth(self, color_msg: Image, depth_msg: Image) -> None:
        self._process(color_msg, depth_msg)

    def _process(self, color_msg: Image, depth_msg: Optional[Image]) -> None:
        try:
            cv_bgr = self._bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"cv_bridge color decode failed: {exc}")
            return

        results = self._yolo.predict(
            source=cv_bgr,
            conf=self._conf,
            iou=self._iou,
            imgsz=self._imgsz,
            classes=self._classes,
            device=self._device,
            verbose=False,
        )
        if not results:
            return
        result = results[0]

        depth_img: Optional[np.ndarray] = None
        if depth_msg is not None:
            try:
                depth_img = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(f"cv_bridge depth decode failed: {exc}")
                depth_img = None

        detections = []
        if result.boxes is not None and len(result.boxes) > 0:
            xyxy = result.boxes.xyxy.cpu().numpy()
            conf = result.boxes.conf.cpu().numpy()
            cls = result.boxes.cls.cpu().numpy().astype(int)
            for i in range(len(xyxy)):
                x1, y1, x2, y2 = [float(v) for v in xyxy[i]]
                cx_px = 0.5 * (x1 + x2)
                cy_px = 0.5 * (y1 + y2)
                det = {
                    "class_id": int(cls[i]),
                    "class_name": self._class_names.get(int(cls[i]), str(int(cls[i]))),
                    "confidence": float(conf[i]),
                    "bbox_xyxy": [x1, y1, x2, y2],
                    "bbox_center_px": [cx_px, cy_px],
                }
                point_3d = self._project_to_3d(cx_px, cy_px, depth_img)
                if point_3d is not None:
                    det["point_camera"] = point_3d
                detections.append(det)

        payload = {
            "header": {
                "stamp_sec": int(color_msg.header.stamp.sec),
                "stamp_nanosec": int(color_msg.header.stamp.nanosec),
                "frame_id": color_msg.header.frame_id,
            },
            "model": self._model_name,
            "detections": detections,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._pub_detections.publish(msg)

        if self._publish_annotated:
            annotated = result.plot()
            try:
                out = self._bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
                out.header = color_msg.header
                self._pub_annotated.publish(out)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(f"annotated publish failed: {exc}")

    def _project_to_3d(
        self, u: float, v: float, depth_img: Optional[np.ndarray]
    ) -> Optional[List[float]]:
        if depth_img is None or self._fx is None:
            return None
        h, w = depth_img.shape[:2]
        ui, vi = int(round(u)), int(round(v))
        if not (0 <= ui < w and 0 <= vi < h):
            return None
        # RealSense aligned depth is uint16 millimeters by default
        raw = depth_img[vi, ui]
        if depth_img.dtype == np.uint16:
            z = float(raw) * 1e-3
        else:
            z = float(raw)
        if not np.isfinite(z) or z <= 0.0:
            return None
        x = (u - self._cx) * z / self._fx
        y = (v - self._cy) * z / self._fy
        return [x, y, z]


def main() -> None:
    rclpy.init()
    node = Yolov8Node()

    def _graceful(_signum, _frame):
        # ros2 run sends SIGTERM on shutdown; convert it to a clean rclpy stop.
        if rclpy.ok():
            rclpy.shutdown()

    signal.signal(signal.SIGTERM, _graceful)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
