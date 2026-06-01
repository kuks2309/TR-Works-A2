"""YOLOv8 (Ultralytics) on-demand action server for RealSense D435 color stream.

Exposes a `DetectBox` action: inference runs ONCE per goal on the latest
buffered camera frame instead of continuously on every frame. On a goal the node
decodes the cached frame, runs Ultralytics YOLOv8, publishes the annotated image
and JSON-encoded detection list on the usual topics (so downstream box_plane_node
still reacts), and returns a summary in the action result. Optional depth +
camera_info subscription lets it back-project each detection center into a 3D
point in the camera optical frame.

The camera subscriptions only cache the latest message reference (no decode, no
inference), so the node sits idle (~0 CPU, model resident in RAM) until a goal
arrives.
"""

from __future__ import annotations

import json
import signal
import threading
from typing import List, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header, String
from ultralytics import YOLO

from yolov8_detection_msgs.action import DetectBox


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

        # Latest-frame caches: callbacks only store the most recent message
        # reference (no decode, no inference), so the node stays idle until a
        # DetectBox goal arrives. Inference happens only inside _execute.
        self._last_color: Optional[Image] = None
        self._last_depth: Optional[Image] = None

        self.create_subscription(Image, self._image_topic, self._on_color, sensor_qos)
        self.get_logger().info(f"Buffering color: {self._image_topic}")
        if self._use_depth:
            self.create_subscription(CameraInfo, self._info_topic, self._on_camera_info, 10)
            self.create_subscription(Image, self._depth_topic, self._on_depth, sensor_qos)
            self.get_logger().info(f"Buffering depth: {self._depth_topic}")

        # On-demand action server (busy-flag pattern: reject overlapping goals).
        self._goal_lock = threading.Lock()
        self._active = False
        action_cb = ReentrantCallbackGroup()
        self._action_srv = ActionServer(
            self,
            DetectBox,
            "~/detect",
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=action_cb,
        )

        self.get_logger().info(
            f"DetectBox action ready at ~/detect (idle; conf>={self._conf}, "
            f"iou={self._iou}, imgsz={self._imgsz})"
        )

    def _on_camera_info(self, msg: CameraInfo) -> None:
        if self._fx is None:
            k = msg.k
            self._fx, self._fy = float(k[0]), float(k[4])
            self._cx, self._cy = float(k[2]), float(k[5])
            self.get_logger().info(
                f"CameraInfo locked: fx={self._fx:.2f} fy={self._fy:.2f} cx={self._cx:.2f} cy={self._cy:.2f}"
            )

    def _on_color(self, msg: Image) -> None:
        # Cheap: store the latest reference only (no decode / no inference).
        self._last_color = msg

    def _on_depth(self, msg: Image) -> None:
        self._last_depth = msg

    def _publish_detections(self, payload: dict) -> None:
        msg = String()
        msg.data = json.dumps(payload)
        self._pub_detections.publish(msg)

    def _process(
        self,
        color_msg: Image,
        depth_msg: Optional[Image],
        conf: Optional[float] = None,
        annotate: Optional[bool] = None,
    ) -> dict:
        """Run inference on a single frame, publish on the usual topics, and
        return the detection payload (the dict published on ~/detections)."""
        conf_val = float(conf) if (conf is not None and conf > 0.0) else self._conf
        header = {
            "stamp_sec": int(color_msg.header.stamp.sec),
            "stamp_nanosec": int(color_msg.header.stamp.nanosec),
            "frame_id": color_msg.header.frame_id,
        }

        try:
            cv_bgr = self._bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"cv_bridge color decode failed: {exc}")
            payload = {"header": header, "model": self._model_name, "detections": []}
            self._publish_detections(payload)
            return payload

        results = self._yolo.predict(
            source=cv_bgr,
            conf=conf_val,
            iou=self._iou,
            imgsz=self._imgsz,
            classes=self._classes,
            device=self._device,
            verbose=False,
        )
        if not results:
            payload = {"header": header, "model": self._model_name, "detections": []}
            self._publish_detections(payload)
            return payload
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
            conf_arr = result.boxes.conf.cpu().numpy()
            cls = result.boxes.cls.cpu().numpy().astype(int)
            for i in range(len(xyxy)):
                x1, y1, x2, y2 = [float(v) for v in xyxy[i]]
                cx_px = 0.5 * (x1 + x2)
                cy_px = 0.5 * (y1 + y2)
                det = {
                    "class_id": int(cls[i]),
                    "class_name": self._class_names.get(int(cls[i]), str(int(cls[i]))),
                    "confidence": float(conf_arr[i]),
                    "bbox_xyxy": [x1, y1, x2, y2],
                    "bbox_center_px": [cx_px, cy_px],
                }
                point_3d = self._project_to_3d(cx_px, cy_px, depth_img)
                if point_3d is not None:
                    det["point_camera"] = point_3d
                detections.append(det)

        payload = {"header": header, "model": self._model_name, "detections": detections}
        self._publish_detections(payload)

        if self._publish_annotated or bool(annotate):
            annotated = result.plot()
            try:
                out = self._bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
                out.header = color_msg.header
                self._pub_annotated.publish(out)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(f"annotated publish failed: {exc}")

        return payload

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

    # ------------------------------------------------------------------ action

    def _on_goal(self, _goal_request) -> GoalResponse:
        with self._goal_lock:
            if self._active:
                self.get_logger().warning("DetectBox goal rejected: server busy")
                return GoalResponse.REJECT
            self._active = True
            return GoalResponse.ACCEPT

    def _on_cancel(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _publish_feedback(self, goal_handle, phase: str, progress: float) -> None:
        fb = DetectBox.Feedback()
        fb.phase = phase
        fb.progress = float(progress)
        goal_handle.publish_feedback(fb)

    def _execute(self, goal_handle):
        result = DetectBox.Result()
        result.success = False
        result.message = ""
        result.num_detections = 0
        result.detections_json = ""
        try:
            goal = goal_handle.request
            self._publish_feedback(goal_handle, "grabbing_frame", 0.1)

            color = self._last_color
            depth = self._last_depth if self._use_depth else None
            if color is None:
                goal_handle.abort()
                result.message = "no color frame buffered yet"
                return result

            if depth is not None:
                skew = abs(
                    (color.header.stamp.sec - depth.header.stamp.sec)
                    + 1e-9 * (color.header.stamp.nanosec - depth.header.stamp.nanosec)
                )
                if skew > self._sync_slop:
                    self.get_logger().warning(
                        f"color/depth stamp skew {skew:.3f}s > sync_slop "
                        f"{self._sync_slop:.3f}s; proceeding"
                    )

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.message = "canceled"
                return result

            # Per-goal prompt override (YOLO-World models only).
            if goal.prompts and hasattr(self._yolo, "set_classes"):
                prompts = [p.strip() for p in goal.prompts.split(",") if p.strip()]
                if prompts:
                    self._yolo.set_classes(prompts)

            conf_override = goal.confidence if goal.confidence > 0.0 else None

            self._publish_feedback(goal_handle, "inferring", 0.5)
            payload = self._process(
                color, depth, conf=conf_override, annotate=goal.publish_annotated
            )

            self._publish_feedback(goal_handle, "publishing", 0.9)
            goal_handle.succeed()
            result.success = True
            result.message = "ok"
            result.num_detections = len(payload["detections"])
            result.detections_json = json.dumps(payload)
            return result
        finally:
            with self._goal_lock:
                self._active = False


def main() -> None:
    rclpy.init()
    node = Yolov8Node()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    def _graceful(_signum, _frame):
        # ros2 run sends SIGTERM on shutdown; convert it to a clean rclpy stop.
        if rclpy.ok():
            rclpy.shutdown()

    signal.signal(signal.SIGTERM, _graceful)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
