"""Remote YOLOv8 bridge: on-demand DetectBox action that offloads inference to a
non-ROS2 HTTP server (e.g. a Raspberry Pi 5 + Hailo-8 running pi_yolo_server).

This is a drop-in alternative to yolov8_node: it exposes the SAME `DetectBox`
action at `~/detect` and publishes the SAME JSON schema on `~/detections`
(header / model / detections[]), so box_plane_node and any downstream consumer
work unchanged.

Split of work:
  * Robot PC (this node): buffers the latest color (+ optional aligned depth and
    camera_info), encodes ONE color frame as JPEG per goal, POSTs it to the
    remote server, and receives 2D detections (class_id / class_name /
    confidence / bbox_xyxy). Depth NEVER leaves this PC; the 3D back-projection
    of each box center is done here from the local depth + intrinsics, exactly
    like yolov8_node.
  * Remote server (pi_yolo_server): runs the actual YOLOv8s.hef on the NPU and
    returns 2D boxes only.

Only the inference step differs from yolov8_node; everything else (action
server, busy-flag, depth projection, detection schema) is mirrored. No
Ultralytics / torch import here — the robot PC does not run the model.
"""

from __future__ import annotations

import json
import signal
import threading
import time
import urllib.error
import urllib.request
from typing import List, Optional
from urllib.parse import urlencode

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
from std_msgs.msg import String

from yolov8_detection_msgs.action import DetectBox


class YoloRemoteNode(Node):
    def __init__(self) -> None:
        super().__init__("yolo_remote_node")

        # --- remote server ---
        self.declare_parameter("server_url", "http://10.42.0.2:8080/detect")
        self.declare_parameter("request_timeout", 5.0)
        self.declare_parameter("jpeg_quality", 80)
        # Label reported in the published payload's "model" field.
        self.declare_parameter("model_label", "remote-hailo-yolov8s")

        # --- detection params (forwarded to the server as query args) ---
        self.declare_parameter("confidence", 0.35)
        self.declare_parameter("iou", 0.5)

        # --- camera I/O (same defaults as yolov8_node) ---
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("use_depth", False)
        self.declare_parameter("sync_slop", 0.05)
        self.declare_parameter("publish_annotated", True)

        self._server_url = str(self.get_parameter("server_url").value)
        self._timeout = float(self.get_parameter("request_timeout").value)
        self._jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self._model_label = str(self.get_parameter("model_label").value)
        self._conf = float(self.get_parameter("confidence").value)
        self._iou = float(self.get_parameter("iou").value)
        self._image_topic = self.get_parameter("image_topic").value
        self._depth_topic = self.get_parameter("depth_topic").value
        self._info_topic = self.get_parameter("camera_info_topic").value
        self._use_depth = bool(self.get_parameter("use_depth").value)
        self._sync_slop = float(self.get_parameter("sync_slop").value)
        self._publish_annotated = bool(self.get_parameter("publish_annotated").value)

        self._bridge = CvBridge()
        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value

        self._pub_annotated = self.create_publisher(Image, "~/image_annotated", 10)
        self._pub_detections = self.create_publisher(String, "~/detections", 10)

        self._fx = self._fy = self._cx = self._cy = None

        # Latest-frame caches (store reference only; no decode/inference here).
        self._last_color: Optional[Image] = None
        self._last_depth: Optional[Image] = None

        self.create_subscription(Image, self._image_topic, self._on_color, sensor_qos)
        self.get_logger().info(f"Buffering color: {self._image_topic}")
        if self._use_depth:
            self.create_subscription(CameraInfo, self._info_topic, self._on_camera_info, 10)
            self.create_subscription(Image, self._depth_topic, self._on_depth, sensor_qos)
            self.get_logger().info(f"Buffering depth: {self._depth_topic}")

        # On-demand action server (busy-flag: reject overlapping goals).
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
            f"Remote DetectBox ready at ~/detect -> {self._server_url} "
            f"(idle; conf>={self._conf}, iou={self._iou}, jpeg_q={self._jpeg_quality})"
        )

    # ------------------------------------------------------------ subscriptions

    def _on_camera_info(self, msg: CameraInfo) -> None:
        if self._fx is None:
            k = msg.k
            self._fx, self._fy = float(k[0]), float(k[4])
            self._cx, self._cy = float(k[2]), float(k[5])
            self.get_logger().info(
                f"CameraInfo locked: fx={self._fx:.2f} fy={self._fy:.2f} "
                f"cx={self._cx:.2f} cy={self._cy:.2f}"
            )

    def _on_color(self, msg: Image) -> None:
        self._last_color = msg

    def _on_depth(self, msg: Image) -> None:
        self._last_depth = msg

    # ------------------------------------------------------------------ helpers

    def _publish_detections(self, payload: dict) -> None:
        msg = String()
        msg.data = json.dumps(payload)
        self._pub_detections.publish(msg)

    def _request_remote(self, jpeg: bytes, conf: float) -> dict:
        """POST the JPEG frame to the remote server and return the parsed JSON.

        Raises urllib.error.URLError / ValueError on transport or decode failure.
        """
        query = urlencode({"conf": f"{conf:.4f}", "iou": f"{self._iou:.4f}"})
        url = f"{self._server_url}?{query}" if query else self._server_url
        req = urllib.request.Request(
            url, data=jpeg, method="POST",
            headers={"Content-Type": "image/jpeg", "Content-Length": str(len(jpeg))},
        )
        with urllib.request.urlopen(req, timeout=self._timeout) as resp:
            raw = resp.read()
        return json.loads(raw.decode("utf-8"))

    def _project_to_3d(
        self, u: float, v: float, depth_img: Optional[np.ndarray]
    ) -> Optional[List[float]]:
        if depth_img is None or self._fx is None:
            return None
        h, w = depth_img.shape[:2]
        ui, vi = int(round(u)), int(round(v))
        if not (0 <= ui < w and 0 <= vi < h):
            return None
        # RealSense aligned depth is uint16 millimeters by default.
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

    def _build_payload(
        self,
        color_msg: Image,
        depth_img: Optional[np.ndarray],
        remote_dets: list,
        class_filter: Optional[set],
    ) -> dict:
        """Convert the remote 2D detections into the canonical yolov8_node schema,
        adding bbox_center_px and (when depth is available) point_camera."""
        header = {
            "stamp_sec": int(color_msg.header.stamp.sec),
            "stamp_nanosec": int(color_msg.header.stamp.nanosec),
            "frame_id": color_msg.header.frame_id,
        }
        detections = []
        for d in remote_dets:
            try:
                x1, y1, x2, y2 = [float(v) for v in d["bbox_xyxy"]]
            except (KeyError, TypeError, ValueError):
                continue
            class_name = str(d.get("class_name", str(d.get("class_id", "?"))))
            if class_filter is not None and class_name.lower() not in class_filter:
                continue
            cx_px = 0.5 * (x1 + x2)
            cy_px = 0.5 * (y1 + y2)
            det = {
                "class_id": int(d.get("class_id", -1)),
                "class_name": class_name,
                "confidence": float(d.get("confidence", 0.0)),
                "bbox_xyxy": [x1, y1, x2, y2],
                "bbox_center_px": [cx_px, cy_px],
            }
            # Segmentation backend supplies the mask centre of mass; use it as the
            # 3D pick point when present, otherwise the bbox centre.
            centroid = d.get("centroid_px")
            if isinstance(centroid, (list, tuple)) and len(centroid) == 2:
                px, py = float(centroid[0]), float(centroid[1])
                det["centroid_px"] = [px, py]
            else:
                px, py = cx_px, cy_px
            if "mask_area_px" in d:
                det["mask_area_px"] = int(d["mask_area_px"])
            point_3d = self._project_to_3d(px, py, depth_img)
            if point_3d is not None:
                det["point_camera"] = point_3d
            detections.append(det)
        return {"header": header, "model": self._model_label, "detections": detections}

    def _publish_annotated_image(self, bgr: np.ndarray, header, detections: list) -> None:
        canvas = bgr.copy()
        for det in detections:
            x1, y1, x2, y2 = [int(round(v)) for v in det["bbox_xyxy"]]
            cv2.rectangle(canvas, (x1, y1), (x2, y2), (0, 200, 0), 2)
            label = f"{det['class_name']} {det['confidence']:.2f}"
            cv2.putText(canvas, label, (x1, max(0, y1 - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 0), 1, cv2.LINE_AA)
        try:
            out = self._bridge.cv2_to_imgmsg(canvas, encoding="bgr8")
            out.header = header
            self._pub_annotated.publish(out)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"annotated publish failed: {exc}")

    # ------------------------------------------------------------------- action

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

            # Decode color once: needed both to JPEG-encode for the server and to
            # draw the annotated preview locally.
            try:
                cv_bgr = self._bridge.imgmsg_to_cv2(color, desired_encoding="bgr8")
            except Exception as exc:  # noqa: BLE001
                goal_handle.abort()
                result.message = f"cv_bridge color decode failed: {exc}"
                self.get_logger().error(result.message)
                return result

            ok, buf = cv2.imencode(
                ".jpg", cv_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality]
            )
            if not ok:
                goal_handle.abort()
                result.message = "JPEG encode failed"
                self.get_logger().error(result.message)
                return result

            depth_img: Optional[np.ndarray] = None
            if depth is not None:
                try:
                    depth_img = self._bridge.imgmsg_to_cv2(depth, desired_encoding="passthrough")
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warning(f"cv_bridge depth decode failed: {exc}")
                    depth_img = None

            # Per-goal overrides. For a fixed-class YOLOv8s the "prompts" field is
            # repurposed as a class-name allow-list applied to the response.
            conf_val = goal.confidence if goal.confidence > 0.0 else self._conf
            class_filter = None
            if goal.prompts:
                names = [p.strip().lower() for p in goal.prompts.split(",") if p.strip()]
                class_filter = set(names) if names else None

            self._publish_feedback(goal_handle, "inferring", 0.5)
            t0 = time.monotonic()
            try:
                response = self._request_remote(bytes(buf), conf_val)
            except (urllib.error.URLError, OSError) as exc:
                goal_handle.abort()
                result.message = f"remote server unreachable ({self._server_url}): {exc}"
                self.get_logger().error(result.message)
                return result
            except (ValueError, json.JSONDecodeError) as exc:
                goal_handle.abort()
                result.message = f"bad response from remote server: {exc}"
                self.get_logger().error(result.message)
                return result
            rtt_ms = (time.monotonic() - t0) * 1e3

            remote_dets = response.get("detections", []) if isinstance(response, dict) else []
            payload = self._build_payload(color, depth_img, remote_dets, class_filter)
            self._publish_detections(payload)

            if self._publish_annotated or bool(goal.publish_annotated):
                self._publish_annotated_image(cv_bgr, color.header, payload["detections"])

            self._publish_feedback(goal_handle, "publishing", 0.9)
            goal_handle.succeed()
            result.success = True
            result.message = (
                f"ok (remote {rtt_ms:.0f}ms, infer "
                f"{float(response.get('infer_ms', 0.0)):.0f}ms)"
                if isinstance(response, dict) else "ok"
            )
            result.num_detections = len(payload["detections"])
            result.detections_json = json.dumps(payload)
            return result
        finally:
            with self._goal_lock:
                self._active = False


def main() -> None:
    rclpy.init()
    node = YoloRemoteNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    def _graceful(_signum, _frame):
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
