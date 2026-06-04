#!/usr/bin/env python3
"""인지(Perception) 단계: 2D YOLOv8 검출 -> 3D box 좌표(토픽) + RViz 마커.

yolo_remote_node(브리지)가 매 검출마다 발행하는 2D 검출 JSON
(``/yolov8_node/detections``)을 구독해, aligned depth + camera_info +
TF(camera->base, 내부 계산용)로 각 박스의 윗면 중심 3D 좌표(base frame)를 구해

  * ``/detected_boxes``         geometry_msgs/PoseArray   (base frame, 모션이 소비)
  * ``/detected_boxes_markers`` visualization_msgs/MarkerArray (RViz 시각화)

로 발행한다. **box TF 를 브로드캐스트하지 않는다** — 검출 객체 목록은 TF 프레임이
아니라 좌표 토픽 + 마커로 표현하는 것이 정석이며, 모션은 PoseArray 만 구독하면
TF lookup 없이 움직일 수 있다(인지/모션 분리).

워크스페이스(ws_x/ws_y_abs/ws_z, base frame) 밖이거나 depth 가 빈약한 검출은
버려 이미지 가장자리 슬리버/오검출을 거른다. 마커는 클래스(색)별로 색칠하고
클래스명 텍스트를 함께 표시한다.
"""
import json
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import ColorRGBA, String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

# mini-box 클래스(색) -> RGBA 마커 색.
CLASS_RGBA = {
    "mini-box-blue": (0.1, 0.3, 1.0),
    "mini-box-green": (0.1, 0.8, 0.2),
    "mini-box-orange": (1.0, 0.55, 0.0),
    "mini-box-red": (1.0, 0.1, 0.1),
    "mini-box-yellow": (1.0, 0.9, 0.0),
}


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class BoxPerceptionNode(Node):
    def __init__(self):
        super().__init__("box_perception_node")
        self.declare_parameter("base_frame", "openarmx_body_link0")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("detections_topic", "/yolov8_node/detections")
        self.declare_parameter("depth_topic",
                               "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic",
                               "/camera/camera/color/camera_info")
        self.declare_parameter("min_confidence", 0.0)   # 0 -> 검출측 conf 신뢰
        self.declare_parameter("marker_size", 0.04)     # 마커 큐브 한 변(m)
        self.declare_parameter("ws_x", [0.05, 0.70])
        self.declare_parameter("ws_y_abs", 0.45)
        self.declare_parameter("ws_z", [0.10, 0.32])
        gp = self.get_parameter
        self.base = str(gp("base_frame").value)
        self.cam = str(gp("camera_frame").value)
        self.min_conf = float(gp("min_confidence").value)
        self.msize = float(gp("marker_size").value)
        self.ws_x = [float(v) for v in gp("ws_x").value]
        self.ws_y = float(gp("ws_y_abs").value)
        self.ws_z = [float(v) for v in gp("ws_z").value]

        self.bridge = CvBridge()
        self.depth = None
        self.fx = self.fy = self.cx = self.cy = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # latched(transient_local): /detected_boxes 는 검출 1회당 1개 발행이라,
        # 늦게 구독한 모션 백엔드도 최신 1개를 받도록 한다.
        self.pub_poses = self.create_publisher(
            PoseArray, "/detected_boxes",
            QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.pub_markers = self.create_publisher(MarkerArray, "/detected_boxes_markers", 10)

        self.create_subscription(Image, str(gp("depth_topic").value),
                                 self._on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, str(gp("camera_info_topic").value),
                                 self._on_ci, 10)
        self.create_subscription(String, str(gp("detections_topic").value),
                                 self._on_detections, 10)
        self.get_logger().info(
            f"box_perception_node ready: {gp('detections_topic').value} -> "
            f"/detected_boxes (PoseArray, base={self.base}) + /detected_boxes_markers")

    # ---- sensors ----
    def _on_depth(self, m):
        self.depth = self.bridge.imgmsg_to_cv2(m, "passthrough")

    def _on_ci(self, m):
        if self.fx is None:
            k = m.k
            self.fx, self.fy, self.cx, self.cy = k[0], k[4], k[2], k[5]

    def _tf(self, target, source, timeout=0.5):
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < timeout:
            try:
                return self.tf_buffer.lookup_transform(target, source, Time()).transform
            except Exception:
                time.sleep(0.02)
        return None

    # ---- 2D bbox -> 3D (camera frame, 윗면 중심) ----
    def _point3d(self, bbox):
        if self.depth is None or self.fx is None:
            return None
        x1, y1, x2, y2 = [int(round(v)) for v in bbox]
        H, W = self.depth.shape[:2]
        x1, y1, x2, y2 = max(0, x1), max(0, y1), min(W, x2), min(H, y2)
        if x2 <= x1 or y2 <= y1:
            return None
        roi = self.depth[y1:y2, x1:x2].astype(float) / 1000.0
        vv, uu = np.where(roi > 0)
        if uu.size < 20:
            return None
        zz = roi[vv, uu]
        z_near = np.percentile(zz, 5)
        top = zz < (z_near + 0.03)
        if top.sum() < 10:
            top = zz <= np.percentile(zz, 30)
        u, v, z = uu[top] + x1, vv[top] + y1, zz[top]
        X = (u - self.cx) * z / self.fx
        Y = (v - self.cy) * z / self.fy
        return [float(np.mean(X)), float(np.mean(Y)), float(np.mean(z))]

    def _cam_to_base(self, p):
        tf = self._tf(self.base, self.cam)
        if tf is None:
            return None
        q, t = tf.rotation, tf.translation
        R = quat_to_R([q.x, q.y, q.z, q.w])
        return (R @ np.array(p) + np.array([t.x, t.y, t.z])).tolist()

    # ---- detections -> poses + markers ----
    def _on_detections(self, msg):
        try:
            dets = json.loads(msg.data).get("detections", [])
        except Exception:
            return
        boxes = []
        for d in dets:
            if float(d.get("confidence", 0.0)) < self.min_conf:
                continue
            bbox = d.get("bbox_xyxy")
            if not bbox:
                continue
            p = self._point3d(bbox)
            if p is None:
                continue
            b = self._cam_to_base(p)
            if b is None:
                continue
            if (self.ws_x[0] < b[0] < self.ws_x[1] and abs(b[1]) < self.ws_y
                    and self.ws_z[0] < b[2] < self.ws_z[1]):
                boxes.append({"base": b, "cls": d.get("class_name", ""),
                              "conf": float(d.get("confidence", 0.0))})
        boxes.sort(key=lambda d: -d["base"][1])      # +Y(좌) 우선
        self._publish(boxes)

    def _publish(self, boxes):
        now = self.get_clock().now().to_msg()

        pa = PoseArray()
        pa.header.stamp = now
        pa.header.frame_id = self.base
        for b in boxes:
            pose = Pose()
            (pose.position.x, pose.position.y, pose.position.z) = map(float, b["base"])
            pose.orientation.w = 1.0
            pa.poses.append(pose)
        self.pub_poses.publish(pa)

        ma = MarkerArray()
        clear = Marker()
        clear.header.frame_id = self.base
        clear.action = Marker.DELETEALL          # 이전 마커 전부 제거(가변 개수)
        ma.markers.append(clear)
        for i, b in enumerate(boxes):
            r, g, bl = CLASS_RGBA.get(b["cls"], (0.7, 0.7, 0.7))
            cube = Marker()
            cube.header.frame_id = self.base
            cube.header.stamp = now
            cube.ns = "boxes"
            cube.id = i
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            (cube.pose.position.x, cube.pose.position.y,
             cube.pose.position.z) = map(float, b["base"])
            cube.pose.orientation.w = 1.0
            cube.scale.x = cube.scale.y = cube.scale.z = self.msize
            cube.color = ColorRGBA(r=r, g=g, b=bl, a=0.85)
            ma.markers.append(cube)
            txt = Marker()
            txt.header.frame_id = self.base
            txt.header.stamp = now
            txt.ns = "labels"
            txt.id = i
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            (txt.pose.position.x, txt.pose.position.y) = float(b["base"][0]), float(b["base"][1])
            txt.pose.position.z = float(b["base"][2]) + 0.05
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.03
            txt.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.95)
            txt.text = f"{b['cls']} {b['conf']:.2f}"
            ma.markers.append(txt)
        self.pub_markers.publish(ma)
        self.get_logger().info(f"detected {len(boxes)} box(es) -> /detected_boxes + markers")


def main():
    rclpy.init()
    node = BoxPerceptionNode()
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
