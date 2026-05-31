"""Fit the top plane of a YOLO-detected cardboard box.

Consumes:
  - /yolov8_node/detections (std_msgs/String, JSON from yolov8_node)
  - /camera/camera/aligned_depth_to_color/image_raw (sensor_msgs/Image, uint16 mm)
  - /camera/camera/color/camera_info (sensor_msgs/CameraInfo)

For each detection whose class_name matches one of the configured keywords
(default: "box", "cardboard box", "carton", "package"), the node crops depth
to the 2D bbox, back-projects valid pixels into the color optical frame,
keeps only the upper portion of the depth distribution (the box top surface),
and runs a plain numpy RANSAC to fit a plane ax+by+cz+d=0.

Publishes per call:
  - /box_plane/cloud      sensor_msgs/PointCloud2  (inlier points, colored)
  - /box_plane/markers    visualization_msgs/MarkerArray  (disk + normal arrow)
  - /box_plane/info       std_msgs/String  (JSON plane summary)
"""

from __future__ import annotations

import json
import math
import signal
import struct
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Vector3
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import ColorRGBA, Header, String
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros
from rclpy.duration import Duration


def _quat_to_rot(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    n = qx * qx + qy * qy + qz * qz + qw * qw
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = qx * qx * s, qy * qy * s, qz * qz * s
    xy, xz, yz = qx * qy * s, qx * qz * s, qy * qz * s
    wx, wy, wz = qw * qx * s, qw * qy * s, qw * qz * s
    return np.array([
        [1.0 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1.0 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1.0 - (xx + yy)],
    ])


def _ransac_plane(
    points: np.ndarray,
    threshold: float,
    iterations: int,
    rng: np.random.Generator,
) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    """Return (plane_coef[a,b,c,d], inlier_mask) or (None, None) on failure."""
    n = points.shape[0]
    if n < 50:
        return None, None
    best_count = 0
    best_mask: Optional[np.ndarray] = None
    best_coef: Optional[np.ndarray] = None
    for _ in range(iterations):
        idx = rng.choice(n, 3, replace=False)
        p0, p1, p2 = points[idx]
        v1 = p1 - p0
        v2 = p2 - p0
        normal = np.cross(v1, v2)
        nn = np.linalg.norm(normal)
        if nn < 1e-9:
            continue
        normal = normal / nn
        d = -float(normal @ p0)
        dist = np.abs(points @ normal + d)
        mask = dist < threshold
        count = int(mask.sum())
        if count > best_count:
            best_count = count
            best_mask = mask
            best_coef = np.array([normal[0], normal[1], normal[2], d])
    if best_mask is None or best_count < 50:
        return None, None
    # SVD refinement on inliers
    inliers = points[best_mask]
    centroid = inliers.mean(axis=0)
    centered = inliers - centroid
    _u, _s, vh = np.linalg.svd(centered, full_matrices=False)
    normal = vh[-1]
    d = -float(normal @ centroid)
    if normal[2] > 0:  # orient toward camera (camera looks +z)
        normal = -normal
        d = -d
    refined = np.array([normal[0], normal[1], normal[2], d])
    return refined, best_mask


def _pack_pointcloud2_xyzrgb(
    points: np.ndarray,
    rgb: Tuple[int, int, int],
    frame_id: str,
    stamp,
) -> PointCloud2:
    n = points.shape[0]
    r, g, b = rgb
    rgb_uint32 = (r << 16) | (g << 8) | b
    rgb_float = struct.unpack("f", struct.pack("I", rgb_uint32))[0]
    arr = np.zeros(n, dtype=[("x", np.float32), ("y", np.float32),
                              ("z", np.float32), ("rgb", np.float32)])
    arr["x"] = points[:, 0].astype(np.float32)
    arr["y"] = points[:, 1].astype(np.float32)
    arr["z"] = points[:, 2].astype(np.float32)
    arr["rgb"].fill(rgb_float)
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.height = 1
    msg.width = n
    msg.is_dense = True
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * n
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.data = arr.tobytes()
    return msg


def _disk_marker(
    centroid: np.ndarray,
    normal: np.ndarray,
    frame_id: str,
    stamp,
    marker_id: int,
    color: Tuple[float, float, float, float],
    radius: float,
) -> Marker:
    # Build a rotation that maps +z to `normal`.
    n = normal / max(np.linalg.norm(normal), 1e-9)
    z = np.array([0.0, 0.0, 1.0])
    v = np.cross(z, n)
    s = float(np.linalg.norm(v))
    c = float(z @ n)
    if s < 1e-9:
        qx = qy = qz = 0.0
        qw = 1.0 if c > 0 else 0.0
        if c < 0:
            qx = 1.0
            qw = 0.0
    else:
        axis = v / s
        angle = math.atan2(s, c)
        sa = math.sin(angle / 2.0)
        qx, qy, qz = axis * sa
        qw = math.cos(angle / 2.0)
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = stamp
    m.ns = "box_top_plane"
    m.id = marker_id
    m.type = Marker.CYLINDER
    m.action = Marker.ADD
    m.pose.position = Point(x=float(centroid[0]), y=float(centroid[1]), z=float(centroid[2]))
    m.pose.orientation.x = qx
    m.pose.orientation.y = qy
    m.pose.orientation.z = qz
    m.pose.orientation.w = qw
    m.scale.x = 2.0 * radius
    m.scale.y = 2.0 * radius
    m.scale.z = 0.003
    m.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
    return m


def _arrow_marker(
    centroid: np.ndarray,
    normal: np.ndarray,
    frame_id: str,
    stamp,
    marker_id: int,
    color: Tuple[float, float, float, float],
    length: float = 0.15,
) -> Marker:
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = stamp
    m.ns = "box_top_normal"
    m.id = marker_id
    m.type = Marker.ARROW
    m.action = Marker.ADD
    tip = centroid + normal * length
    m.points = [
        Point(x=float(centroid[0]), y=float(centroid[1]), z=float(centroid[2])),
        Point(x=float(tip[0]), y=float(tip[1]), z=float(tip[2])),
    ]
    m.scale = Vector3(x=0.008, y=0.018, z=0.0)
    m.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
    return m


class BoxPlaneNode(Node):
    def __init__(self) -> None:
        super().__init__("box_plane_node")

        self.declare_parameter("detections_topic", "/yolov8_node/detections")
        self.declare_parameter("depth_topic",
                               "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic",
                               "/camera/camera/color/camera_info")
        self.declare_parameter(
            "box_keywords",
            "box,cardboard box,carton,package",
        )
        self.declare_parameter("min_confidence", 0.20)
        self.declare_parameter("ransac_threshold", 0.006)  # m
        self.declare_parameter("ransac_iterations", 400)
        self.declare_parameter("depth_min", 0.20)  # m, ignore closer
        self.declare_parameter("depth_max", 2.50)  # m, ignore farther
        self.declare_parameter("top_depth_quantile", 0.35)
        self.declare_parameter("subsample_max", 4000)
        # Ground-plane fitting params
        self.declare_parameter("fit_ground", True)
        self.declare_parameter("ground_pixel_step", 6)
        self.declare_parameter("ground_ransac_threshold", 0.015)  # looser, floor is noisy
        self.declare_parameter("ground_ransac_iterations", 400)
        self.declare_parameter("ground_subsample_max", 10000)
        self.declare_parameter("ground_max_normal_deviation_deg", 20.0)
        self.declare_parameter("ground_min_inliers", 200)
        # Body frame whose +z axis is "up" in the world. The static TF
        # body_frame -> camera_link is required (provided by show_robot:=true
        # in the launch). Without it the node refuses to fit planes because
        # there is no shared notion of "up".
        self.declare_parameter("body_frame", "openarmx_body_link0")

        self._detections_topic = self.get_parameter("detections_topic").value
        self._depth_topic = self.get_parameter("depth_topic").value
        self._info_topic = self.get_parameter("camera_info_topic").value
        kw = str(self.get_parameter("box_keywords").value or "")
        self._keywords = {k.strip().lower() for k in kw.split(",") if k.strip()}
        self._min_conf = float(self.get_parameter("min_confidence").value)
        self._ransac_thr = float(self.get_parameter("ransac_threshold").value)
        self._ransac_iter = int(self.get_parameter("ransac_iterations").value)
        self._depth_min = float(self.get_parameter("depth_min").value)
        self._depth_max = float(self.get_parameter("depth_max").value)
        self._top_q = float(self.get_parameter("top_depth_quantile").value)
        self._subsample_max = int(self.get_parameter("subsample_max").value)
        self._fit_ground = bool(self.get_parameter("fit_ground").value)
        self._g_step = int(self.get_parameter("ground_pixel_step").value)
        self._g_thr = float(self.get_parameter("ground_ransac_threshold").value)
        self._g_iter = int(self.get_parameter("ground_ransac_iterations").value)
        self._g_subsample = int(self.get_parameter("ground_subsample_max").value)
        self._g_max_dev = math.radians(float(self.get_parameter("ground_max_normal_deviation_deg").value))
        self._g_min_inliers = int(self.get_parameter("ground_min_inliers").value)
        self._body_frame = str(self.get_parameter("body_frame").value)
        self._tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        # Cached transforms: rotation R such that body_pt = R @ cam_pt + t
        self._R_body_cam: Optional[np.ndarray] = None
        self._t_body_cam: Optional[np.ndarray] = None
        self._body_up_in_cam: Optional[np.ndarray] = None  # body +z expressed in camera frame

        self._bridge = CvBridge()
        self._rng = np.random.default_rng(42)
        self._depth: Optional[np.ndarray] = None
        self._depth_stamp = None
        self._depth_frame: Optional[str] = None
        self._fx = self._fy = self._cx = self._cy = None

        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value
        self.create_subscription(Image, self._depth_topic, self._on_depth, sensor_qos)
        self.create_subscription(CameraInfo, self._info_topic, self._on_info, 10)
        self.create_subscription(String, self._detections_topic, self._on_detections, 10)

        self._pub_cloud = self.create_publisher(PointCloud2, "/box_plane/cloud", 5)
        self._pub_markers = self.create_publisher(MarkerArray, "/box_plane/markers", 5)
        self._pub_info = self.create_publisher(String, "/box_plane/info", 10)
        self._pub_ground_cloud = self.create_publisher(PointCloud2, "/ground_plane/cloud", 5)

        self.get_logger().info(
            f"Listening to {self._detections_topic} + {self._depth_topic}; "
            f"keywords={sorted(self._keywords)}, min_conf={self._min_conf}"
        )

    def _on_info(self, msg: CameraInfo) -> None:
        if self._fx is None:
            k = msg.k
            self._fx, self._fy = float(k[0]), float(k[4])
            self._cx, self._cy = float(k[2]), float(k[5])
            self.get_logger().info(
                f"CameraInfo locked: fx={self._fx:.2f} fy={self._fy:.2f} "
                f"cx={self._cx:.2f} cy={self._cy:.2f}"
            )

    def _on_depth(self, msg: Image) -> None:
        try:
            depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"depth decode failed: {exc}")
            return
        self._depth = depth
        self._depth_stamp = msg.header.stamp
        self._depth_frame = msg.header.frame_id

    def _ensure_body_tf(self) -> bool:
        """Look up body_frame -> depth_frame and cache the rotation/translation.

        Returns True if the transform is available, False otherwise.
        """
        if self._body_up_in_cam is not None:
            return True
        if not self._depth_frame:
            return False
        try:
            ts = self._tf_buffer.lookup_transform(
                self._depth_frame,           # target frame (cam optical)
                self._body_frame,            # source frame (body)
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return False
        q = ts.transform.rotation
        t = ts.transform.translation
        # R takes a vector expressed in body frame to camera frame:
        #   v_cam = R @ v_body
        R = _quat_to_rot(q.x, q.y, q.z, q.w)
        T = np.array([t.x, t.y, t.z])
        self._R_body_cam = R.T              # cam -> body rotation
        self._t_body_cam = -R.T @ T         # cam -> body translation
        self._body_up_in_cam = R[:, 2]       # body +z direction expressed in cam frame
        self.get_logger().info(
            f"TF {self._body_frame} -> {self._depth_frame} locked: "
            f"body_up_in_cam=({self._body_up_in_cam[0]:+.3f},"
            f"{self._body_up_in_cam[1]:+.3f},{self._body_up_in_cam[2]:+.3f})"
        )
        return True

    def _to_body(self, pts_cam: np.ndarray) -> np.ndarray:
        return pts_cam @ self._R_body_cam.T + self._t_body_cam

    def _on_detections(self, msg: String) -> None:
        if self._depth is None or self._fx is None:
            return
        if not self._ensure_body_tf():
            self.get_logger().warning(
                f"Waiting for TF {self._body_frame} -> {self._depth_frame}; "
                "skipping this detection. (Did you launch with show_robot:=true?)"
            )
            return
        try:
            data = json.loads(msg.data)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"detections JSON parse failed: {exc}")
            return

        markers = MarkerArray()
        info_payload = {
            "stamp_sec": int(self._depth_stamp.sec) if self._depth_stamp else 0,
            "stamp_nanosec": int(self._depth_stamp.nanosec) if self._depth_stamp else 0,
            "frame_id": self._depth_frame or "",
            "planes": [],
            "ground": None,
        }

        eligible = [
            d for d in data.get("detections", [])
            if d.get("class_name", "").lower() in self._keywords
            and float(d.get("confidence", 0.0)) >= self._min_conf
        ]
        if not eligible:
            return

        # Sort largest bbox first; take top 3 to keep this cheap.
        def _area(d):
            x1, y1, x2, y2 = d["bbox_xyxy"]
            return max(0.0, x2 - x1) * max(0.0, y2 - y1)
        eligible.sort(key=_area, reverse=True)

        marker_id = 0
        for det in eligible[:3]:
            plane = self._fit_box_top(det)
            if plane is None:
                continue
            coef, centroid, inliers, area = plane
            color = (1.0, 0.2, 0.2, 0.6)
            radius = 0.5 * math.sqrt(max(area, 1e-4)) * 1.3
            markers.markers.append(
                _disk_marker(centroid, coef[:3], self._depth_frame,
                             self._depth_stamp, marker_id, color, radius)
            )
            marker_id += 1
            markers.markers.append(
                _arrow_marker(centroid, coef[:3], self._depth_frame,
                              self._depth_stamp, marker_id, (1.0, 0.8, 0.1, 0.9))
            )
            marker_id += 1

            cloud_msg = _pack_pointcloud2_xyzrgb(
                inliers, (240, 60, 60), self._depth_frame, self._depth_stamp
            )
            self._pub_cloud.publish(cloud_msg)

            centroid_body = self._to_body(centroid[None, :])[0]
            normal_body = self._R_body_cam @ coef[:3]
            info_payload["planes"].append({
                "class_name": det["class_name"],
                "bbox_xyxy": det["bbox_xyxy"],
                "plane_coef_abcd": [float(x) for x in coef],
                "normal_unit": [float(x) for x in coef[:3]],
                "centroid": [float(x) for x in centroid],
                "centroid_body": [float(x) for x in centroid_body],
                "normal_body": [float(x) for x in normal_body],
                "inlier_count": int(inliers.shape[0]),
                "approx_area_m2": float(area),
            })

        if self._fit_ground and info_payload["planes"]:
            box_bboxes = [tuple(int(round(v)) for v in p["bbox_xyxy"])
                          for p in info_payload["planes"]]
            # The reference normal is body +z expressed in the camera frame —
            # both the floor and the box top are horizontal in the body frame.
            ref_normal = self._body_up_in_cam
            box_top_body_z = float(info_payload["planes"][0]["centroid_body"][2])
            ground = self._fit_ground_plane(box_bboxes, ref_normal, box_top_body_z)
            if ground is not None:
                g_coef, g_centroid, g_inliers = ground
                color = (0.2, 0.9, 0.2, 0.55)
                radius = 0.30
                markers.markers.append(
                    _disk_marker(g_centroid, g_coef[:3], self._depth_frame,
                                 self._depth_stamp, marker_id, color, radius)
                )
                marker_id += 1
                markers.markers.append(
                    _arrow_marker(g_centroid, g_coef[:3], self._depth_frame,
                                  self._depth_stamp, marker_id,
                                  (0.2, 0.9, 0.2, 0.9))
                )
                marker_id += 1
                self._pub_ground_cloud.publish(_pack_pointcloud2_xyzrgb(
                    g_inliers, (60, 230, 60), self._depth_frame, self._depth_stamp
                ))
                g_centroid_body = self._to_body(g_centroid[None, :])[0]
                g_normal_body = self._R_body_cam @ g_coef[:3]
                # Box height = box-top body z minus ground body z.
                box_top_body = np.array(info_payload["planes"][0]["centroid_body"])
                box_height = float(box_top_body[2] - g_centroid_body[2])
                info_payload["ground"] = {
                    "plane_coef_abcd": [float(x) for x in g_coef],
                    "normal_unit": [float(x) for x in g_coef[:3]],
                    "centroid": [float(x) for x in g_centroid],
                    "centroid_body": [float(x) for x in g_centroid_body],
                    "normal_body": [float(x) for x in g_normal_body],
                    "inlier_count": int(g_inliers.shape[0]),
                    "box_height_m": float(box_height),
                }

        if markers.markers:
            self._pub_markers.publish(markers)
        if info_payload["planes"]:
            out = String()
            out.data = json.dumps(info_payload)
            self._pub_info.publish(out)
            g = info_payload.get("ground")
            g_str = ""
            if g:
                g_str = (f" | ground@body_z={g['centroid_body'][2]:.3f}m "
                         f"(inliers={g['inlier_count']}, "
                         f"h_box={g['box_height_m']*100:.1f}cm)")
            self.get_logger().info(
                "fit " + ", ".join(
                    f"{p['class_name']}@body_z={p['centroid_body'][2]:.3f}m "
                    f"inliers={p['inlier_count']}"
                    for p in info_payload["planes"]
                ) + g_str
            )

    def _fit_box_top(self, det: dict):
        if self._depth is None:
            return None
        x1, y1, x2, y2 = [int(round(v)) for v in det["bbox_xyxy"]]
        h, w = self._depth.shape[:2]
        x1 = max(0, min(w - 1, x1))
        x2 = max(0, min(w - 1, x2))
        y1 = max(0, min(h - 1, y1))
        y2 = max(0, min(h - 1, y2))
        if x2 <= x1 or y2 <= y1:
            return None
        roi = self._depth[y1:y2, x1:x2]
        if self._depth.dtype == np.uint16:
            z = roi.astype(np.float32) * 1e-3
        else:
            z = roi.astype(np.float32)
        valid = np.isfinite(z) & (z >= self._depth_min) & (z <= self._depth_max)
        if int(valid.sum()) < 200:
            return None

        ys, xs = np.where(valid)
        zs = z[valid]
        ys = ys + y1
        xs = xs + x1
        Xc = (xs - self._cx) * zs / self._fx
        Yc = (ys - self._cy) * zs / self._fy
        pts_cam_full = np.stack([Xc, Yc, zs], axis=1).astype(np.float64)
        # Keep only the upper slab of the box (largest body-z values) — this
        # is the box top. Selecting by camera depth would catch the front
        # face instead, which is vertical in the body frame.
        body_z = self._to_body(pts_cam_full)[:, 2]
        cutoff = np.quantile(body_z, 1.0 - self._top_q)
        keep = body_z >= cutoff
        if int(keep.sum()) < 100:
            return None
        pts = pts_cam_full[keep]
        if pts.shape[0] > self._subsample_max:
            idx = self._rng.choice(pts.shape[0], self._subsample_max, replace=False)
            pts = pts[idx]

        coef, mask = _ransac_plane(pts, self._ransac_thr,
                                   self._ransac_iter, self._rng)
        if coef is None or mask is None:
            return None
        # Orient normal so it points along body +z (the world "up"), not down
        # into the box. Without this the camera-tilt sign flip in _ransac_plane
        # can leave the normal pointing into the surface.
        if self._body_up_in_cam is not None and float(coef[:3] @ self._body_up_in_cam) < 0.0:
            coef = -coef
        inliers = pts[mask]
        centroid = inliers.mean(axis=0)
        # Rough area from inlier 2D extents projected onto plane (bbox proxy).
        spread = inliers.max(axis=0) - inliers.min(axis=0)
        area = float(spread[0] * spread[1] + 1e-6)
        return coef, centroid, inliers, area

    def _fit_ground_plane(
        self,
        box_bboxes: List[Tuple[int, int, int, int]],
        ref_normal: np.ndarray,
        box_top_body_z: float,
    ) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray]]:
        """Fit the floor plane: horizontal in the body frame, below the box top.

        Candidates must (a) have a normal aligned with body +z (expressed in
        the camera frame as ``ref_normal``) within the configured tolerance,
        and (b) have their centroid below the box top in body z. Among the
        qualifying peel-off iterations we pick the one with the most inliers.
        """
        if self._depth is None or self._fx is None:
            return None
        h, w = self._depth.shape[:2]
        ys, xs = np.mgrid[0:h:self._g_step, 0:w:self._g_step].reshape(2, -1)
        if self._depth.dtype == np.uint16:
            zs = self._depth[ys, xs].astype(np.float32) * 1e-3
        else:
            zs = self._depth[ys, xs].astype(np.float32)
        valid = np.isfinite(zs) & (zs >= self._depth_min) & (zs <= self._depth_max)
        for (bx1, by1, bx2, by2) in box_bboxes:
            in_box = (xs >= bx1) & (xs < bx2) & (ys >= by1) & (ys < by2)
            valid &= ~in_box
        if int(valid.sum()) < 500:
            return None
        ys, xs, zs = ys[valid], xs[valid], zs[valid]
        Xc = (xs - self._cx) * zs / self._fx
        Yc = (ys - self._cy) * zs / self._fy
        pts = np.stack([Xc, Yc, zs], axis=1).astype(np.float64)
        if pts.shape[0] > self._g_subsample:
            idx = self._rng.choice(pts.shape[0], self._g_subsample, replace=False)
            pts = pts[idx]

        ref = ref_normal / max(np.linalg.norm(ref_normal), 1e-9)
        cos_lim = math.cos(self._g_max_dev)
        candidates = []
        remaining = pts
        for _ in range(6):
            if remaining.shape[0] < 300:
                break
            coef, mask = _ransac_plane(remaining, self._g_thr,
                                       self._g_iter, self._rng)
            if coef is None or mask is None:
                break
            n = coef[:3]
            inliers = remaining[mask]
            centroid_cam = inliers.mean(axis=0)
            centroid_body_z = float(self._to_body(centroid_cam[None, :])[0, 2])
            parallel = abs(float(n @ ref)) >= cos_lim
            below_box = centroid_body_z < box_top_body_z - 0.02  # ≥ 2 cm below
            if parallel and below_box and inliers.shape[0] >= self._g_min_inliers:
                if float(coef[:3] @ ref) < 0.0:
                    coef = -coef  # orient ground normal toward body +z
                candidates.append((inliers.shape[0], coef, centroid_cam, inliers))
            remaining = remaining[~mask]
        if not candidates:
            return None
        candidates.sort(key=lambda c: c[0], reverse=True)
        _, coef, centroid, inliers = candidates[0]
        return coef, centroid, inliers


def main() -> None:
    rclpy.init()
    node = BoxPlaneNode()

    def _graceful(_signum, _frame):
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
