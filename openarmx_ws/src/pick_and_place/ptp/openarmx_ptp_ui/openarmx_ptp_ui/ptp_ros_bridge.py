"""ROS2 bridge for the ptp pick-and-place UI.

DRIVE layer (구동-ROS). Runs an rclpy node in a background thread (so the Qt
event loop is never blocked) and exposes Qt signals for AlignToBoxes feedback,
result, and errors. Modelled on
openarmx_scenario_ui/openarmx_scenario_ui/scenario_action_client.py, trimmed to
the single AlignToBoxes action the ptp backend provides.

Public API (called from the Qt main thread):
  - run(z, roll_deg, pitch_deg, yaw_deg, arms) -> bool   (send a goal)
  - cancel() -> None                                      (cancel active goal)
  - live_node_names() -> set                              (status detection)
  - shutdown() -> None

Qt signals (received by main_window):
  - sig_feedback(dict)   {phase, progress}
  - sig_result(dict)     {success, message, detections_json, assignments_json}
  - sig_error(str)       connection / call errors
  - sig_goal_state(str)  'sent' | 'accepted' | 'rejected' | 'canceled' | 'done'
"""

from __future__ import annotations

import math
import threading
import time
from typing import Optional

import numpy as np
import rclpy
import tf2_ros
from PyQt5.QtCore import QObject, pyqtSignal
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from builtin_interfaces.msg import Duration as DurationMsg
from sensor_msgs.msg import Image, JointState, PointCloud2, Range
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from openarmx_ptp_box_align_msgs.action import AlignToBoxes


def _quat_to_rpy(x, y, z, w):
    """Quaternion → (roll, pitch, yaw) radians."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

# Action name the ptp_box_align_node advertises (see ptp_box_align_node.cpp /
# pick_and_place_tab BACKENDS["ptp"]).
PTP_ACTION = "/openarmx/ptp_align_to_boxes"

# Detection-tab video sources.
CAM_COLOR_TOPIC = "/camera/camera/color/image_raw"            # live colour feed
CAM_ANNOTATED_TOPIC = "/yolov8_node/image_annotated"          # YOLO overlay (on detect)
CLOUD_TOPIC = "/camera/camera/depth/color/points"            # 3D point cloud (XYZRGB)
TOF_TOPIC = "/tof/range"                                      # laser distance (VL53L0X)
DETECTIONS_TOPIC = "/yolov8_node/detections"                 # YOLO detections JSON (box colours)
PLACE_BOX_TOPIC = "/place_box/info"                          # big (place) box: wall pose + colour JSON


class PtpRosBridge(QObject):
    sig_feedback = pyqtSignal(dict)
    sig_result = pyqtSignal(dict)
    sig_error = pyqtSignal(str)
    sig_goal_state = pyqtSignal(str)
    # Detection-tab video / 3D.
    sig_image_cam = pyqtSignal(object, bool)  # (BGR ndarray, is_annotated): left pane
    sig_cloud = pyqtSignal(object, object)    # point cloud (right pane): xyz Nx3, rgba Nx4
    # Motion Jog tab: {joint_name: deg} from /joint_states.
    sig_joint_state = pyqtSignal(dict)
    # Detection tab sensor/detect readout.
    sig_tof = pyqtSignal(float)        # laser distance, metres
    sig_box_colors = pyqtSignal(object)  # list[str] of detected box colours
    sig_place_box = pyqtSignal(dict)   # big (place) box: ok/centroid/front/colour

    def __init__(self, action_name: str = PTP_ACTION, parent=None) -> None:
        super().__init__(parent)
        rclpy.init(args=None)
        self._node: Node = rclpy.create_node("openarmx_ptp_ui")
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)

        # action_name is provided by config (config/ptp_pnp_ui.yaml: action_name);
        # PTP_ACTION is the built-in default when constructed without config.
        self._action_name = action_name
        self._client = ActionClient(self._node, AlignToBoxes, action_name)
        self._active_goal_handle = None

        # ---- Detection-tab image subscriptions (lazy: only while that tab is
        # visible, to avoid decoding the 30 Hz depth/colour streams otherwise).
        # Subscriptions are created/destroyed on the SPIN thread (pending flag)
        # so rclpy entities are never mutated during spin_once().
        self._cv_bridge = None
        self._img_subs = {}                 # topic -> subscription
        self._img_active = False
        self._img_pending = None            # None | True(start) | False(stop)
        self._img_lock = threading.Lock()
        self._last_annot_t = 0.0            # monotonic time of last YOLO overlay
        self._last_cloud_t = 0.0           # cloud throttle clock
        self._cloud_min_dt = 0.1           # process cloud at most ~10 Hz
        self._cloud_max_pts = 40000        # downsample cap for the GL scatter
        self._img_qos = QoSProfile(
            depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE)

        # ---- Pipe Health rate-monitoring (lazy: only while that tab is shown).
        self._setup_diagnostics()

        # ---- Motion Jog tab: /joint_states (Joint) + TF (Cartesian base→hand_tcp).
        # BEST_EFFORT + VOLATILE subscriber = universally compatible (a best-effort
        # sub accepts reliable pubs → never mismatches on reliability; volatile
        # accepts volatile/transient_local) — addresses the "no QoS mismatch" ask.
        js_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT,
                            durability=QoSDurabilityPolicy.VOLATILE)
        self._node.create_subscription(JointState, "/joint_states",
                                       self._on_joint_state, js_qos)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self._node)
        self._latest_joint_deg = {}        # arm joint -> deg (for move planning)
        # Per-arm JTC command publishers (RELIABLE volatile matches the JTC's
        # /joint_trajectory subscription) — HOME/INIT and any jog move use these.
        traj_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE,
                              durability=QoSDurabilityPolicy.VOLATILE)
        self._traj_pub = {
            s: self._node.create_publisher(
                JointTrajectory,
                f"/{s}_joint_trajectory_controller/joint_trajectory", traj_qos)
            for s in ("left", "right")}

        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._thread.start()

    # ---------- background spin ----------

    def _spin_loop(self) -> None:
        while not self._stop_event.is_set():
            # Create/destroy image + diagnostics subs on THIS (executor) thread.
            self._apply_img_pending()
            self._apply_diag_pending()
            try:
                self._executor.spin_once(timeout_sec=0.1)
            except Exception as e:  # pragma: no cover - defensive
                self.sig_error.emit(f"Executor: {e}")

    # ---------- action ----------

    def server_ready(self, timeout_sec: float = 0.0) -> bool:
        try:
            return self._client.wait_for_server(timeout_sec=timeout_sec)
        except Exception:
            return False

    def run(self, z: float, roll_deg: float, pitch_deg: float,
            yaw_deg: float, arms: str) -> bool:
        """Send an AlignToBoxes goal. Returns False if the server is absent."""
        if self._active_goal_handle is not None:
            self.sig_error.emit("이미 실행 중인 골이 있습니다 — 먼저 Cancel 하세요.")
            return False
        if not self._client.wait_for_server(timeout_sec=2.0):
            self.sig_error.emit(
                f"{self._action_name} action server unavailable "
                "(ptp 정렬 백엔드를 먼저 Start 하세요)")
            return False

        goal = AlignToBoxes.Goal()
        goal.z = float(z)
        goal.roll_deg = float(roll_deg)
        goal.pitch_deg = float(pitch_deg)
        goal.yaw_deg = float(yaw_deg)
        goal.arms = str(arms)
        # ptp ignores confidence/prompts (detection is box_perception_node's job),
        # leave them at message defaults.

        send_future = self._client.send_goal_async(
            goal, feedback_callback=self._on_feedback)
        send_future.add_done_callback(self._on_goal_response)
        self.sig_goal_state.emit("sent")
        return True

    def _on_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as e:
            self.sig_error.emit(f"Goal send failed: {e}")
            self.sig_goal_state.emit("rejected")
            return
        if goal_handle is None or not goal_handle.accepted:
            self.sig_error.emit("Goal rejected by server")
            self.sig_goal_state.emit("rejected")
            return
        self._active_goal_handle = goal_handle
        self.sig_goal_state.emit("accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_feedback(self, msg) -> None:
        fb = msg.feedback
        self.sig_feedback.emit({
            "phase": fb.phase,
            "progress": float(fb.progress),
        })

    def _on_result(self, future) -> None:
        self._active_goal_handle = None
        try:
            wrapped = future.result()
        except Exception as e:
            self.sig_error.emit(f"Result fetch failed: {e}")
            self.sig_goal_state.emit("done")
            return
        r = wrapped.result
        self.sig_result.emit({
            "success": bool(r.success),
            "message": r.message,
            "detections_json": r.detections_json,
            "assignments_json": r.assignments_json,
        })
        self.sig_goal_state.emit("done")

    def cancel(self) -> None:
        if self._active_goal_handle is None:
            self.sig_error.emit("실행 중인 골이 없습니다.")
            return
        self._active_goal_handle.cancel_goal_async()
        self.sig_goal_state.emit("canceled")

    # ---------- status detection ----------

    def live_node_names(self) -> set:
        """Set of currently discoverable node names (short + fully-qualified).
        Used by main_window to detect launches running outside this UI."""
        names = set()
        try:
            for n, ns in self._node.get_node_names_and_namespaces():
                names.add(n)
                names.add((ns.rstrip("/") + "/" + n)
                          if ns and ns != "/" else "/" + n)
        except Exception:
            pass
        return names

    # ---------- Detection-tab images (lazy) ----------

    def images_start(self) -> None:
        with self._img_lock:
            self._img_pending = True

    def images_stop(self) -> None:
        with self._img_lock:
            self._img_pending = False

    def _apply_img_pending(self) -> None:
        with self._img_lock:
            want = self._img_pending
            self._img_pending = None
        if want is None or want == self._img_active:
            return
        if want:
            self._img_subs[CAM_COLOR_TOPIC] = self._node.create_subscription(
                Image, CAM_COLOR_TOPIC, self._on_color, self._img_qos)
            self._img_subs[CAM_ANNOTATED_TOPIC] = self._node.create_subscription(
                Image, CAM_ANNOTATED_TOPIC, self._on_annotated, self._img_qos)
            self._img_subs[CLOUD_TOPIC] = self._node.create_subscription(
                PointCloud2, CLOUD_TOPIC, self._on_cloud, self._img_qos)
            self._img_subs[TOF_TOPIC] = self._node.create_subscription(
                Range, TOF_TOPIC, self._on_tof, self._img_qos)
            self._img_subs[DETECTIONS_TOPIC] = self._node.create_subscription(
                String, DETECTIONS_TOPIC, self._on_detections, self._img_qos)
            self._img_subs[PLACE_BOX_TOPIC] = self._node.create_subscription(
                String, PLACE_BOX_TOPIC, self._on_place_box, self._img_qos)
        else:
            for sub in self._img_subs.values():
                try:
                    self._node.destroy_subscription(sub)
                except Exception:
                    pass
            self._img_subs.clear()
        self._img_active = want

    def _on_color(self, msg: Image) -> None:
        # Emit both streams tagged; main_window's overlay checkbox decides which
        # the left pane shows (and holds a fresh overlay over the 30 Hz colour).
        bgr = self._decode(msg)
        if bgr is not None:
            self.sig_image_cam.emit(bgr, False)

    def _on_annotated(self, msg: Image) -> None:
        bgr = self._decode(msg)
        if bgr is not None:
            self.sig_image_cam.emit(bgr, True)

    def _on_cloud(self, msg: PointCloud2) -> None:
        # Throttle (cloud is large) and downsample for the GL scatter.
        now = time.monotonic()
        if now - self._last_cloud_t < self._cloud_min_dt:
            return
        self._last_cloud_t = now
        try:
            from sensor_msgs_py import point_cloud2 as pc2
            rec = pc2.read_points(
                msg, field_names=("x", "y", "z", "rgb"), skip_nans=True)
            n = len(rec)
            if n == 0:
                return
            step = max(1, n // self._cloud_max_pts)
            rec = rec[::step]
            xyz = np.stack([rec["x"], rec["y"], rec["z"]], axis=-1).astype(np.float32)
            # 'rgb' is a float32 with packed 0x00RRGGBB; reinterpret bits → RGBA.
            try:
                rgb_i = np.ascontiguousarray(rec["rgb"], dtype=np.float32).view(np.uint32)
                rgba = np.empty((xyz.shape[0], 4), np.float32)
                rgba[:, 0] = ((rgb_i >> 16) & 0xFF) / 255.0
                rgba[:, 1] = ((rgb_i >> 8) & 0xFF) / 255.0
                rgba[:, 2] = (rgb_i & 0xFF) / 255.0
                rgba[:, 3] = 1.0
            except Exception:
                # No usable rgb → colour by height (z).
                z = xyz[:, 2]
                t = (z - z.min()) / max(1e-6, (z.max() - z.min()))
                rgba = np.empty((xyz.shape[0], 4), np.float32)
                rgba[:, 0] = t
                rgba[:, 1] = 0.4
                rgba[:, 2] = 1.0 - t
                rgba[:, 3] = 1.0
            self.sig_cloud.emit(xyz, rgba)
        except Exception as e:
            self.sig_error.emit(f"Cloud parse: {e}")

    def _on_tof(self, msg: Range) -> None:
        self.sig_tof.emit(float(msg.range))

    def _on_detections(self, msg: String) -> None:
        import json
        from collections import Counter
        try:
            dets = json.loads(msg.data).get("detections", [])
        except Exception:
            self.sig_box_colors.emit([])
            return
        counts = Counter()
        for d in dets:
            name = str(d.get("class_name", ""))
            colour = name.split("-")[-1].split("_")[-1] or name  # mini-box-blue → blue
            if colour:
                counts[colour] += 1
        self.sig_box_colors.emit(
            [f"{c}×{n}" if n > 1 else c for c, n in counts.items()])

    def _on_place_box(self, msg: String) -> None:
        import json
        try:
            d = json.loads(msg.data)
        except Exception:
            return
        wall = d.get("wall", {}) or {}
        colour = d.get("color", {}) or {}
        tof = d.get("tof", {}) or {}
        self.sig_place_box.emit({
            "ok": bool(wall.get("ok")),
            "centroid": wall.get("centroid"),          # [x, y, z] (body) or None
            "front_distance": wall.get("front_distance"),
            "width": wall.get("width"),
            "color_name": colour.get("name", ""),
            "color_conf": colour.get("confidence", 0.0),
            "tof_present": bool(tof.get("present")),
            "tof_distance": tof.get("distance_m"),
        })

    def _decode(self, msg: Image):
        """Decode a sensor_msgs/Image to a contiguous BGR uint8 ndarray.

        Colour is decoded with numpy directly (cv_bridge imgmsg_to_cv2('bgr8')
        is broken on this PC — numpy 2.x/OpenCV skew, see cv_bridge_numpy2_bug);
        depth (16UC1/mono16) goes through cv_bridge passthrough → JET colormap."""
        try:
            import cv2
            if msg.encoding in ("16UC1", "mono16"):
                if self._cv_bridge is None:
                    from cv_bridge import CvBridge
                    self._cv_bridge = CvBridge()
                depth = self._cv_bridge.imgmsg_to_cv2(msg, "passthrough")
                vis = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth, alpha=0.03), cv2.COLORMAP_JET)
            else:
                enc = (msg.encoding or "").lower()
                buf = np.frombuffer(msg.data, dtype=np.uint8)
                if enc in ("rgb8", "bgr8"):
                    im = buf.reshape(msg.height, msg.width, 3)
                    vis = cv2.cvtColor(im, cv2.COLOR_RGB2BGR) if enc == "rgb8" else im
                elif enc == "mono8":
                    vis = cv2.cvtColor(
                        buf.reshape(msg.height, msg.width), cv2.COLOR_GRAY2BGR)
                elif enc in ("rgba8", "bgra8"):
                    im = buf.reshape(msg.height, msg.width, 4)
                    vis = cv2.cvtColor(
                        im, cv2.COLOR_RGBA2BGR if enc == "rgba8" else cv2.COLOR_BGRA2BGR)
                else:
                    if self._cv_bridge is None:
                        from cv_bridge import CvBridge
                        self._cv_bridge = CvBridge()
                    vis = self._cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            return np.ascontiguousarray(vis)
        except Exception as e:
            self.sig_error.emit(f"Image decode: {e}")
            return None

    # ---------- Pipe Health rate monitoring (lazy) ----------

    def _setup_diagnostics(self) -> None:
        from openarmx_ptp_ui import diag_spec as ds
        self._diag = {}                       # topic -> {"count", "last"}
        self._diag_t0 = time.monotonic()
        self._diag_qos = {
            ds.QOS_SENSOR: QoSProfile(
                depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE),
            ds.QOS_LATCHED: QoSProfile(
                depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL),
            ds.QOS_DEFAULT: QoSProfile(
                depth=10, reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE),
        }
        self._diag_lazy = []
        for _cat, topic, mtype, qkind, _kind, _detail in ds.TOPIC_SPECS:
            self._diag[topic] = {"count": 0, "last": 0.0}
            self._diag_lazy.append((topic, mtype, qkind))
        self._diag_subs = {}
        self._diag_active = False
        self._diag_pending = None
        self._diag_lock = threading.Lock()

    def diag_start(self) -> None:
        with self._diag_lock:
            self._diag_pending = True

    def diag_stop(self) -> None:
        with self._diag_lock:
            self._diag_pending = False

    def _apply_diag_pending(self) -> None:
        with self._diag_lock:
            want = self._diag_pending
            self._diag_pending = None
        if want is None or want == self._diag_active:
            return
        if want:
            for topic, mtype, qkind in self._diag_lazy:
                def _cb(_msg, t=topic):
                    d = self._diag[t]
                    d["count"] += 1
                    d["last"] = time.monotonic()
                self._diag_subs[topic] = self._node.create_subscription(
                    mtype, topic, _cb, self._diag_qos[qkind])
            for d in self._diag.values():
                d["count"] = 0
            self._diag_t0 = time.monotonic()
        else:
            for sub in self._diag_subs.values():
                try:
                    self._node.destroy_subscription(sub)
                except Exception:
                    pass
            self._diag_subs.clear()
        self._diag_active = want

    def diag_consume_rates(self) -> dict:
        """{topic: {hz, age}} measured since the last call, then reset. Exactly
        ONE consumer (Pipe Health tab) — reading resets the per-topic counters."""
        now = time.monotonic()
        elapsed = max(now - self._diag_t0, 1e-3)
        out = {}
        for topic, d in self._diag.items():
            out[topic] = {
                "hz": d["count"] / elapsed,
                "age": (now - d["last"]) if d["last"] > 0 else None,
            }
            d["count"] = 0
        self._diag_t0 = now
        return out

    # ---------- Motion Jog: joint states + Cartesian (TF) ----------

    _ARM_JOINTS = [f"openarmx_{s}_joint{i}"
                   for s in ("left", "right") for i in range(1, 8)]

    def _on_joint_state(self, msg: JointState) -> None:
        want = set(self._ARM_JOINTS)
        deg = {n: math.degrees(p) for n, p in zip(msg.name, msg.position)
               if n in want}
        if deg:
            self._latest_joint_deg.update(deg)
            self.sig_joint_state.emit(deg)

    def latest_joint_deg(self) -> dict:
        return dict(self._latest_joint_deg)

    def send_arm_trajectory(self, positions_deg: dict, duration_sec: float) -> None:
        """Publish one JointTrajectory point per arm to its JTC (HOME/INIT/jog).
        positions_deg maps arm joint names → degrees; an arm with no entry is
        left untouched."""
        sec = int(duration_sec)
        nsec = int((duration_sec - sec) * 1e9)
        stamp = self._node.get_clock().now().to_msg()
        for side in ("left", "right"):
            names = [f"openarmx_{side}_joint{i}" for i in range(1, 8)]
            if not any(n in positions_deg for n in names):
                continue
            traj = JointTrajectory()
            traj.header.stamp = stamp
            traj.joint_names = names
            pt = JointTrajectoryPoint()
            pt.positions = [math.radians(positions_deg.get(n, 0.0)) for n in names]
            pt.velocities = [0.0] * 7
            pt.time_from_start = DurationMsg(sec=sec, nanosec=nsec)
            traj.points = [pt]
            self._traj_pub[side].publish(traj)

    def park_arms_home(self, speed_deg_s: float = 25.0, tol_deg: float = 3.0,
                       extra_timeout: float = 2.0, tick=None) -> bool:
        """Move BOTH arms to HOME (0°) and block until they arrive (or timeout).
        Call BEFORE killing the controllers/hardware on a graceful shutdown:
        cutting MIT motor torque mid-air lets the arm drop under gravity.
        Returns False (skip) if no JTC is listening (no robot / SIL down). `tick`
        is invoked each poll (pass QApplication.processEvents) to keep the GUI
        responsive; the spin thread keeps latest_joint_deg fresh meanwhile."""
        subs = sum(self._node.count_subscribers(
            f"/{s}_joint_trajectory_controller/joint_trajectory")
            for s in ("left", "right"))
        cur = self.latest_joint_deg()
        if subs == 0 or not cur:
            return False
        home = {n: 0.0 for n in self._ARM_JOINTS}
        max_delta = max((abs(cur.get(n, 0.0)) for n in self._ARM_JOINTS), default=0.0)
        duration = max(1.5, max_delta / max(1.0, speed_deg_s))
        self.send_arm_trajectory(home, duration)
        deadline = time.monotonic() + duration + extra_timeout
        while time.monotonic() < deadline:
            if tick is not None:
                tick()
            cur = self.latest_joint_deg()
            if cur and all(abs(cur.get(n, 1e3)) <= tol_deg for n in self._ARM_JOINTS):
                return True
            time.sleep(0.05)
        return True

    def get_ee_pose(self, arm: str, frame_id: str = "",
                    link: str = "") -> Optional[dict]:
        """TF pose of hand_tcp (default) in the base frame. None if the
        transform is unavailable (e.g. /joint_states / robot TF not flowing)."""
        target = link or f"openarmx_{arm}_hand_tcp"
        base = frame_id or "openarmx_body_link0"
        try:
            if not self._tf_buffer.can_transform(base, target, Time()):
                return None
            ts = self._tf_buffer.lookup_transform(base, target, Time())
        except Exception:
            return None
        tr = ts.transform.translation
        q = ts.transform.rotation
        roll, pitch, yaw = _quat_to_rpy(q.x, q.y, q.z, q.w)
        return {"x": tr.x, "y": tr.y, "z": tr.z,
                "roll": roll, "pitch": pitch, "yaw": yaw}

    # ---------- shutdown ----------

    def shutdown(self) -> None:
        self._stop_event.set()
        self._thread.join(timeout=2.0)
        try:
            self._node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
