"""Camera tab for openarmx_scenario_ui.

Shows the live D435 RealSense image (pulled from the running ROS driver via the
shared ScenarioRosBridge, NOT pyrealsense2) and overlays YOLOv8 detection
results obtained from the remote Raspberry-Pi-5 + Hailo-8 server.

The overlay draw() and the request_remote() HTTP helper are ported from
openarmx_ws/src/Sensor/RGBD/view_remote_seg.py (same detection JSON schema).

Image streaming + remote inference are active only while this tab is visible
(showEvent subscribes, hideEvent unsubscribes) to avoid constant image
bandwidth and inference load when another tab is in front.
"""

from __future__ import annotations

import json
import threading
import time
import urllib.error
import urllib.request
from urllib.parse import urlencode

import numpy as np
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (
    QCheckBox, QComboBox, QDoubleSpinBox, QHBoxLayout, QLabel, QLineEdit,
    QVBoxLayout, QWidget,
)


# NOTE: cv2 is imported LAZILY (inside the functions below), never at module
# top. The pip `opencv-python` ships its own bundled Qt xcb plugin; importing
# cv2 BEFORE QApplication is created hijacks QT_QPA_PLATFORM_PLUGIN_PATH and the
# whole GUI fails with "Could not load the Qt platform plugin xcb" (core dump).
# This module is imported by main_window at startup (before QApplication), so a
# top-level `import cv2` here crashes the app; deferring it past app init is safe.

# BGR colours matching the class names (ported from view_remote_seg.py:
# mini-box-blue/green/orange/red/yellow).
CLASS_COLORS = {0: (255, 0, 0), 1: (0, 200, 0), 2: (0, 165, 255),
                3: (0, 0, 255), 4: (0, 255, 255)}

COLOR_TOPIC = "/camera/camera/color/image_raw"
DEPTH_TOPIC = "/camera/camera/aligned_depth_to_color/image_raw"
TECHMAN_TOPIC = "/techman_image"
DEFAULT_SERVER = "http://10.42.0.2:8080/detect"


def request_remote(url: str, jpeg: bytes, timeout: float) -> dict:
    """POST a JPEG frame to the remote Hailo server, return the JSON dict.
    Ported from view_remote_seg.py."""
    req = urllib.request.Request(
        url, data=jpeg, method="POST", headers={"Content-Type": "image/jpeg"})
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read().decode("utf-8"))


def draw(frame: np.ndarray, dets: list) -> np.ndarray:
    """Overlay translucent filled polygons + boxes + centroid + label.
    Ported from view_remote_seg.py."""
    import cv2  # lazy: see module note on the cv2<->PyQt5 xcb-plugin clash
    out = frame.copy()
    overlay = out.copy()
    for d in dets:                                   # filled masks first (translucent)
        color = CLASS_COLORS.get(int(d.get("class_id", -1)), (200, 200, 200))
        poly = d.get("polygon")
        if poly and len(poly) >= 3:
            cv2.fillPoly(overlay, [np.array(poly, np.int32)], color)
    out = cv2.addWeighted(overlay, 0.45, out, 0.55, 0)
    for d in dets:                                   # boxes + centroid + labels
        color = CLASS_COLORS.get(int(d.get("class_id", -1)), (200, 200, 200))
        x1, y1, x2, y2 = [int(v) for v in d["bbox_xyxy"]]
        cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
        cen = d.get("centroid_px", [(x1 + x2) / 2, (y1 + y2) / 2])
        cx, cy = int(cen[0]), int(cen[1])
        cv2.circle(out, (cx, cy), 4, (255, 255, 255), -1)
        cv2.circle(out, (cx, cy), 5, color, 2)
        label = f"{d.get('class_name', '?')} {d.get('confidence', 0.0):.2f}"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        yt = max(0, y1 - th - 6)
        cv2.rectangle(out, (x1, yt), (x1 + tw + 2, yt + th + 6), color, -1)
        tc = (0, 0, 0) if sum(color) > 400 else (255, 255, 255)
        cv2.putText(out, label, (x1 + 1, yt + th + 1),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, tc, 1)
    return out


class CameraTab(QWidget):
    def __init__(self, bridge, parent=None) -> None:
        super().__init__(parent)
        self._bridge = bridge

        # ---- shared state (rebound atomically; no lock — see module doc) ----
        self._latest_frame = None      # BGR ndarray | None (sig_image slot writes)
        self._latest_dets = []         # list (worker writes, render reads)
        self._det_status = ""          # str (worker writes)
        self._running = False
        self._worker = None            # threading.Thread | None

        # ---- snapshot attrs (worker + render read these, never Qt widgets) ----
        self._topic = COLOR_TOPIC
        self._yolo_on = True
        self._conf = 0.30
        self._url = DEFAULT_SERVER

        # ---- fps counters ----
        self._vid_t0 = time.monotonic()
        self._vid_n = 0
        self._vid_fps = 0.0
        self._det_t0 = time.monotonic()
        self._det_n = 0
        self._det_fps = 0.0

        self._build_ui()

        self._bridge.sig_image.connect(self._on_frame)

        # ~33ms timer: refreshes the status line + re-snapshots settings so the
        # worker picks up control changes even between frames.
        self._ui_timer = QTimer(self)
        self._ui_timer.timeout.connect(self._on_ui_tick)

    # ------------------------------------------------------------------
    # UI construction (mirrors existing tab style; _btn helper from
    # joint_control_tab — kept for colored buttons if needed; layout uses
    # combos/checkbox/spin like the other tabs).
    # ------------------------------------------------------------------

    @staticmethod
    def _btn(text, bg, hover, pressed):
        from PyQt5.QtWidgets import QPushButton
        b = QPushButton(text)
        b.setMinimumWidth(96)
        b.setMinimumHeight(34)
        b.setStyleSheet(
            f"QPushButton {{ background-color:{bg}; color:white; font-weight:bold; "
            f"border:none; border-radius:4px; padding:6px 14px; }}"
            f"QPushButton:hover {{ background-color:{hover}; }}"
            f"QPushButton:pressed {{ background-color:{pressed}; }}")
        return b

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)

        ctrl = QHBoxLayout()

        ctrl.addWidget(QLabel("Source:"))
        self.cmbSource = QComboBox()
        self.cmbSource.addItem("D435 Color", userData=COLOR_TOPIC)
        self.cmbSource.addItem("D435 Depth", userData=DEPTH_TOPIC)
        self.cmbSource.addItem("techman_image", userData=TECHMAN_TOPIC)
        ctrl.addWidget(self.cmbSource)

        self.chkYolo = QCheckBox("YOLOv8 (Hailo) 오버레이")
        self.chkYolo.setChecked(True)
        ctrl.addWidget(self.chkYolo)

        ctrl.addWidget(QLabel("conf:"))
        self.spnConf = QDoubleSpinBox()
        self.spnConf.setRange(0.0, 1.0)
        self.spnConf.setSingleStep(0.05)
        self.spnConf.setDecimals(2)
        self.spnConf.setValue(0.30)
        ctrl.addWidget(self.spnConf)

        ctrl.addWidget(QLabel("Server:"))
        self.edtServer = QLineEdit(DEFAULT_SERVER)
        self.edtServer.setMinimumWidth(260)
        ctrl.addWidget(self.edtServer)

        ctrl.addStretch()

        self.lblStat = QLabel("video 0.0 Hz | det 0.0 Hz 0 |")
        self.lblStat.setStyleSheet("font-family:monospace; color:#444;")
        ctrl.addWidget(self.lblStat)

        root.addLayout(ctrl)

        self.imgLabel = QLabel("no image")
        self.imgLabel.setAlignment(Qt.AlignCenter)
        self.imgLabel.setMinimumSize(640, 480)
        self.imgLabel.setStyleSheet("background-color:#202020; color:#888;")
        self.imgLabel.setScaledContents(False)
        root.addWidget(self.imgLabel, stretch=1)

        # Re-snapshot + re-subscribe when controls change.
        self.cmbSource.currentIndexChanged.connect(self._on_source_changed)
        self.chkYolo.toggled.connect(self._snapshot_settings)
        self.spnConf.valueChanged.connect(self._snapshot_settings)
        self.edtServer.textChanged.connect(self._snapshot_settings)

    # ------------------------------------------------------------------
    # Settings snapshot (controls -> plain attrs read by worker/render)
    # ------------------------------------------------------------------

    def _snapshot_settings(self) -> None:
        self._topic = self.cmbSource.currentData()
        self._yolo_on = self.chkYolo.isChecked()
        self._conf = self.spnConf.value()
        self._url = self.edtServer.text()

    def _on_source_changed(self, _idx: int) -> None:
        self._snapshot_settings()
        if self._running:
            self._bridge.set_image_topic(self._topic)
            # New source -> stale overlay no longer applies.
            self._latest_frame = None
            self._latest_dets = []

    # ------------------------------------------------------------------
    # Frame in (MAIN thread, queued via sig_image)
    # ------------------------------------------------------------------

    def _on_frame(self, frame) -> None:
        self._latest_frame = frame
        self._vid_n += 1
        now = time.monotonic()
        if now - self._vid_t0 >= 1.0:
            self._vid_fps = self._vid_n / (now - self._vid_t0)
            self._vid_t0, self._vid_n = now, 0
        self._render()

    # ------------------------------------------------------------------
    # Render (MAIN thread)
    # ------------------------------------------------------------------

    def _render(self) -> None:
        import cv2  # lazy: see module note on the cv2<->PyQt5 xcb-plugin clash
        f = self._latest_frame
        if f is None:
            self.imgLabel.setText("no image")
            self._update_stat()
            return
        out = f.copy()
        if self._yolo_on and self._topic == COLOR_TOPIC:
            out = draw(out, self._latest_dets)
        rgb = cv2.cvtColor(out, cv2.COLOR_BGR2RGB)
        h, w = rgb.shape[:2]
        qimg = QImage(rgb.data, w, h, 3 * w, QImage.Format_RGB888).copy()
        pix = QPixmap.fromImage(qimg).scaled(
            self.imgLabel.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.imgLabel.setPixmap(pix)
        self._update_stat()

    def _update_stat(self) -> None:
        self.lblStat.setText(
            f"video {self._vid_fps:4.1f} Hz | "
            f"det {self._det_fps:4.1f} Hz {len(self._latest_dets)} | "
            f"{self._det_status}")

    def _on_ui_tick(self) -> None:
        # Keep snapshot fresh + refresh status even if frames stall.
        self._snapshot_settings()
        self._update_stat()

    # ------------------------------------------------------------------
    # HTTP inference worker (background thread)
    # ------------------------------------------------------------------

    def _worker_loop(self) -> None:
        import cv2  # lazy: see module note on the cv2<->PyQt5 xcb-plugin clash
        while self._running:
            f = self._latest_frame
            if f is None or not self._yolo_on or self._topic != COLOR_TOPIC:
                self._latest_dets = []
                time.sleep(0.1)
                continue
            ok, buf = cv2.imencode(
                ".jpg", f, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if not ok:
                time.sleep(0.02)
                continue
            url = f"{self._url}?{urlencode({'conf': self._conf, 'masks': 1})}"
            try:
                resp = request_remote(url, bytes(buf), timeout=5.0)
                self._latest_dets = resp.get("detections", [])
                self._det_status = ""
                self._det_n += 1
                now = time.monotonic()
                if now - self._det_t0 >= 1.0:
                    self._det_fps = self._det_n / (now - self._det_t0)
                    self._det_t0, self._det_n = now, 0
            except Exception as e:
                self._det_status = str(e)[:60]
            time.sleep(0.02)

    # ------------------------------------------------------------------
    # Lifecycle (subscribe only while visible)
    # ------------------------------------------------------------------

    def _start(self) -> None:
        if self._running:
            return
        self._snapshot_settings()
        self._bridge.set_image_topic(self._topic)
        self._running = True
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()
        self._ui_timer.start(33)

    def _stop(self) -> None:
        self._ui_timer.stop()
        self._running = False
        if self._worker is not None:
            self._worker.join(timeout=1.0)
            self._worker = None
        self._bridge.stop_image()
        self._latest_frame = None
        self._latest_dets = []

    def showEvent(self, event) -> None:
        super().showEvent(event)
        self._start()

    def hideEvent(self, event) -> None:
        super().hideEvent(event)
        self._stop()

    def shutdown(self) -> None:
        self._stop()
