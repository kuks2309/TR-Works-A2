"""Main window for openarmx_ptp_ui — the ptp pick-and-place dedicated UI.

DISPLAY-wiring layer (표시 결선). Loads the Qt Designer layout (ui/ptp_pnp_ui.ui)
and connects its widgets to the two DRIVE layers, keeping display and drive
strictly separated:

  * DISPLAY (표시)        : ui/ptp_pnp_ui.ui  (this module only wires it, never
                            touches ROS or subprocess directly except via the
                            drive layers below)
  * DRIVE — process (구동): managed_process.ManagedProcess  (start/stop launches)
  * DRIVE — ROS (구동)    : ptp_ros_bridge.PtpRosBridge      (AlignToBoxes action)

Operational values (action name, status timer, detect-workspace setup, hardware
CAN interfaces, AlignToBoxes goal defaults) live in config/ptp_pnp_ui.yaml and
are loaded at startup (load-only). The launch presets' internal identifiers
(node/proc/sweep patterns) stay in build_presets — they are not operational
values a user tunes.

The Qt thread is never blocked: the ROS bridge spins rclpy in its own thread and
reports back via Qt signals; process control is fire-and-forget subprocess work.
"""

from __future__ import annotations

import math
import os
import re
import signal
import subprocess
import time
from datetime import datetime

import numpy as np
from ament_index_python.packages import get_package_share_directory
from PyQt5 import uic
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication, QDoubleSpinBox, QGridLayout, QGroupBox, QHBoxLayout, QLabel,
    QMainWindow, QMessageBox, QPushButton, QVBoxLayout, QWidget,
)

from openarmx_ptp_ui.app_config import load_config
from openarmx_ptp_ui.managed_process import ManagedProcess
from openarmx_ptp_ui.pipe_health_tab import PipeHealthTab
from openarmx_ptp_ui.ptp_ros_bridge import PtpRosBridge


# Start/Run = green (go), Stop/Cancel = red — mirrors the scenario UI buttons.
_BTN_GREEN_QSS = (
    "QPushButton { background-color:#43A047; color:white; font-weight:bold; "
    "border:none; border-radius:4px; padding:5px 14px; }"
    "QPushButton:hover { background-color:#388E3C; }"
    "QPushButton:pressed { background-color:#2E7D32; }"
)
_BTN_RED_QSS = (
    "QPushButton { background-color:#E53935; color:white; font-weight:bold; "
    "border:none; border-radius:4px; padding:5px 14px; }"
    "QPushButton:hover { background-color:#C62828; }"
    "QPushButton:pressed { background-color:#B71C1C; }"
)


# Motion Jog HOME / INIT joint targets (deg) — joint_data.{HOME,INIT}_POSITION_DEG.
_HOME_DEG = {f"openarmx_{s}_joint{i}": 0.0
             for s in ("left", "right") for i in range(1, 8)}
_INIT_DEG = dict(_HOME_DEG)
_INIT_DEG.update({
    "openarmx_left_joint1": 50.0, "openarmx_left_joint4": 100.0,
    "openarmx_left_joint7": 50.0,
    "openarmx_right_joint1": -50.0, "openarmx_right_joint4": 100.0,
    "openarmx_right_joint7": -50.0,
})


def _pkg_config(filename: str) -> str:
    """Resolve <openarmx_ptp_ui>/config/<filename>. Prefer the SRC copy (so
    hand-edits apply without a rebuild), fall back to the installed share copy
    (project rule: load configs from src, not install/share)."""
    # SRC: this file is <pkg>/openarmx_ptp_ui/main_window.py → <pkg>/config/...
    src = os.path.join(
        os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
        "config", filename)
    if os.path.isfile(src):
        return src
    try:
        return os.path.join(
            get_package_share_directory("openarmx_ptp_ui"),
            "config", filename)
    except Exception:
        return ""


def _find_config() -> str:
    return _pkg_config("ptp_pnp_ui.yaml")


# RViz config: a COPY of the scenario RViz (openarmx_scenario.rviz) kept INSIDE
# this package as config/openarmx_ptp.rviz, so the ptp UI is self-contained and
# shows the same view as the scenario without depending on openarmx_scenario_player.
# (User instruction 2026-06-08: "scenario 에서 실행되는 rviz 를 복사해서 사용".)
_PTP_RVIZ = _pkg_config("openarmx_ptp.rviz")


def build_presets(cfg: dict) -> dict:
    """Launch presets keyed by row. Operational fields (hardware CAN, detect
    workspace setup) come from `cfg`; the node/proc/sweep identifiers and which
    .ui widgets drive each row are structural and stay here.

    Each preset:
      cmd     : argv to launch
      confirm : warn before start (real hardware)
      nodes   : ros2 node names that mean "running" (checked via rclpy graph)
      procs   : /proc cmdline substrings that mean "running"
      sweep   : extra kill patterns on stop (orphan reaping)
      widgets : (start_btn, stop_btn, status_label) objectNames in the .ui
    """
    can = cfg["hw_can"]
    # 3d_detect_ws is a separate workspace; source it then `exec` so the same
    # PID/process-group survives (ManagedProcess.stop's killpg tears it down).
    detect_cmd = [
        "bash", "-c",
        f"source {cfg['detect_ws_setup']} && exec ros2 launch yolov8_detection "
        "yolo_remote.launch.py node_name:=yolov8_node",
    ]
    return {
        "hw_sil": {
            "cmd": ["ros2", "launch", "openarmx_scenario_player",
                    "openarmx_hardware.launch.py", "use_fake_hardware:=true"],
            "confirm": False,
            "nodes": [],
            "procs": ["use_fake_hardware:=true"],
            "sweep": ["openarmx_hardware.launch", "ros2_control_node"],
            "widgets": ("btnHwSilStart", "btnHwSilStop", "lblHwSil"),
        },
        "hw_hw": {
            "cmd": ["ros2", "launch", "openarmx_scenario_player",
                    "openarmx_hardware.launch.py", "use_fake_hardware:=false",
                    f"control_mode:={can['mode']}",
                    f"right_can_interface:={can['right']}",
                    f"left_can_interface:={can['left']}",
                    f"can_fd:={'true' if can.get('can_fd') else 'false'}"],
            "confirm": True,
            "nodes": [],
            "procs": ["use_fake_hardware:=false"],
            "sweep": ["openarmx_hardware.launch", "ros2_control_node"],
            "widgets": ("btnHwHwStart", "btnHwHwStop", "lblHwHw"),
        },
        # L1 controllers. Two spawners in parallel: the active set (JSB + L/R JTC
        # + grippers) AND the L/R arm_position_controllers loaded --inactive.
        # The pick sequence needs the latter: it switches each arm between its
        # JTC and arm_position (forward_position) controller, which SHARE the
        # position command interface (so arm_position must be loaded-inactive
        # while JTC is active). `--inactive` applies to a whole spawner call, so
        # it cannot be mixed into the active spawner — hence two spawners. Both
        # use --unload-on-kill so Stop unloads everything for a clean restart.
        "ctrl": {
            "cmd": [
                "bash", "-c",
                "ros2 run controller_manager spawner joint_state_broadcaster "
                "left_joint_trajectory_controller right_joint_trajectory_controller "
                "left_gripper_controller right_gripper_controller "
                "-c /controller_manager --unload-on-kill & "
                "ros2 run controller_manager spawner "
                "left_arm_position_controller right_arm_position_controller "
                "--inactive -c /controller_manager --unload-on-kill & "
                "wait",
            ],
            "confirm": False,
            "nodes": ["/left_joint_trajectory_controller",
                      "/right_joint_trajectory_controller"],
            "procs": ["controller_manager spawner"],
            "sweep": [],
            "widgets": ("btnCtrlStart", "btnCtrlStop", "lblCtrl"),
        },
        "cam": {
            "cmd": ["ros2", "launch", "openarmx_scenario_player",
                    "d435_camera.launch.py"],
            "confirm": False,
            "nodes": [],
            "procs": ["realsense2_camera", "d435_camera.launch"],
            "sweep": ["d435_camera.launch", "realsense2_camera"],
            "widgets": ("btnCamStart", "btnCamStop", "lblCam"),
        },
        "det": {
            "cmd": detect_cmd,
            "confirm": False,
            "nodes": ["/yolov8_node", "/box_perception_node"],
            "procs": ["yolo_remote_node", "box_perception_node",
                      "yolo_remote.launch"],
            "sweep": ["yolo_remote.launch", "yolo_remote_node",
                      "box_perception_node"],
            "widgets": ("btnDetStart", "btnDetStop", "lblDet"),
        },
        "backend": {
            "cmd": ["ros2", "launch", "openarmx_ptp_box_align",
                    "ptp_box_align.launch.py"],
            "confirm": False,
            "nodes": ["/ptp_box_align_node"],
            "procs": ["ptp_box_align_node", "ptp_box_align.launch"],
            "sweep": ["ptp_box_align.launch", "ptp_box_align_node"],
            "widgets": ("btnBackendStart", "btnBackendStop", "lblBackend"),
        },
        "rviz": {
            "cmd": ["rviz2", "-d", _PTP_RVIZ,
                    "--ros-args", "-r", "__node:=openarmx_scenario_rviz"],
            "confirm": False,
            "nodes": ["/openarmx_scenario_rviz"],
            "procs": ["openarmx_scenario.rviz"],
            "sweep": ["openarmx_scenario.rviz"],
            "widgets": ("btnRvizStart", "btnRvizStop", "lblRviz"),
        },
        # TOF 수신: ESP32-C3(VL53L0X) USB-serial(/dev/ttyACM0,115200) → /tof/range.
        # 노드 기본 파라미터가 place_box_detection.yaml 의 tof_serial_driver 와 동일해
        # 단독 `ros2 run` 으로 충분(검출/벽 노드 없이 수신만).
        "tof": {
            "cmd": ["ros2", "run", "place_box_detection",
                    "tof_serial_driver_node"],
            "confirm": False,
            "nodes": ["/tof_serial_driver"],
            "procs": ["tof_serial_driver_node"],
            "sweep": ["tof_serial_driver_node"],
            "widgets": ("btnTofStart", "btnTofStop", "lblTof"),
        },
        # 중력보상: 양팔 g(q) 피드포워드 (L1 forward_effort_controller 스폰 + node).
        # Stop 은 _stop 에서 enable_compensation=false 로 토크 0 화 후 종료(특별처리).
        "gravity": {
            "cmd": ["ros2", "launch", "openarmx_gravity_comp",
                    "gravity_comp.launch.py", "g_scale:=0.95"],
            "confirm": False,
            "nodes": ["/gravity_comp_node"],
            "procs": ["gravity_comp.launch"],
            "sweep": ["gravity_comp_node"],
            "widgets": ("btnGravityStart", "btnGravityStop", "lblGravity"),
        },
        # ee_leader 마커: RViz 6-DoF 드래그 티칭(뷰 전용). openarmx 프레임/토픽 지정.
        "ee_markers": {
            "cmd": ["ros2", "launch", "ee_leader_marker",
                    "ee_leader_marker_bimanual.launch.py",
                    "base_frame:=openarmx_body_link0",
                    "left_controlled_link:=openarmx_left_link7",
                    "right_controlled_link:=openarmx_right_link7",
                    "left_goal_topic:=/openarmx/left/ee_leader/goal_pose",
                    "right_goal_topic:=/openarmx/right/ee_leader/goal_pose",
                    "start_rviz:=false"],
            "confirm": False,
            "nodes": ["/ee_leader_left_marker", "/ee_leader_right_marker"],
            "procs": ["ee_leader_marker_bimanual"],
            "sweep": ["ee_leader_marker_bimanual"],
            "widgets": ("btnEeStart", "btnEeStop", "lblEe"),
        },
    }


class PtpPnpMainWindow(QMainWindow):
    def __init__(self, with_rviz: bool = True) -> None:
        super().__init__()
        ui_path = os.path.join(
            get_package_share_directory("openarmx_ptp_ui"),
            "ui", "ptp_pnp_ui.ui")
        uic.loadUi(ui_path, self)

        # Operational config (load-only) + launch presets built from it.
        self._config_path = _find_config()
        self._cfg = load_config(self._config_path)
        self._presets = build_presets(self._cfg)

        # DRIVE layers.
        self._procs = {key: ManagedProcess(f"ptp_ui_{key}") for key in self._presets}
        self._bridge = PtpRosBridge(
            action_name=self._cfg["action_name"], parent=self)
        self._started_keys = set()      # keys started this session (cleanup)
        self._node_cache = set()        # last live node names

        self._style_buttons()
        self._wire_launch_rows()
        self._wire_action_controls()
        self._apply_goal_defaults()

        # bridge -> display signals.
        self._bridge.sig_feedback.connect(self._on_feedback)
        self._bridge.sig_result.connect(self._on_result)
        self._bridge.sig_error.connect(self._on_error)
        self._bridge.sig_goal_state.connect(self._on_goal_state)
        # Detection-tab video (left: camera + YOLO overlay) + 3D point cloud (right).
        self._setup_cloud_view()
        self._bridge.sig_image_cam.connect(self._on_img_cam)
        self._bridge.sig_cloud.connect(self._on_cloud)
        # Stream images/cloud only while the Detection tab is visible (lazy).
        self.tabs.currentChanged.connect(self._on_tab_changed)
        # YOLO overlay toggle + one-shot detect trigger (Detection tab).
        self._overlay_on = False
        self._has_annot = False
        self.chkOverlay.toggled.connect(self._on_overlay_toggled)
        self.btnDetectOnce.clicked.connect(self._detect_once)
        # Detection-tab sensor/detect readout (laser distance + box colours).
        self._bridge.sig_tof.connect(self._on_tof)
        self._bridge.sig_box_colors.connect(self._on_box_colors)
        # Motion Jog tab: live Cartesian (TF) + Joint (/joint_states) readout.
        self._build_velocity_tab()   # _vel_spin must exist before HOME/INIT
        self._build_motion_jog()

        # periodic status refresh (this-tab / external / stopped).
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._refresh_status)
        self._timer.start(int(self._cfg["status_timer_ms"]))
        self._refresh_status()

        self._set_status(f"config: {self._config_path or '(기본값)'}")

        # Pipe Health tab (dynamic table, built in code) — appended last.
        self._pipe_tab = PipeHealthTab(self._bridge, parent=self)
        self.tabs.addTab(self._pipe_tab, "Pipe Health")

        # Project rule: RViz must always spawn alongside the UI.
        if with_rviz:
            self._auto_start_rviz()

        # Start the video stream if the initial tab (default: Pick and Place)
        # is one that shows it (currentChanged does not fire for the start tab).
        self._on_tab_changed(self.tabs.currentIndex())

    # ------------------------------------------------------------------ wiring
    def _style_buttons(self) -> None:
        for p in self._presets.values():
            start, stop, _ = p["widgets"]
            getattr(self, start).setStyleSheet(_BTN_GREEN_QSS)
            getattr(self, stop).setStyleSheet(_BTN_RED_QSS)
        self.btnRun.setStyleSheet(_BTN_GREEN_QSS)
        self.btnCancel.setStyleSheet(_BTN_RED_QSS)

    def _wire_launch_rows(self) -> None:
        for key, p in self._presets.items():
            start_name, stop_name, _ = p["widgets"]
            getattr(self, start_name).clicked.connect(
                lambda _c, k=key: self._start(k))
            getattr(self, stop_name).clicked.connect(
                lambda _c, k=key: self._stop(k))
        self.btnStopAll.clicked.connect(self._stop_all)
        self.btnRefresh.clicked.connect(self._refresh_status)

    def _wire_action_controls(self) -> None:
        self.btnRun.clicked.connect(self._run)
        self.btnCancel.clicked.connect(self._cancel)
        self.btnClearLog.clicked.connect(self._clear_log)

    def _apply_goal_defaults(self) -> None:
        gd = self._cfg["goal_defaults"]
        self.spnZ.setValue(float(gd["z"]))
        self.spnRoll.setValue(float(gd["roll_deg"]))
        self.spnPitch.setValue(float(gd["pitch_deg"]))
        self.spnYaw.setValue(float(gd["yaw_deg"]))
        idx = self.cmbArms.findText(str(gd["arms"]))
        if idx >= 0:
            self.cmbArms.setCurrentIndex(idx)

    # --------------------------------------------------------------- detection
    @staticmethod
    def _running_cmdlines() -> list:
        """Snapshot every process command line via one /proc read (no
        subprocess). A substring test reproduces `pgrep -f`'s literal match."""
        cmds = []
        try:
            for pid in os.listdir("/proc"):
                if not pid.isdigit():
                    continue
                try:
                    with open(f"/proc/{pid}/cmdline", "rb") as f:
                        raw = f.read()
                except OSError:
                    continue
                if raw:
                    cmds.append(
                        raw.replace(b"\x00", b" ").decode("utf-8", "replace"))
        except OSError:
            pass
        return cmds

    def _detect_external(self, preset: dict, cmdlines: list) -> bool:
        for node in preset.get("nodes", []):
            if any(node == n or n.endswith(node) for n in self._node_cache):
                return True
        for pat in preset.get("procs", []):
            if any(pat in c for c in cmdlines):
                return True
        return False

    # ------------------------------------------------------------ start / stop
    def _start(self, key: str) -> None:
        proc = self._procs[key]
        preset = self._presets[key]
        label = self._label(key)
        if proc.running:
            self._set_status(f"{label}: 이미 실행 중(this UI).")
            return

        self._node_cache = self._bridge.live_node_names()
        if self._detect_external(preset, self._running_cmdlines()):
            ans = QMessageBox.question(
                self, "이미 실행 중",
                f"'{label}' 와(과) 동일한 launch/노드가 이미 실행 중인 것으로 "
                f"감지되었습니다 (다른 터미널/UI).\n\n"
                f"그래도 새로 실행하시겠습니까? (중복 실행은 컨트롤러 충돌을 일으킬 수 있습니다)",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if ans != QMessageBox.Yes:
                self._set_status(f"{label}: 실행 취소 (중복 감지).")
                return

        if preset.get("confirm"):
            ans = QMessageBox.warning(
                self, "실행 확인",
                f"'{label}' 을(를) 실행합니다.\n"
                f"실제 하드웨어/모터가 동작할 수 있습니다. 안전을 확인했습니까?\n\n"
                f"{' '.join(preset['cmd'])}",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if ans != QMessageBox.Yes:
                return

        if proc.start(preset["cmd"]):
            self._started_keys.add(key)
            self._set_status(f"Started: {label} → {proc.log_path}")
        else:
            self._set_status(f"{label}: 이미 실행 중.")
        self._refresh_status()

    def _stop(self, key: str) -> None:
        if key == "gravity":
            # Zero the feed-forward FIRST: the forward_effort_controllers persist
            # after the node is killed and would hold the last torque. Setting
            # enable_compensation=false makes the node publish zeros, then stop.
            try:
                subprocess.run(
                    ["ros2", "param", "set", "/gravity_comp_node",
                     "enable_compensation", "false"],
                    capture_output=True, timeout=5)
                time.sleep(0.3)
            except Exception:
                pass
        proc = self._procs[key]
        proc.stop()
        # Sweep by signature so orphans (parent already exited) are also reaped.
        preset = self._presets[key]
        for pat in preset.get("procs", []) + preset.get("sweep", []):
            self._kill_pattern(pat)
        self._set_status(f"Stopped: {self._label(key)}")
        self._refresh_status()

    def _auto_start_rviz(self) -> None:
        preset = self._presets["rviz"]
        self._node_cache = self._bridge.live_node_names()
        if self._detect_external(preset, self._running_cmdlines()):
            self._set_status("RViz: 외부에서 이미 실행 중 — 자동 spawn 생략.")
            return
        if not _PTP_RVIZ:
            self._set_status("RViz config 를 찾지 못함 — RViz 자동 spawn 생략.")
            return
        if self._procs["rviz"].start(preset["cmd"]):
            self._started_keys.add("rviz")
            self._set_status(f"RViz 자동 spawn → {_PTP_RVIZ}")
        self._refresh_status()

    def _stop_all(self) -> None:
        for proc in self._procs.values():
            proc.stop()
        for key in list(self._started_keys):
            for pat in (self._presets[key].get("procs", [])
                        + self._presets[key].get("sweep", [])):
                self._kill_pattern(pat)
        self._set_status("Stop All (this UI) 실행됨.")
        self._refresh_status()

    # --------------------------------------------------------------- status UI
    def _refresh_status(self) -> None:
        self._node_cache = self._bridge.live_node_names()
        cmdlines = self._running_cmdlines()
        for key, p in self._presets.items():
            _, _, lbl_name = p["widgets"]
            lbl = getattr(self, lbl_name)
            if self._procs[key].running:
                lbl.setText("● Running (this UI)")
                lbl.setStyleSheet("color:#080; font-weight:bold;")
            elif self._detect_external(p, cmdlines):
                lbl.setText("● Running (external)")
                lbl.setStyleSheet("color:#d80;")
            else:
                lbl.setText("● Stopped")
                lbl.setStyleSheet("color:gray;")

    # ------------------------------------------------------------- action ctrl
    def _run(self) -> None:
        ok = self._bridge.run(
            z=self.spnZ.value(),
            roll_deg=self.spnRoll.value(),
            pitch_deg=self.spnPitch.value(),
            yaw_deg=self.spnYaw.value(),
            arms=self.cmbArms.currentText(),
        )
        if ok:
            self.prgPhase.setValue(0)
            self.lblPhase.setText("phase: 전송 중…")
            self._append_log(
                f"$ AlignToBoxes z={self.spnZ.value():.3f} "
                f"R={self.spnRoll.value():.0f} P={self.spnPitch.value():.0f} "
                f"Y={self.spnYaw.value():.0f} arms={self.cmbArms.currentText()}")
            self._set_status("AlignToBoxes 전송됨.")

    def _cancel(self) -> None:
        self._bridge.cancel()
        self._set_status("Cancel 요청.")

    # ---------------------------------------------------------- bridge -> view
    def _on_feedback(self, fb: dict) -> None:
        pct = int(max(0.0, min(1.0, fb.get("progress", 0.0))) * 100)
        self.prgPhase.setValue(pct)
        self.lblPhase.setText(f"phase: {fb.get('phase', '?')}  ({pct}%)")

    def _on_result(self, r: dict) -> None:
        ok = r.get("success")
        self._append_log(
            f"[RESULT] success={ok}  message={r.get('message', '')}")
        if r.get("assignments_json"):
            self._append_log(f"  assignments: {r['assignments_json']}")
        if r.get("detections_json"):
            self._append_log(f"  detections : {r['detections_json']}")
        if ok:
            self.prgPhase.setValue(100)
            self.lblPhase.setText("phase: ✅ done")
        else:
            self.lblPhase.setText("phase: ❌ failed / canceled")

    def _on_error(self, msg: str) -> None:
        self._append_log(f"[ERROR] {msg}")
        self._set_status(msg)

    def _on_goal_state(self, state: str) -> None:
        if state == "accepted":
            self._set_status("골 수락됨 — 실행 중.")
        elif state == "rejected":
            self._set_status("골 거부됨.")
        elif state == "canceled":
            self._set_status("골 취소 요청 전송됨.")
        elif state == "done":
            self._set_status("골 종료.")

    # --------------------------------------------------- Detection-tab video
    def _on_tab_changed(self, idx: int) -> None:
        # Stream camera/cloud while EITHER the Detection or Pick-and-Place tab
        # (both show the video) is visible.
        if self.tabs.tabText(idx) in ("Detection", "Pick and Place"):
            self._bridge.images_start()
        else:
            self._bridge.images_stop()

    def _on_overlay_toggled(self, checked: bool) -> None:
        # Reset so turning the overlay on shows live colour until the next YOLO
        # result arrives, then holds that result.
        self._overlay_on = checked
        self._has_annot = False

    def _on_img_cam(self, bgr, is_annot) -> None:
        # Overlay checkbox decides the left pane: ON → show the YOLO-annotated
        # frame and HOLD it (the result stays until the next detection); live
        # colour shows only until the first overlay arrives. OFF → live colour.
        # Rendered to both the Detection and Pick-and-Place camera panes.
        if is_annot:
            if self._overlay_on:
                self._has_annot = True
                self._render_cam(bgr)
        elif (not self._overlay_on) or (not self._has_annot):
            self._render_cam(bgr)

    def _render_cam(self, bgr) -> None:
        for lbl in (self.imgCam, self.imgCamPnp):
            self._render_to_label(lbl, bgr)

    def _detect_once(self) -> None:
        """Trigger one DetectBox goal so /yolov8_node/image_annotated (the YOLO
        overlay) updates. Subprocess + 3d_detect_ws source avoids an in-process
        dependency on that action type. Auto-enables the overlay checkbox."""
        setup = self._cfg["detect_ws_setup"]
        cmd = (f"source {setup} && ros2 action send_goal /yolov8_node/detect "
               "yolov8_detection_msgs/action/DetectBox "
               "'{prompts: \"\", confidence: 0.5, publish_annotated: true}'")
        try:
            subprocess.Popen(["bash", "-c", cmd], stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL, start_new_session=True)
            self.chkOverlay.setChecked(True)
            self._set_status("검출요청 전송 — 잠시 후 좌측에 YOLO 오버레이 표시.")
        except Exception as e:
            self._set_status(f"검출요청 실패: {e}")

    def _on_tof(self, meters: float) -> None:
        self.lblLaser.setText(f"{meters:.3f} m" if meters > 0 else "--- m")

    def _on_box_colors(self, colors) -> None:
        self.lblBoxColors.setText(", ".join(colors) if colors else "---")

    def _setup_cloud_view(self) -> None:
        """Embed pyqtgraph OpenGL point-cloud views in BOTH right panes (Detection
        and Pick-and-Place). GL/pyqtgraph unavailable (e.g. headless) → a label per
        pane so the rest of the UI still runs."""
        self._cloud_scatters = []
        try:
            import pyqtgraph.opengl as gl
        except Exception as e:
            for h in (self.cloudHolder, self.cloudHolderPnp):
                h.layout().addWidget(QLabel(f"3D 뷰 사용 불가: {e}"))
            return
        for holder in (self.cloudHolder, self.cloudHolderPnp):
            try:
                view = gl.GLViewWidget()
                view.setCameraPosition(distance=1.5, elevation=-65, azimuth=90)
                grid = gl.GLGridItem()
                grid.setSize(2.0, 2.0)
                grid.setSpacing(0.1, 0.1)
                view.addItem(grid)
                scatter = gl.GLScatterPlotItem(size=2.0, pxMode=True)
                view.addItem(scatter)
                holder.layout().addWidget(view)
                self._cloud_scatters.append(scatter)
            except Exception as e:
                holder.layout().addWidget(QLabel(f"3D 뷰 사용 불가: {e}"))

    def _on_cloud(self, xyz, rgba) -> None:
        for scatter in self._cloud_scatters:
            scatter.setData(pos=xyz, color=rgba, size=2.0)

    @staticmethod
    def _render_to_label(label, bgr) -> None:
        """BGR ndarray → QPixmap scaled into `label` (keep aspect).

        numpy channel-swap (no cv2 on the GUI thread → avoids the cv2↔PyQt5
        xcb-plugin clash; the bridge spin thread owns cv2 decode/colormap)."""
        if bgr is None or bgr.ndim != 3 or bgr.shape[2] != 3:
            return
        rgb = np.ascontiguousarray(bgr[:, :, ::-1])   # BGR → RGB
        h, w = rgb.shape[:2]
        qimg = QImage(rgb.data, w, h, 3 * w, QImage.Format_RGB888).copy()
        label.setPixmap(QPixmap.fromImage(qimg).scaled(
            label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    # ------------------------------------------------ Motion Jog tab readout
    _RO_QSS = ("background:#f5f5f5; border:1px solid #ddd; border-radius:3px; "
               "padding:1px 6px; font-family:monospace;")

    def _build_motion_jog(self) -> None:
        """Populate the (empty) Motion Jog tab with live Cartesian (TF) and Joint
        (/joint_states) readouts, each in its own group box (Cartesian on top)."""
        self._cart_lbl = {"left": {}, "right": {}}
        self._joint_lbl = {"left": {}, "right": {}}
        root = QVBoxLayout(self.tabMotionJog)

        # HOME / INIT — move both arms on press; speed from the Joint Velocity tab.
        btn_row = QHBoxLayout()
        self._btn_home = QPushButton("HOME (0°)")
        self._btn_home.setMinimumHeight(34)
        self._btn_home.setStyleSheet(_BTN_GREEN_QSS)
        self._btn_home.clicked.connect(self._go_home)
        self._btn_init = QPushButton("INIT (초기자세)")
        self._btn_init.setMinimumHeight(34)
        self._btn_init.setStyleSheet(_BTN_GREEN_QSS)
        self._btn_init.clicked.connect(self._go_init)
        btn_row.addWidget(self._btn_home)
        btn_row.addWidget(self._btn_init)
        btn_row.addStretch()
        root.addLayout(btn_row)

        grp_c = QGroupBox("Cartesian 좌표 (TCP · base 기준, 실측 TF)")
        gc = QGridLayout(grp_c)
        for c, txt in ((0, "<b>축</b>"), (1, "<b>Left</b>"), (2, "<b>Right</b>")):
            gc.addWidget(QLabel(txt), 0, c)
        rows = [("x (m)", "x"), ("y (m)", "y"), ("z (m)", "z"),
                ("roll (°)", "roll"), ("pitch (°)", "pitch"), ("yaw (°)", "yaw")]
        for r, (lab, key) in enumerate(rows, start=1):
            gc.addWidget(QLabel(lab), r, 0)
            for col, arm in ((1, "left"), (2, "right")):
                v = QLabel("---")
                v.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
                v.setStyleSheet(self._RO_QSS)
                self._cart_lbl[arm][key] = v
                gc.addWidget(v, r, col)
        gc.setColumnStretch(1, 1)
        gc.setColumnStretch(2, 1)
        root.addWidget(grp_c)

        grp_j = QGroupBox("Joint 좌표 (관절각 · /joint_states)")
        gj = QGridLayout(grp_j)
        for c, txt in ((0, "<b>관절</b>"), (1, "<b>Left (°)</b>"), (2, "<b>Right (°)</b>")):
            gj.addWidget(QLabel(txt), 0, c)
        for j in range(1, 8):
            gj.addWidget(QLabel(f"J{j}"), j, 0)
            for col, arm in ((1, "left"), (2, "right")):
                v = QLabel("---")
                v.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
                v.setStyleSheet(self._RO_QSS)
                self._joint_lbl[arm][f"openarmx_{arm}_joint{j}"] = v
                gj.addWidget(v, j, col)
        gj.setColumnStretch(1, 1)
        gj.setColumnStretch(2, 1)
        root.addWidget(grp_j)
        root.addStretch(1)

        self._bridge.sig_joint_state.connect(self._on_joint_readout)
        self._mj_timer = QTimer(self)
        self._mj_timer.timeout.connect(self._refresh_cartesian)
        self._mj_timer.start(500)

    def _on_joint_readout(self, positions: dict) -> None:
        for arm in ("left", "right"):
            for name, lb in self._joint_lbl[arm].items():
                if name in positions:
                    lb.setText(f"{positions[name]:+.1f}")

    def _refresh_cartesian(self) -> None:
        # Only when the Motion Jog tab is current (skip wasted TF lookups).
        if self.tabs.tabText(self.tabs.currentIndex()) != "Motion Jog":
            return
        for arm in ("left", "right"):
            cl = self._cart_lbl[arm]
            pose = self._bridge.get_ee_pose(arm)
            if not pose:
                for lb in cl.values():
                    lb.setText("---")
                continue
            cl["x"].setText(f"{pose['x']:+.4f}")
            cl["y"].setText(f"{pose['y']:+.4f}")
            cl["z"].setText(f"{pose['z']:+.4f}")
            cl["roll"].setText(f"{math.degrees(pose['roll']):+.1f}")
            cl["pitch"].setText(f"{math.degrees(pose['pitch']):+.1f}")
            cl["yaw"].setText(f"{math.degrees(pose['yaw']):+.1f}")

    # ------------------------------------------- Joint Velocity tab + HOME/INIT
    def _build_velocity_tab(self) -> None:
        """New 'Joint Velocity' tab: per-joint max angular velocity (deg/s, shared
        L/R) used to time the HOME/INIT moves (duration = max |Δ|/vel)."""
        self._vel_spin = {}
        tab = QWidget()
        root = QVBoxLayout(tab)
        grp = QGroupBox("조인트 각속도 (deg/s) — HOME/INIT 이동 속도")
        g = QGridLayout(grp)
        g.addWidget(QLabel("<b>관절</b>"), 0, 0)
        g.addWidget(QLabel("<b>각속도</b>"), 0, 1)
        for j in range(1, 8):
            g.addWidget(QLabel(f"J{j}"), j, 0)
            s = QDoubleSpinBox()
            s.setRange(1.0, 180.0)
            s.setDecimals(0)
            s.setSingleStep(5.0)
            s.setValue(30.0)
            s.setSuffix(" °/s")
            self._vel_spin[j] = s
            g.addWidget(s, j, 1)
        g.setColumnStretch(1, 1)
        root.addWidget(grp)
        allrow = QHBoxLayout()
        self._spn_vel_all = QDoubleSpinBox()
        self._spn_vel_all.setRange(1.0, 180.0)
        self._spn_vel_all.setDecimals(0)
        self._spn_vel_all.setSingleStep(5.0)
        self._spn_vel_all.setValue(30.0)
        self._spn_vel_all.setSuffix(" °/s")
        btn_all = QPushButton("전체 적용")
        btn_all.clicked.connect(self._apply_vel_all)
        allrow.addWidget(QLabel("전체:"))
        allrow.addWidget(self._spn_vel_all)
        allrow.addWidget(btn_all)
        allrow.addStretch()
        root.addLayout(allrow)
        root.addStretch(1)
        self.tabs.addTab(tab, "Joint Velocity")

    def _apply_vel_all(self) -> None:
        v = self._spn_vel_all.value()
        for s in self._vel_spin.values():
            s.setValue(v)

    def _go_home(self) -> None:
        self._go_pose(_HOME_DEG, "HOME")

    def _go_init(self) -> None:
        self._go_pose(_INIT_DEG, "INIT")

    def _go_pose(self, target_deg: dict, label: str) -> None:
        cur = self._bridge.latest_joint_deg()
        if not cur:
            self._set_status(f"{label}: /joint_states 미수신 — L1 컨트롤러 확인.")
            return
        dur = 1.0
        for name, tgt in target_deg.items():
            j = int(name[-1])                    # ...joint{i} → i (1..7)
            vel = max(1.0, self._vel_spin[j].value())
            dur = max(dur, abs(tgt - cur.get(name, tgt)) / vel)
        self._bridge.send_arm_trajectory(target_deg, dur)
        self._set_status(f"{label} 이동 전송 (~{dur:.1f}s).")

    # ---------------------------------------------------------------- helpers
    @staticmethod
    def _label(key: str) -> str:
        return {
            "hw_sil": "하드웨어 SIL", "hw_hw": "하드웨어 실로봇",
            "ctrl": "컨트롤러 스폰", "cam": "D435 카메라",
            "det": "원격검출+인지", "backend": "ptp 정렬 백엔드", "rviz": "RViz",
            "tof": "TOF 수신", "gravity": "중력보상", "ee_markers": "ee_leader 마커",
        }.get(key, key)

    def _append_log(self, text: str) -> None:
        ts = datetime.now().strftime("%H:%M:%S")
        self.txtLog.append(f"{ts}  {text}")
        sb = self.txtLog.verticalScrollBar()
        sb.setValue(sb.maximum())

    def _clear_log(self) -> None:
        self.txtLog.clear()
        self._set_status("로그를 지웠습니다.")

    def _set_status(self, text: str) -> None:
        ts = datetime.now().strftime("%H:%M:%S")
        self.lblStatus.setText(f"{ts}  {text}")

    def _kill_pattern(self, pattern: str) -> None:
        """Kill any process group whose cmdline matches `pattern` (literal).
        Reaps orphans whose ManagedProcess parent already exited."""
        lit = re.escape(pattern)
        try:
            r = subprocess.run(["pgrep", "-f", lit],
                               capture_output=True, text=True, timeout=3)
            pids = [int(x) for x in r.stdout.split() if x.strip().isdigit()]
        except Exception:
            return
        me = os.getpid()
        pids = [p for p in pids if p != me]
        if not pids:
            return
        pgids = set()
        for p in pids:
            try:
                pgids.add(os.getpgid(p))
            except (ProcessLookupError, PermissionError):
                pass
        for pg in pgids:
            try:
                os.killpg(pg, signal.SIGINT)
            except (ProcessLookupError, PermissionError):
                pass
        time.sleep(0.7)
        for pg in pgids:
            try:
                os.killpg(pg, signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass
        for p in pids:
            try:
                os.kill(p, signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass

    # ---------------------------------------------------------------- shutdown
    def closeEvent(self, event) -> None:
        # Park both arms to HOME (0°) and wait for arrival BEFORE tearing anything
        # down: killing the controllers/hardware first cuts MIT motor torque and
        # the arm drops under gravity. park_arms_home skips (returns False) when no
        # JTC is listening (no robot / SIL down). This runs the bridge while it is
        # still alive, so it must precede bridge.shutdown(). Covers the window
        # close AND Ctrl+C (the entry script's SIGINT handler calls win.close()).
        try:
            self._set_status("종료 전 HOME(0°) 복귀 중…")
            QApplication.processEvents()
            self._bridge.park_arms_home(tick=QApplication.processEvents)
        except Exception:
            pass
        # Fast shutdown. Sequential proc.stop() waited up to several seconds PER
        # process (SIGINT grace), so closing with a few launches up felt slow.
        # Instead: stop the ROS bridge, SIGINT every group we started AT ONCE
        # (one shared grace), then quick-SIGKILL any straggler.
        self._timer.stop()
        self._mj_timer.stop()
        self._pipe_tab.shutdown()
        self._bridge.shutdown()
        running = [p for p in self._procs.values() if p.running]
        for p in running:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGINT)
            except (ProcessLookupError, PermissionError):
                pass
        if running:
            time.sleep(1.0)                    # single shared grace, not per-proc
        for p in self._procs.values():
            p.stop(sigint_timeout=0.2)         # quick SIGKILL of any straggler
        # Reap orphans (pgrep returns immediately when nothing matches).
        for key in list(self._started_keys):
            for pat in (self._presets[key].get("procs", [])
                        + self._presets[key].get("sweep", [])):
                self._kill_pattern(pat)
        event.accept()
