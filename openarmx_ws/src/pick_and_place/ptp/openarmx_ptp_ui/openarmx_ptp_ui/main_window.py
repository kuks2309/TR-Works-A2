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
import socket
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


# Motion Jog HOME / INIT joint targets (deg).
# HOME = all 0°. INIT = the RAISED hand-guide pose registered 2026-06-07 for
# table-collision avoidance (SSOT: experiments/ptp_pick_seq_v2 INIT_DEG). The old
# [±50,0,0,100,0,0,±50] INIT sat too low and the gripper clipped boxes. Mirror
# rule: j4 (elbow) equal, j1/2/3/5/6/7 sign-flipped (s=-1,-1,-1,+1,-1,-1,-1).
_HOME_DEG = {f"openarmx_{s}_joint{i}": 0.0
             for s in ("left", "right") for i in range(1, 8)}
_INIT_DEG = {
    "openarmx_left_joint1": 42.8, "openarmx_left_joint2": -46.3,
    "openarmx_left_joint3": 14.3, "openarmx_left_joint4": 106.5,
    "openarmx_left_joint5": -41.7, "openarmx_left_joint6": 23.4,
    "openarmx_left_joint7": 42.1,
    "openarmx_right_joint1": -42.8, "openarmx_right_joint2": 46.3,
    "openarmx_right_joint3": -14.3, "openarmx_right_joint4": 106.5,
    "openarmx_right_joint5": 41.7, "openarmx_right_joint6": -23.4,
    "openarmx_right_joint7": -42.1,
}


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
        # 노드 기본 파라미터가 openarmx_tof_driver 의 tof_driver.yaml 과 동일해
        # 단독 `ros2 run` 으로 충분(검출/벽 노드 없이 수신만).
        "tof": {
            "cmd": ["ros2", "run", "openarmx_tof_driver",
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
        # 큰 박스(place) 검출: cloud+TOF → 벽(place box) pose/색 → /place_box/info.
        # Pick and Place 탭 센서 그룹의 "큰 박스 위치/유무/색" 소스.
        "place_box": {
            "cmd": ["ros2", "launch", "place_box_detection",
                    "place_box_detection.launch.py"],
            "confirm": False,
            "nodes": ["/place_box_detection_node"],
            "procs": ["place_box_detection.launch", "place_box_detection_node"],
            "sweep": ["place_box_detection_node"],
            "widgets": ("btnPlaceBoxStart", "btnPlaceBoxStop", "lblPlaceBox"),
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
        # Latest detected container (big place-box) colour from /place_box/info,
        # used by Auto mode when "컨테이너 색" is selected. None = none/unknown.
        self._container_color = None
        self.chkOverlay.toggled.connect(self._on_overlay_toggled)
        self.btnDetectOnce.clicked.connect(self._detect_once)
        # Detection-tab sensor/detect readout (laser distance + box colours).
        self._bridge.sig_tof.connect(self._on_tof)
        self._bridge.sig_box_colors.connect(self._on_box_colors)
        self._bridge.sig_place_box.connect(self._on_place_box)
        # Motion Jog tab: live Cartesian (TF) + Joint (/joint_states) readout.
        self._build_velocity_tab()   # _vel_spin must exist before HOME/INIT
        self._build_motion_jog()

        # ---- Pick&Place 탭: 수동/자동 픽 (상주 픽 서버 ptp_pick_left/right 호출) ----
        from openarmx_ptp_ui.ptp_pick_bridge import PtpPickBridge
        self._pick_bridge = PtpPickBridge(self)
        self._pick_bridge.sig_status.connect(self._on_pick_status)
        self.btnPickOnce.clicked.connect(self._manual_pick)
        # 자동 전용 색 선택 (수동과 분리). 기본 "전체(모든 색)" = 색 무관.
        from PyQt5.QtWidgets import QCheckBox, QComboBox, QLabel, QHBoxLayout, QWidget
        _gl = self.grpAuto.layout()
        self.comboColorAuto = QComboBox(self.grpAuto)
        self.comboColorAuto.addItems(["전체(모든 색)", "빨강", "노랑", "녹색", "파랑", "주황",
                                      "컨테이너 색"])
        self.comboColorAuto.setToolTip(
            "자동 picking 대상 색 (전체=색 무관 / 컨테이너 색=검출된 큰 박스 색과 같은 색만 집음)")
        _arow = QWidget(self.grpAuto)
        _al = QHBoxLayout(_arow)
        _al.setContentsMargins(0, 0, 0, 0)
        _al.addWidget(QLabel("색:"))
        _al.addWidget(self.comboColorAuto)
        if _gl is not None:
            _gl.addWidget(_arow)
        # 자동 시작은 '자동 전용' 콤보 사용(수동 comboColorPnp 와 분리).
        # "컨테이너 색" 선택 시 검출된 큰 박스 색을 픽 색으로 사용(_auto_start).
        self.btnAutoStart.clicked.connect(self._auto_start)
        self.btnAutoStop.clicked.connect(self._auto_stop)
        # 양팔 동시 구동 허용 체크박스 — 기본 OFF=단일팔(충돌방지 뮤텍스), ON=동시 허용
        self.chkDualArm = QCheckBox("양팔 동시 구동 허용", self.grpAuto)
        self.chkDualArm.setToolTip("체크: 양팔 동시 동작 / 해제: 한 번에 한 팔만(충돌방지)")
        if _gl is not None:
            _gl.addWidget(self.chkDualArm)
        self.chkDualArm.toggled.connect(self._pick_bridge.set_dual_arm)

        # [동시구동 차단·SSOT] pick&place 정본 = resident 픽 경로(이 탭). AlignToBoxes(C++)
        # 는 박스 위 hover 정렬만 하는 디버그 경로로 격하. 두 경로가 같은 controller_manager 를
        # 교차 토글하면 충돌하므로 UI 에서 상호배타(한쪽 활성 시 다른쪽 버튼 비활성).
        self._cpp_goal_active = False        # AlignToBoxes(hover) 골 진행 중
        self._cpp_goal_t0 = 0.0              # hover 골 전송 시각(서버 사망 시 래치 타임아웃용)
        self._resident_auto_active = False   # resident 자동 픽 진행 중
        self.btnRun.setText("Hover 정렬 (디버그)")
        self.btnRun.setToolTip(
            "C++ AlignToBoxes — 박스 위 hover 정렬만(파지·놓기 없음), 디버그/테스트용.\n"
            "실제 pick&place 는 이 탭의 수동/자동(상주 픽 서버)을 사용하세요.")
        # 픽 서버(좌/우 상주) — 한 버튼으로 동시 기동/정지 (ManagedProcess)
        _R = "/home/openarmx/TR-Works/kkw/China"
        self._pick_srv_proc = ManagedProcess("ptp_ui_pickservers")
        # ptp pick&place 정본은 정식 패키지 openarmx_ptp_pick(experiments/ 에서 승격).
        # box_detect_loop 가 yolov8_detection_msgs(별도 워크스페이스 3d_detect_ws)를 쓰므로 먼저
        # source 한 뒤, 패키지 launch 로 검출루프+컨테이너게이트+좌/우 resident 를 한 번에 띄운다.
        # (구 experiments/ 절대경로 bash 그룹 대체 — 노드명 ptp_pick_left/right 는 launch 가 remap.)
        self._pick_srv_cmd = ["bash", "-c",
            "source /opt/ros/humble/setup.bash && "
            f"source {_R}/openarmx_ws/install/setup.bash && "
            f"source {_R}/3d_detect_ws/install/setup.bash && "
            "exec ros2 launch openarmx_ptp_pick ptp_pick_servers.launch.py"]
        # 픽 서버 기동/정지는 Launch 탭 '픽 서버' row 에서만 (Pick&Place 탭 중복 버튼 제거).
        self.btnPickSrvLaunchStart.clicked.connect(self._start_pick_servers)
        self.btnPickSrvLaunchStop.clicked.connect(self._stop_pick_servers)

        # periodic status refresh (this-tab / external / stopped).
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._refresh_status)
        self._timer.start(int(self._cfg["status_timer_ms"]))
        self._refresh_status()

        self._set_status(f"config: {self._config_path or '(기본값)'}")

        # Pipe Health tab (dynamic table, built in code) — appended last.
        self._pipe_tab = PipeHealthTab(self._bridge, parent=self)
        self.tabs.addTab(self._pipe_tab, "Pipe Health")

        # Network tab (remote-control settings) — empty placeholder, appended last.
        self._build_network_tab()

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
        # 픽 서버 버튼(Launch 탭 row)은 preset 이 아니라 _pick_srv_proc 재사용 →
        # 여기서 직접 초록/빨강 스타일 적용(다른 행과 동일하게).
        self.btnPickSrvLaunchStart.setStyleSheet(_BTN_GREEN_QSS)
        self.btnPickSrvLaunchStop.setStyleSheet(_BTN_RED_QSS)

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

    def _start_pick_servers(self) -> None:
        if self._pick_srv_proc.running:
            self.lblPnpStatus.setText("픽 서버 상태: 이미 실행 중")
            return
        ok = self._pick_srv_proc.start(self._pick_srv_cmd)
        self.lblPnpStatus.setText(
            "픽 서버 상태: 좌/우 기동 중… (모델 로드 ~4s, 로봇 스택 필요)" if ok
            else "픽 서버 상태: 기동 실패")

    def _stop_pick_servers(self) -> None:
        self._pick_srv_proc.stop()
        self._kill_pattern("ptp_pick_resident")
        self._kill_pattern("box_detect_loop")
        self._kill_pattern("container_pick_gate")
        self.lblPnpStatus.setText("픽 서버 상태: 정지")

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
        # 픽 서버(preset 아님, _pick_srv_proc) 도 함께 정지 + 고아 정리.
        self._pick_srv_proc.stop()
        for pat in ("ptp_pick_servers.launch", "ptp_pick_resident",
                    "box_detect_loop", "container_pick_gate"):
            self._kill_pattern(pat)
        self._set_status("Stop All (this UI) 실행됨.")
        self._refresh_status()

    # --------------------------------------------------------------- status UI
    def _refresh_status(self) -> None:
        # [수정] Hover 골이 비정상 종료(서버 사망 등)해도 _cpp_goal_active 가 영구 고착되지 않게
        # 타임아웃 화해 — 일정 시간 지나면 래치 해제(픽 버튼 재활성). hover 는 수초면 끝난다.
        if self._cpp_goal_active and (time.monotonic() - self._cpp_goal_t0) > 60.0:
            self._cpp_goal_active = False
            self._update_path_exclusion()
            self._set_status("Hover 골 타임아웃(60s) — 상호배타 해제")
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
        # 픽 서버(좌/우 상주, Launch 탭 row) 상태 — _pick_srv_proc(이 UI)/노드그래프(외부) 감지.
        if self._pick_srv_proc.running:
            self.lblPickSrvLaunch.setText("● Running (this UI)")
            self.lblPickSrvLaunch.setStyleSheet("color:#080; font-weight:bold;")
        elif any(("ptp_pick_left" in n or "ptp_pick_right" in n)
                 for n in self._node_cache):
            self.lblPickSrvLaunch.setText("● Running (external)")
            self.lblPickSrvLaunch.setStyleSheet("color:#d80;")
        else:
            self.lblPickSrvLaunch.setText("● Stopped")
            self.lblPickSrvLaunch.setStyleSheet("color:gray;")

    # ------------------------------------------------------------- action ctrl
    def _run(self) -> None:
        if self._resident_auto_active:
            self._set_status("자동 픽 진행 중 — 자동 정지 후 Hover 정렬(디버그)을 쓰세요.")
            return
        self.btnRun.setEnabled(False)        # 더블클릭 재진입 방지(완료/거부/실패 시 복구)
        self._cpp_goal_t0 = time.monotonic()
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
        else:
            self._update_path_exclusion()    # 전송 실패(서버 부재 등) → btnRun 복구

    def _cancel(self) -> None:
        self._bridge.cancel()
        self._set_status("Cancel 요청.")

    # ----------------------------------------------- 두 경로 동시구동 차단(상호배타)
    def _manual_pick(self) -> None:
        """수동 픽 1회 — C++ Hover(디버그) 경로 진행 중이면 거부(동시구동 차단)."""
        if self._cpp_goal_active:
            self._set_status("Hover 정렬(디버그) 진행 중 — Cancel 후 픽을 쓰세요.")
            return
        self._pick_bridge.manual_pick(
            self.comboColorPnp.currentText(),
            {"좌": "left", "우": "right"}.get(self.comboSidePnp.currentText(), "both"))

    def _auto_stop(self) -> None:
        """자동 픽 정지 + 상호배타 해제(Hover 버튼 재활성)."""
        self._pick_bridge.auto_stop()
        self._resident_auto_active = False
        self._update_path_exclusion()

    def _on_pick_status(self, s: str) -> None:
        """resident 픽 서버 상태 표시 + 상호배타 래치 화해.
        resident watchdog 자가정지("AUTO 자동정지...")면 UI 가 모르고 btnRun 을 영구
        비활성으로 두지 않도록 _resident_auto_active 를 해제(btnRun 재활성)."""
        self.lblPnpStatus.setText(f"픽 서버 상태: {s}")
        if "자동정지" in s and self._resident_auto_active:
            self._resident_auto_active = False
            self._update_path_exclusion()

    def _update_path_exclusion(self) -> None:
        """C++ Hover(디버그) 경로와 resident 픽 경로의 UI 상호배타.
        같은 controller_manager 를 교차 토글하면 충돌하므로 한쪽 활성 시 다른쪽 비활성."""
        cpp = self._cpp_goal_active
        res = self._resident_auto_active
        self.btnPickOnce.setEnabled(not cpp)
        self.btnAutoStart.setEnabled(not cpp)
        self.btnRun.setEnabled(not res and not cpp)   # cpp 활성 중엔 btnRun 도 비활성(더블 발사 방지)

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
            self._cpp_goal_active = True
            self._update_path_exclusion()
            self._set_status("골 수락됨 — 실행 중.")
        elif state == "rejected":
            self._cpp_goal_active = False
            self._update_path_exclusion()
            self._set_status("골 거부됨.")
        elif state == "canceled":
            self._set_status("골 취소 요청 전송됨.")
        elif state == "done":
            self._cpp_goal_active = False
            self._update_path_exclusion()
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
        txt = f"{meters:.3f} m" if meters > 0 else "--- m"
        self.lblLaser.setText(txt)        # Detection tab
        self.lblTofPnp.setText(txt)       # Pick and Place tab

    def _on_box_colors(self, colors) -> None:
        self.lblBoxColors.setText(", ".join(colors) if colors else "---")

    def _on_place_box(self, d: dict) -> None:
        # Big (place) box from /place_box/info. 실제 스키마: 박스=d["wall"](centroid/ok/
        # front_distance), 색=d["color"](name/confidence). 구버전 최상위 키는 폴백으로 유지.
        wall = d.get("wall") or {}
        color = d.get("color") or {}
        c = wall.get("centroid") or d.get("centroid")
        present = wall.get("ok", d.get("ok"))
        if present and c and len(c) == 3:
            fd = wall.get("front_distance", d.get("front_distance"))
            self.lblBigBox.setText(
                f"x={c[0]:+.3f} y={c[1]:+.3f} z={c[2]:+.3f} m"
                + (f" (front {fd:.3f})" if isinstance(fd, (int, float)) else ""))
            col = color.get("name") or d.get("color_name") or "?"
            conf = color.get("confidence") or d.get("color_conf") or 0.0
            self.lblBigBoxColor.setText(f"있음 · {col} ({conf:.2f})")
            # Cache for Auto "컨테이너 색" mode (only a known colour is usable).
            self._container_color = col if col not in ("?", "", "unknown") else None
        else:
            self.lblBigBox.setText("미검출")
            self.lblBigBoxColor.setText("없음")
            self._container_color = None

    # place_box colour name (english, from /place_box/info) -> Auto combo key.
    _ENG2KOR = {"red": "빨강", "yellow": "노랑", "green": "녹색",
                "blue": "파랑", "orange": "주황"}

    def _auto_start(self) -> None:
        """Auto-start picking. "컨테이너 색" 은 색 판단을 UI 가 하지 않는다 — container_pick_gate
        노드가 거리 게이트 + 색 시간-다수결로 /pick_color 를 몬다(UI 는 표시만). 그 외(전체/특정
        색)는 UI 가 색을 직접 지정한다."""
        if self._cpp_goal_active:
            self._set_status("Hover 정렬(디버그) 진행 중 — Cancel 후 자동 픽을 쓰세요.")
            return
        sel = self.comboColorAuto.currentText()
        if sel == "컨테이너 색":
            # 일회성 색 게이트 제거 — 게이트 노드가 컨테이너 유무(거리)·색(다수결)을 연속 판단.
            # 비웠다 다시 공급돼도 게이트가 거리로 인식·지속. UI 는 발행자가 아니다.
            self.lblPnpStatus.setText(
                "픽 서버 상태: 컨테이너 자동 — 게이트가 거리·색 판단 (~/status 표시)")
            self._pick_bridge.auto_start_container()
        else:
            self._pick_bridge.auto_start(sel)
        self._resident_auto_active = True
        self._update_path_exclusion()

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
        # Cloud는 카메라 optical 프레임(+X 오른쪽, +Y 아래, +Z 깊이/전방). 표시용으로만
        # (데이터 사본, 실좌표·pick 계산은 미변경) 영상과 좌우·상하를 일치시킨다.
        # GL 뷰(azimuth=90, elevation=-65)에서 화면 가로=-X_opt, 세로≈-Y_opt 라
        # Y만 뒤집으면 상하는 맞지만 단일축 반사(reflection)라 좌우가 거울처럼 뒤집힌다.
        # → X도 함께 뒤집어(= Z축 180° 정상회전, 거울 아님) 좌우까지 영상과 일치시킨다
        # (상하는 Y·Z 에만 의존하므로 X 반전의 영향 없음).
        pos = xyz.copy()
        pos[:, 0] = -pos[:, 0]
        pos[:, 1] = -pos[:, 1]
        for scatter in self._cloud_scatters:
            scatter.setData(pos=pos, color=rgba, size=2.0)

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
        # 자유구동(핸드 가이드) — 좌/우 토글. ON: 홈 이동 후 kp=0(중력보상 유지)로 손 이동. OFF: 복원+홈.
        self._btn_fd = {}
        for _side, _lab in (("left", "좌 자유구동"), ("right", "우 자유구동")):
            _b = QPushButton(_lab)
            _b.setCheckable(True)
            _b.setMinimumHeight(34)
            _b.setStyleSheet("QPushButton:checked{background:#ffb300;font-weight:bold;}")
            _b.setToolTip("ON: 홈 이동 후 kp=0(중력보상 유지)로 손으로 이동 / OFF: 위치제어 복원+홈복귀")
            _b.toggled.connect(lambda on, s=_side: self._toggle_freedrive(s, on))
            self._btn_fd[_side] = _b
            btn_row.addWidget(_b)
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

    def _toggle_freedrive(self, side: str, on: bool) -> None:
        """자유구동(핸드 가이드) 토글. ON: 홈 이동 -> --enable(kp=0, 중력보상 유지)로 손 이동 가능.
        OFF: --restore(JTC 활성=현재자세 캡처 -> kp 복원 -> 홈). arm_freedrive.py 가 안전순서 보장."""
        _R = "/home/openarmx/TR-Works/kkw/China"
        base = ("source /opt/ros/humble/setup.bash && "
                f"source {_R}/openarmx_ws/install/setup.bash")
        fd = f"python3 {_R}/experiments/arm_freedrive.py --side {side}"
        if on:
            cmd = f"{base} && {fd} --home && {fd} --enable"
            self._set_status(f"{side} 자유구동 ON — 홈 이동 후 손으로 이동 가능 (중력보상 유지)")
        else:
            cmd = f"{base} && {fd} --restore"
            self._set_status(f"{side} 자유구동 OFF — 위치제어 복원 + 홈 복귀")
        subprocess.Popen(["bash", "-c", cmd], stdout=subprocess.DEVNULL,
                         stderr=subprocess.DEVNULL, start_new_session=True)

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

    # ------------------------------------------------------------- Network tab
    def _build_network_tab(self) -> None:
        """New '네트워크' (Network) tab for remote-control settings — read-only
        check of ROS env (DOMAIN_ID, localhost-only, RMW) and host IP addresses.
        Display layer only; '새로고침' re-reads the live values."""
        tab = QWidget()
        root = QVBoxLayout(tab)

        # --- ROS 환경 (원격 제어 식별) ---
        grp_ros = QGroupBox("ROS 네트워크 (원격 제어 식별)")
        g = QGridLayout(grp_ros)
        self._net_lbl = {}
        rows = [
            ("ros_domain", "ROS_DOMAIN_ID"),
            ("localhost", "ROS_LOCALHOST_ONLY"),
            ("rmw", "RMW_IMPLEMENTATION"),
        ]
        for r, (key, cap) in enumerate(rows):
            g.addWidget(QLabel(f"<b>{cap}</b>"), r, 0)
            v = QLabel("---")
            v.setTextInteractionFlags(Qt.TextSelectableByMouse)
            self._net_lbl[key] = v
            g.addWidget(v, r, 1)
        g.setColumnStretch(1, 1)
        root.addWidget(grp_ros)

        # --- 호스트 IP (설정 확인) ---
        grp_ip = QGroupBox("호스트 IP (네트워크 설정 확인)")
        gi = QGridLayout(grp_ip)
        gi.addWidget(QLabel("<b>호스트명</b>"), 0, 0)
        self._net_lbl["host"] = QLabel("---")
        self._net_lbl["host"].setTextInteractionFlags(Qt.TextSelectableByMouse)
        gi.addWidget(self._net_lbl["host"], 0, 1)
        gi.addWidget(QLabel("<b>기본(외부) IP</b>"), 1, 0)
        self._net_lbl["primary_ip"] = QLabel("---")
        self._net_lbl["primary_ip"].setTextInteractionFlags(Qt.TextSelectableByMouse)
        gi.addWidget(self._net_lbl["primary_ip"], 1, 1)
        gi.addWidget(QLabel("<b>인터페이스</b>"), 2, 0, Qt.AlignTop)
        self._net_lbl["ifaces"] = QLabel("---")
        self._net_lbl["ifaces"].setTextInteractionFlags(Qt.TextSelectableByMouse)
        self._net_lbl["ifaces"].setWordWrap(True)
        gi.addWidget(self._net_lbl["ifaces"], 2, 1)
        gi.setColumnStretch(1, 1)
        root.addWidget(grp_ip)

        btn = QPushButton("새로고침")
        btn.clicked.connect(self._refresh_network)
        row = QHBoxLayout()
        row.addWidget(btn)
        row.addStretch()
        root.addLayout(row)
        root.addStretch(1)

        self.tabs.addTab(tab, "네트워크")
        self._refresh_network()

    def _refresh_network(self) -> None:
        """Re-read live ROS env + host IP and update the network-tab labels."""
        # ROS env
        self._net_lbl["ros_domain"].setText(
            os.environ.get("ROS_DOMAIN_ID", "0 (기본)"))
        lho = os.environ.get("ROS_LOCALHOST_ONLY", "0")
        self._net_lbl["localhost"].setText(
            f"{lho} ({'로컬 전용' if lho == '1' else '네트워크 개방'})")
        self._net_lbl["rmw"].setText(
            os.environ.get("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp (기본)"))

        # Host name
        try:
            self._net_lbl["host"].setText(socket.gethostname())
        except OSError:
            self._net_lbl["host"].setText("(조회 실패)")

        # Primary outbound IP (no packet actually sent)
        primary = "(없음)"
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                s.connect(("8.8.8.8", 80))
                primary = s.getsockname()[0]
            finally:
                s.close()
        except OSError:
            pass
        self._net_lbl["primary_ip"].setText(primary)

        # Per-interface IPv4 via `ip -4 -o addr` (read-only query)
        ifaces = []
        try:
            out = subprocess.run(
                ["ip", "-4", "-o", "addr", "show"],
                capture_output=True, text=True, timeout=2).stdout
            for ln in out.splitlines():
                parts = ln.split()
                if len(parts) >= 4 and parts[2] == "inet":
                    ifaces.append(f"{parts[1]} : {parts[3]}")
        except (OSError, subprocess.SubprocessError):
            pass
        self._net_lbl["ifaces"].setText("\n".join(ifaces) if ifaces else "(조회 실패)")

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
            "place_box": "큰박스(place) 검출",
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
        # ---- Motor-safe, two-phase teardown ---------------------------------
        # MIT-mode motors HOLD their last commanded position until the hardware's
        # on_deactivate() runs disable_all() (per CAN bus / per arm). ros2_control
        # deactivates the two arm components SEQUENTIALLY, so if ros2_control_node
        # is SIGKILLed before it finishes, the arm disabled SECOND (left, can3)
        # stays energised/stiff after the UI exits. Therefore: tear down the
        # non-hardware groups quickly, then give the HARDWARE node a generous
        # graceful window to disable BOTH arms before any SIGKILL.
        self._timer.stop()
        self._mj_timer.stop()
        self._pipe_tab.shutdown()
        self._bridge.shutdown()

        # Pick servers/bridge command the arms (and may hold an arm on its
        # arm_position_controller) — stop them FIRST so nothing drives the arm
        # while the hardware is being deactivated. They live outside self._procs.
        try:
            self._pick_bridge.shutdown()
        except Exception:
            pass
        self._pick_srv_proc.stop()
        self._kill_pattern("ptp_pick_resident")
        self._kill_pattern("box_detect_loop")
        self._kill_pattern("container_pick_gate")

        hw_keys = ("hw_sil", "hw_hw")
        # Phase 1 — non-hardware groups (controllers/cam/det/backend/rviz/gravity/
        # ee/tof/place_box): SIGINT all at once, one shared grace, quick SIGKILL.
        others = [p for k, p in self._procs.items()
                  if k not in hw_keys and p.running]
        for p in others:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGINT)
            except (ProcessLookupError, PermissionError):
                pass
        if others:
            time.sleep(1.0)                    # single shared grace, not per-proc
        for p in others:
            p.stop(sigint_timeout=0.2)         # quick SIGKILL of any straggler

        # Phase 2 — hardware (ros2_control_node): SIGINT, then WAIT for the launch
        # to exit on its own so on_deactivate()/disable_all() releases BOTH arms
        # (left included). `ros2 launch` only exits after ros2_control_node has
        # finished deactivating, so waiting for the parent ≈ waiting for the motor
        # disable to complete. SIGKILL only if it overruns the (generous) window.
        hw_procs = [self._procs[k] for k in hw_keys
                    if k in self._procs and self._procs[k].running]
        for p in hw_procs:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGINT)
            except (ProcessLookupError, PermissionError):
                pass
        if hw_procs:
            self._set_status("종료 전 모터 토크 해제(양팔 disable) 대기 중…")
            deadline = time.monotonic() + 6.0
            while time.monotonic() < deadline and any(p.running for p in hw_procs):
                QApplication.processEvents()
                time.sleep(0.05)
        for p in hw_procs:
            p.stop(sigint_timeout=0.2)         # SIGKILL only if it overran

        # Reap orphans (pgrep returns immediately when nothing matches).
        for key in list(self._started_keys):
            for pat in (self._presets[key].get("procs", [])
                        + self._presets[key].get("sweep", [])):
                self._kill_pattern(pat)
        event.accept()
