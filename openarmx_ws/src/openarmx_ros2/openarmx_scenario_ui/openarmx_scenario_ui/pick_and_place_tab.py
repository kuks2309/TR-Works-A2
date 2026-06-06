"""Pick and Place tab for openarmx_scenario_ui.

비전 박스 pick-and-place 통합 진입점. 선택한 백엔드(cyclo QP+CBF MoveL / pilz
MoveIt Pilz)의 ``AlignToBoxes`` 액션 골을 보내 박스를 감지→좌/우 팔 배정→타깃
위로 이동시키고, 결과(감지 박스·배정·도달 오차)를 표시한다.

감지는 YOLO ``DetectBox`` 서버(원격 Pi Hailo seg 백엔드 또는 로컬)에서, 모션은
선택 백엔드의 스택에서 수행된다. 전제 노드는 이 탭의 'Launch' 버튼으로 직접
기동할 수 있다(또는 Launch Manager / 터미널):
  D435 카메라 + YOLO DetectBox(`/yolov8_node/detect`) + base→camera TF
  + 선택 백엔드의 모션 스택
    · cyclo : cyclo MoveL 컨트롤러(`/openarmx/{left,right}/movel`)
    · pilz  : MoveIt move_group(Pilz, `/plan_kinematic_path`) + 좌/우 JTC

의존 결합을 줄이려 액션은 Launch Manager 와 동일하게 subprocess
``ros2 action send_goal`` 으로 보낸다(메시지 빌드 의존 불필요).
"""

from __future__ import annotations

import math
import os
import subprocess
from datetime import datetime

from PyQt5.QtCore import QProcess, Qt, QTimer
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QButtonGroup, QCheckBox, QComboBox, QDoubleSpinBox, QGridLayout, QGroupBox,
    QHBoxLayout, QLabel, QPushButton, QRadioButton, QTextEdit,
    QVBoxLayout, QWidget,
)

from openarmx_scenario_ui.managed_process import ManagedProcess

# 각 백엔드의 AlignToBoxes 액션 이름/타입. pilz 만 vel_scale·planner 필드를 가진다.
BACKENDS = {
    "cyclo": {
        "label": "cyclo  (QP+CBF MoveL · MoveIt-free)",
        "action": "/openarmx/align_to_boxes",
        "type": "openarmx_cyclo_box_align_msgs/action/AlignToBoxes",
        "pilz_fields": False,
    },
    "pilz": {
        "label": "pilz  (MoveIt Pilz LIN/PTP/CIRC)",
        "action": "/openarmx/pilz_align_to_boxes",
        "type": "openarmx_pilz_box_align_msgs/action/AlignToBoxes",
        "pilz_fields": True,
    },
    "ptp": {
        "label": "ptp  (Pinocchio 역기구학 · 단일점 직결 · 툴센터)",
        "action": "/openarmx/ptp_align_to_boxes",
        "type": "openarmx_ptp_box_align_msgs/action/AlignToBoxes",
        "pilz_fields": False,
    },
}

# mini-box seg 모델의 고정 5클래스(학습 순서). 체크된 색만 prompts(=DetectBox
# 클래스 필터)로 전달한다. (full class name, display label)
MINIBOX_CLASSES = [
    ("mini-box-blue", "blue"),
    ("mini-box-green", "green"),
    ("mini-box-orange", "orange"),
    ("mini-box-red", "red"),
    ("mini-box-yellow", "yellow"),
]

# D435 카메라(RealSense) 기동 — 깊이 정렬 + 컬러 + 포인트클라우드(박스 검출용).
# 전용 launch(d435_camera.launch.py)가 realsense 와 함께
# d435_center_link->camera_link 정적 TF 브리지를 같은 프로세스그룹으로 띄운다.
# 이 브리지가 있어야 /camera/camera 클라우드/박스 TF 가 openarmx_body_link0 으로
# 풀려 RViz 에 렌더된다(카메라 Start 시 클라우드가 RViz 에 보이도록).
D435_CMD = [
    "ros2", "launch", "openarmx_scenario_player", "d435_camera.launch.py",
]

# 원격 seg 검출 서버를 노드명 yolov8_node 로 기동(Pi Hailo seg 백엔드 드롭인) →
# /yolov8_node/detect 제공. yolov8_detection 은 3d_detect_ws 패키지라 UI 환경엔
# 보통 없으므로, launch 전에 해당 워크스페이스를 source 한 뒤 exec 한다(exec 로
# 같은 PID/프로세스그룹 유지 → ManagedProcess.stop 의 killpg 가 정상 동작).
_DETECT_WS_SETUP = "/home/openarmx/TR-Works/kkw/China/3d_detect_ws/install/setup.bash"
DETECT_CMD = [
    "bash", "-c",
    f"source {_DETECT_WS_SETUP} && exec ros2 launch yolov8_detection "
    "yolo_remote.launch.py node_name:=yolov8_node",
]

# 정렬 백엔드 box_align 노드 launch (선택 백엔드에 따라).
BACKEND_LAUNCH = {
    "cyclo": ["ros2", "launch", "openarmx_cyclo_box_align", "box_align.launch.py"],
    "pilz": ["ros2", "launch", "openarmx_pilz_box_align", "pilz_box_align.launch.py"],
    "ptp": ["ros2", "launch", "openarmx_ptp_box_align", "ptp_box_align.launch.py"],
}

# 외부(이 탭 밖)에서 이미 떠 있는지 pgrep 패턴.
EXT_DETECT = "yolov8_node|yolo_remote_node"
# 백엔드는 선택(cyclo/pilz)별로 구분한다. 'box_align_node' 는 'pilz_box_align_node'
# 에도 부분일치하므로, cyclo 는 패키지 경로로 한정해야 오탐이 없다.
BACKEND_EXT = {
    "cyclo": "openarmx_cyclo_box_align",
    "pilz": "pilz_box_align_node",
    "ptp": "ptp_box_align_node",
}

# Launch Manager 와 동일한 Start=초록 / Cancel=빨강 버튼 스타일(미러링).
_BTN_RUN_QSS = (
    "QPushButton { background-color:#43A047; color:white; font-weight:bold; "
    "border:none; border-radius:4px; padding:6px 16px; }"
    "QPushButton:hover { background-color:#388E3C; }"
    "QPushButton:pressed { background-color:#2E7D32; }"
)
_BTN_CANCEL_QSS = (
    "QPushButton { background-color:#E53935; color:white; font-weight:bold; "
    "border:none; border-radius:4px; padding:6px 16px; }"
    "QPushButton:hover { background-color:#C62828; }"
    "QPushButton:pressed { background-color:#B71C1C; }"
)
_HDR_QSS = "color:#1565c0; padding-top:8px; font-weight:bold;"


class PickAndPlaceTab(QWidget):
    def __init__(self, bridge=None, parent=None) -> None:
        super().__init__(parent)
        self._bridge = bridge
        # 라이브 자세 readout 라벨: {arm: {axis 또는 joint이름: QLabel}}
        self._cart_labels = {"left": {}, "right": {}}
        self._joint_labels = {"left": {}, "right": {}}
        self._proc = QProcess(self)
        self._proc.setProcessChannelMode(QProcess.MergedChannels)
        self._proc.readyReadStandardOutput.connect(self._on_output)
        self._proc.finished.connect(self._on_finished)
        # 단계적 점검용 단발 DetectBox 요청(로봇 이동 없음) — 별도 QProcess.
        self._detreq_proc = QProcess(self)
        self._detreq_proc.setProcessChannelMode(QProcess.MergedChannels)
        self._detreq_proc.readyReadStandardOutput.connect(self._on_detreq_output)
        self._detreq_proc.finished.connect(self._on_detreq_finished)
        self._cam_proc = ManagedProcess("pnp_d435_camera")
        self._det_proc = ManagedProcess("pnp_remote_seg")
        self._backend_proc = ManagedProcess("pnp_box_align_backend")
        self._build_ui()
        self._on_backend_changed()
        # 전제 노드 상태 폴링 (this-tab / external / stopped).
        self._status_timer = QTimer(self)
        self._status_timer.timeout.connect(self._refresh_status)
        self._status_timer.start(2000)
        self._refresh_status()

        # 라이브 자세 readout: joint 은 bridge 신호(이미 deg)로, Cartesian(TCP)은
        # 2Hz TF 로 갱신. bridge 가 있을 때만(독립 실행 대비 방어).
        if self._bridge is not None:
            self._bridge.sig_joint_state.connect(self._on_joint_state)
            self._readout_timer = QTimer(self)
            self._readout_timer.timeout.connect(self._refresh_readout)
            self._readout_timer.start(500)
            self._refresh_readout()

    # ------------------------------------------------------------------ UI
    def _build_ui(self) -> None:
        root = QVBoxLayout(self)

        # 상단: 좌측열(센서/백엔드/launch 그룹) + 우측 라이브 자세 readout 패널.
        top_row = QHBoxLayout()
        left_col = QVBoxLayout()
        top_row.addLayout(left_col, 3)
        top_row.addWidget(self._build_readout(), 2)
        root.addLayout(top_row)

        # --- 센서 (D435 카메라) — Start/Stop ------------------------------
        sgrp = QGroupBox("센서 (D435 카메라 · RealSense)")
        srow = QHBoxLayout(sgrp)
        self._btn_cam_start = QPushButton("카메라 Start")
        self._btn_cam_start.setStyleSheet(_BTN_RUN_QSS)
        self._btn_cam_start.setMinimumHeight(30)
        self._btn_cam_start.clicked.connect(self._start_camera)
        self._btn_cam_stop = QPushButton("카메라 Stop")
        self._btn_cam_stop.setStyleSheet(_BTN_CANCEL_QSS)
        self._btn_cam_stop.setMinimumHeight(30)
        self._btn_cam_stop.clicked.connect(self._stop_camera)
        self._lbl_cam = QLabel("● Stopped")
        self._lbl_cam.setStyleSheet("color:gray;")
        srow.addWidget(self._btn_cam_start)
        srow.addWidget(self._btn_cam_stop)
        srow.addWidget(self._lbl_cam)
        srow.addStretch()
        left_col.addWidget(sgrp)

        # --- 백엔드 선택 -------------------------------------------------
        bgrp = QGroupBox("백엔드 (Motion Backend)")
        brow = QHBoxLayout(bgrp)
        self._backend_group = QButtonGroup(self)
        self._backend_btns = {}
        for key, spec in BACKENDS.items():
            rb = QRadioButton(spec["label"])
            self._backend_btns[key] = rb
            self._backend_group.addButton(rb)
            brow.addWidget(rb)
        self._backend_btns["ptp"].setChecked(True)  # default (연결 전이라 핸들러 미발화)
        brow.addStretch()
        left_col.addWidget(bgrp)

        # --- 검출 · 백엔드 기동 (Launch) ---------------------------------
        lgrp = QGroupBox("검출 · 백엔드 기동 (Launch)")
        lgrid = QGridLayout(lgrp)
        lgrid.setHorizontalSpacing(8)
        lgrid.setVerticalSpacing(6)

        self._btn_det_start = QPushButton("원격검출 브리지(Pi) Start")
        self._btn_det_start.setStyleSheet(_BTN_RUN_QSS)
        self._btn_det_start.setMinimumHeight(30)
        self._btn_det_start.setToolTip(
            "라즈베리파이(Pi5+Hailo) 원격 검출 브리지 노드(yolov8_node) 기동/정지.\n"
            "이 노드는 직접 검출하지 않는다 — /yolov8_node/detect 액션을 제공하고,\n"
            "골이 오면 카메라 프레임 1장을 Pi 로 HTTP 중계해 YOLOv8-seg 결과를\n"
            "받아온다(on-demand). 실제 추론은 Pi 에서 수행.")
        self._btn_det_start.clicked.connect(self._start_detect)
        self._btn_det_stop = QPushButton("Stop")
        self._btn_det_stop.setStyleSheet(_BTN_CANCEL_QSS)
        self._btn_det_stop.setMinimumHeight(30)
        self._btn_det_stop.clicked.connect(self._stop_detect)
        self._lbl_det = QLabel("● Stopped")
        self._lbl_det.setStyleSheet("color:gray;")
        # 단계적 점검: 검출 1회 요청(로봇 이동 없음). 브리지 옆에 배치.
        self._btn_det_req = QPushButton("검출요청 (1회)")
        self._btn_det_req.setStyleSheet(_BTN_RUN_QSS)
        self._btn_det_req.setMinimumHeight(30)
        self._btn_det_req.setToolTip(
            "DetectBox 액션에 골 1개 전송(현재 confidence·클래스 필터 사용).\n"
            "로봇 이동 없이 Pi 검출 결과만 아래 '결과/로그'에 표시 — 단계적 점검용.\n"
            "전제: '원격검출 브리지(Pi) Start' 가 Running.")
        self._btn_det_req.clicked.connect(self._request_detect)
        lgrid.addWidget(self._btn_det_start, 0, 0)
        lgrid.addWidget(self._btn_det_stop, 0, 1)
        lgrid.addWidget(self._lbl_det, 0, 2)
        lgrid.addWidget(self._btn_det_req, 0, 3)

        self._btn_be_start = QPushButton("정렬 백엔드 Start")
        self._btn_be_start.setStyleSheet(_BTN_RUN_QSS)
        self._btn_be_start.setMinimumHeight(30)
        self._btn_be_start.clicked.connect(self._start_backend)
        self._btn_be_stop = QPushButton("Stop")
        self._btn_be_stop.setStyleSheet(_BTN_CANCEL_QSS)
        self._btn_be_stop.setMinimumHeight(30)
        self._btn_be_stop.clicked.connect(self._stop_backend)
        self._lbl_be = QLabel("● Stopped")
        self._lbl_be.setStyleSheet("color:gray;")
        lgrid.addWidget(self._btn_be_start, 1, 0)
        lgrid.addWidget(self._btn_be_stop, 1, 1)
        lgrid.addWidget(self._lbl_be, 1, 2)
        lgrid.setColumnStretch(4, 1)
        left_col.addWidget(lgrp)
        left_col.addStretch()

        # --- AlignToBoxes 골 파라미터 ------------------------------------
        pgrp = QGroupBox("AlignToBoxes 골 파라미터")
        grid = QGridLayout(pgrp)
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(6)

        def spin(lo, hi, step, val, dec, suffix=""):
            s = QDoubleSpinBox()
            s.setRange(lo, hi)
            s.setSingleStep(step)
            s.setDecimals(dec)
            s.setValue(val)
            if suffix:
                s.setSuffix(suffix)
            return s

        # 절대 높이(base). 기본 백엔드 ptp는 TCP(hand_tcp) 기준 → 운용 픽 높이 ≈0.80
        # (책상 0.72m, 박스 top≈0.78). pilz는 link7 기준이라 ≈0.94~0.98로 올려야 함.
        self._z = spin(0.0, 1.2, 0.01, 0.80, 3, " m")
        self._roll = spin(-180.0, 180.0, 1.0, 180.0, 1, " °")  # 기본 수직하강
        self._pitch = spin(-180.0, 180.0, 1.0, 0.0, 1, " °")
        self._yaw = spin(-180.0, 180.0, 1.0, 0.0, 1, " °")
        self._conf = spin(0.0, 1.0, 0.01, 0.50, 3)             # YOLO confidence (기본 0.5)
        self._vel = spin(0.0, 1.0, 0.05, 0.30, 2)              # pilz vel/acc scale

        self._arms = QComboBox()
        self._arms.addItems(["both", "left", "right"])
        self._planner = QComboBox()
        self._planner.addItems(["PTP", "LIN", "CIRC"])         # pilz only

        grid.addWidget(QLabel("<b>z (높이, base)</b>"), 0, 0)
        grid.addWidget(self._z, 0, 1)
        grid.addWidget(QLabel("<b>arms</b>"), 0, 2)
        grid.addWidget(self._arms, 0, 3)

        grid.addWidget(QLabel("<b>roll</b>"), 1, 0)
        grid.addWidget(self._roll, 1, 1)
        grid.addWidget(QLabel("<b>pitch</b>"), 1, 2)
        grid.addWidget(self._pitch, 1, 3)
        grid.addWidget(QLabel("<b>yaw</b>"), 1, 4)
        grid.addWidget(self._yaw, 1, 5)

        grid.addWidget(QLabel("<b>confidence</b>"), 2, 0)
        grid.addWidget(self._conf, 2, 1)
        self._lbl_vel = QLabel("<b>vel_scale</b>")
        grid.addWidget(self._lbl_vel, 2, 2)
        grid.addWidget(self._vel, 2, 3)
        self._lbl_planner = QLabel("<b>planner</b>")
        grid.addWidget(self._lbl_planner, 2, 4)
        grid.addWidget(self._planner, 2, 5)

        # 클래스(색) 필터 — seg 고정 5클래스. 체크된 색만 검출 대상(prompts).
        grid.addWidget(QLabel("<b>클래스(색)</b>"), 3, 0)
        cls_row = QHBoxLayout()
        self._classes = {}
        for cname, disp in MINIBOX_CLASSES:
            cb = QCheckBox(disp)
            cb.setChecked(True)
            self._classes[cname] = cb
            cls_row.addWidget(cb)
        cls_row.addStretch()
        grid.addLayout(cls_row, 3, 1, 1, 5)
        root.addWidget(pgrp)

        # --- 실행 컨트롤 -------------------------------------------------
        ctl = QHBoxLayout()
        self._btn_run = QPushButton("Run AlignToBoxes")
        self._btn_run.setStyleSheet(_BTN_RUN_QSS)
        self._btn_run.setMinimumHeight(34)
        self._btn_run.clicked.connect(self._run)
        self._btn_cancel = QPushButton("Cancel")
        self._btn_cancel.setStyleSheet(_BTN_CANCEL_QSS)
        self._btn_cancel.setMinimumHeight(34)
        self._btn_cancel.setMinimumWidth(90)
        self._btn_cancel.clicked.connect(self._cancel)
        self._btn_check = QPushButton("전제조건 확인")
        self._btn_check.setMinimumHeight(34)
        self._btn_check.clicked.connect(self._check_prereqs)
        ctl.addWidget(self._btn_run)
        ctl.addWidget(self._btn_cancel)
        ctl.addWidget(self._btn_check)
        ctl.addStretch()
        root.addLayout(ctl)

        # --- 결과 / 로그 ------------------------------------------------
        rgrp = QGroupBox("결과 / 로그 (action feedback · result)")
        rlay = QVBoxLayout(rgrp)
        rhdr = QHBoxLayout()
        rhdr.addStretch()
        self._btn_clear_log = QPushButton("로그 지우기")
        self._btn_clear_log.setMinimumHeight(26)
        self._btn_clear_log.setToolTip("결과/로그 내용을 지웁니다.")
        self._btn_clear_log.clicked.connect(self._clear_log)
        rhdr.addWidget(self._btn_clear_log)
        rlay.addLayout(rhdr)
        self._log = QTextEdit()
        self._log.setReadOnly(True)
        self._log.setFont(QFont("monospace"))
        self._log.setLineWrapMode(QTextEdit.NoWrap)
        rlay.addWidget(self._log)
        root.addWidget(rgrp, 1)

        self._lbl_status = QLabel("Ready")
        self._lbl_status.setStyleSheet("color:#444; padding:2px;")
        root.addWidget(self._lbl_status)

        # 모든 위젯 생성 후에 백엔드 토글을 연결한다 — _build_ui 중간에 연결하면
        # 위 setChecked가 vel_scale/planner 위젯 생성 전에 핸들러를 때려 AttributeError.
        for rb in self._backend_btns.values():
            rb.toggled.connect(self._on_backend_changed)

    # --------------------------------------------------------------- helpers
    # ------------------------------------------------------ live pose readout
    _RO_VAL_QSS = ("background:#f5f5f5; border:1px solid #ddd; border-radius:3px; "
                   "padding:1px 6px; font-family:monospace;")

    def _build_readout(self) -> QGroupBox:
        """우상단 라이브 자세 패널: 양팔 TCP(hand_tcp) Cartesian + Joint(deg),
        base(openarmx_body_link0) 기준. Cartesian 은 TF, Joint 은 /joint_states."""
        grp = QGroupBox("현재 자세 (실측 · base 기준)")
        grid = QGridLayout(grp)
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(2)
        for c, txt in ((0, "<b>TCP</b>"), (1, "<b>Left</b>"), (2, "<b>Right</b>")):
            grid.addWidget(QLabel(txt), 0, c)
        r = 1
        for label, key in (("x (m)", "x"), ("y (m)", "y"), ("z (m)", "z"),
                           ("roll (°)", "R"), ("pitch (°)", "P"), ("yaw (°)", "Y")):
            grid.addWidget(QLabel(label), r, 0)
            for c, arm in ((1, "left"), (2, "right")):
                lb = QLabel("---")
                lb.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
                lb.setStyleSheet(self._RO_VAL_QSS)
                self._cart_labels[arm][key] = lb
                grid.addWidget(lb, r, c)
            r += 1
        grid.addWidget(QLabel("<b>Joint (°)</b>"), r, 0)
        r += 1
        for j in range(1, 8):
            grid.addWidget(QLabel(f"J{j}"), r, 0)
            for c, arm in ((1, "left"), (2, "right")):
                lb = QLabel("---")
                lb.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
                lb.setStyleSheet(self._RO_VAL_QSS)
                self._joint_labels[arm][f"openarmx_{arm}_joint{j}"] = lb
                grid.addWidget(lb, r, c)
            r += 1
        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(2, 1)
        grid.setRowStretch(r, 1)
        return grp

    def _on_joint_state(self, positions: dict) -> None:
        """bridge.sig_joint_state(dict, 팔관절은 이미 deg) -> Joint 라벨 갱신."""
        for arm in ("left", "right"):
            for name, lb in self._joint_labels[arm].items():
                if name in positions:
                    lb.setText(f"{positions[name]:+.1f}")

    def _refresh_readout(self) -> None:
        """2Hz: 양팔 hand_tcp 포즈를 TF 로 읽어 Cartesian 라벨 갱신."""
        if self._bridge is None:
            return
        for arm in ("left", "right"):
            cl = self._cart_labels[arm]
            pose = self._bridge.get_ee_pose(
                arm, "openarmx_body_link0", f"openarmx_{arm}_hand_tcp")
            if not pose:
                for lb in cl.values():
                    lb.setText("---")
                continue
            cl["x"].setText(f"{pose['x']:+.4f}")
            cl["y"].setText(f"{pose['y']:+.4f}")
            cl["z"].setText(f"{pose['z']:+.4f}")
            cl["R"].setText(f"{math.degrees(pose['roll']):+.1f}")
            cl["P"].setText(f"{math.degrees(pose['pitch']):+.1f}")
            cl["Y"].setText(f"{math.degrees(pose['yaw']):+.1f}")

    def _backend_key(self) -> str:
        for key, rb in self._backend_btns.items():
            if rb.isChecked():
                return key
        return "ptp"

    def _on_backend_changed(self, *_a) -> None:
        # pilz 전용 필드(vel_scale·planner)는 pilz 외(cyclo/ptp) 선택 시 비활성화.
        pilz = BACKENDS[self._backend_key()]["pilz_fields"]
        for w in (self._lbl_vel, self._vel, self._lbl_planner, self._planner):
            w.setEnabled(pilz)
        # 백엔드를 바꾸면 떠있는(이전) 정렬 백엔드를 종료한다 — 안 그러면 선택은
        # cyclo인데 pilz가 떠 있는 식의 불일치로 AlignToBoxes 가 서버를 못 찾는다
        # ("Waiting for an action server..."). 새 백엔드는 'Start'로 다시 기동.
        if self._backend_proc.running:
            self._backend_proc.stop()
            self._set_status("백엔드 변경 — 정렬 백엔드 종료됨. 'Start'로 선택 백엔드를 기동하세요.")
        self._refresh_status()   # 백엔드 라벨을 선택 백엔드 기준으로 즉시 갱신

    def _selected_classes(self) -> str:
        """체크된 mini-box 색 -> 콤마결합 클래스명(DetectBox prompts = 클래스 필터).
        전체 체크 또는 전체 해제 -> '' (필터 없음 = 5색 모두)."""
        sel = [n for n, cb in self._classes.items() if cb.isChecked()]
        return ",".join(sel) if 0 < len(sel) < len(self._classes) else ""

    def _goal_yaml(self) -> str:
        parts = [
            f"z: {self._z.value():.4f}",
            f"roll_deg: {self._roll.value():.1f}",
            f"pitch_deg: {self._pitch.value():.1f}",
            f"yaw_deg: {self._yaw.value():.1f}",
            f"arms: {self._arms.currentText()}",
            f"confidence: {self._conf.value():.3f}",
            f"prompts: '{self._selected_classes()}'",
        ]
        if BACKENDS[self._backend_key()]["pilz_fields"]:
            parts.append(f"vel_scale: {self._vel.value():.2f}")
            parts.append(f"planner: {self._planner.currentText()}")
        return "{" + ", ".join(parts) + "}"

    def _append(self, text: str) -> None:
        self._log.append(text)
        sb = self._log.verticalScrollBar()
        sb.setValue(sb.maximum())

    def _set_status(self, text: str) -> None:
        ts = datetime.now().strftime("%H:%M:%S")
        self._lbl_status.setText(f"{ts}  {text}")

    def _clear_log(self) -> None:
        self._log.clear()
        self._set_status("로그를 지웠습니다.")

    # --------------------------------------------------------------- actions
    def _run(self) -> None:
        if self._proc.state() != QProcess.NotRunning:
            self._set_status("이미 골 실행 중 — 먼저 Cancel 하세요.")
            return
        spec = BACKENDS[self._backend_key()]
        argv = ["action", "send_goal", "--feedback",
                spec["action"], spec["type"], self._goal_yaml()]
        self._log.clear()
        self._append(f"$ ros2 {' '.join(argv)}\n")
        self._set_status(f"{self._backend_key()} AlignToBoxes 전송 중…")
        self._proc.start("ros2", argv)

    def _cancel(self) -> None:
        if self._proc.state() != QProcess.NotRunning:
            self._proc.kill()
            self._set_status("골 취소(프로세스 종료).")
        else:
            self._set_status("실행 중인 골 없음.")

    def _check_prereqs(self) -> None:
        """선택 백엔드 액션 서버 + YOLO DetectBox 존재 여부를 한 번 확인."""
        if self._proc.state() != QProcess.NotRunning:
            self._set_status("실행 중 — 전제조건 확인은 잠시 후.")
            return
        spec = BACKENDS[self._backend_key()]
        proc = QProcess(self)
        proc.start("ros2", ["action", "list"])
        proc.waitForFinished(4000)
        out = bytes(proc.readAllStandardOutput()).decode(errors="replace")
        has_align = spec["action"] in out
        has_detect = "/yolov8_node/detect" in out
        self._append(
            f"[전제조건] {spec['action']}: {'OK' if has_align else '없음'}  |  "
            f"YOLO DetectBox: {'OK' if has_detect else '없음'}")
        self._set_status(
            "전제조건 OK" if (has_align and has_detect)
            else "전제조건 미충족 — 위 Launch 버튼으로 검출/백엔드를 기동하세요.")

    # --------------------------------------------------------- launch: camera
    def _start_camera(self) -> None:
        if self._cam_proc.running:
            self._set_status("D435 카메라: 이미 실행 중(이 탭).")
            return
        if self._proc_running_external("realsense2_camera",
                                       self._running_cmdlines()):
            self._set_status("D435 카메라가 외부에서 이미 실행 중 — 중복 방지(먼저 Stop).")
            return
        if self._cam_proc.start(D435_CMD):
            self._set_status(f"D435 카메라 기동 → 로그 {self._cam_proc.log_path}")
        self._refresh_status()

    def _stop_camera(self) -> None:
        self._cam_proc.stop()                       # 이 탭이 띄운 프로세스 그룹 종료
        try:                                        # 외부/잔여 realsense 도 SIGINT 로 정리
            subprocess.run(["pkill", "-INT", "-f", "realsense2_camera"], timeout=3)
        except Exception:
            pass
        self._set_status("D435 카메라 종료.")
        self._refresh_status()

    # ------------------------------------------------- launch: detect/backend
    def _start_detect(self) -> None:
        if self._det_proc.running:
            self._set_status("원격검출 브리지: 이미 실행 중(이 탭).")
            return
        if self._det_proc.start(DETECT_CMD):
            self._set_status(
                f"원격검출 브리지 기동 → /yolov8_node/detect (로그 {self._det_proc.log_path})")
        self._refresh_status()

    def _stop_detect(self) -> None:
        self._det_proc.stop()
        self._set_status("원격검출 브리지 종료.")
        self._refresh_status()

    # ----- 단계적 점검: 단발 DetectBox 요청 (로봇 이동 없음) -----
    def _detect_request_cmd(self) -> list:
        # 현재 UI 의 confidence·클래스 필터를 그대로 사용. ros2 CLI 가
        # yolov8_detection_msgs/action/DetectBox 타입을 인식하도록 3d_detect_ws 소싱.
        goal = ("{prompts: '%s', confidence: %.3f, publish_annotated: true}"
                % (self._selected_classes(), self._conf.value()))
        inner = (f"source {_DETECT_WS_SETUP} && exec timeout 30 "
                 f"ros2 action send_goal /yolov8_node/detect "
                 f"yolov8_detection_msgs/action/DetectBox \"{goal}\"")
        return ["bash", "-c", inner]

    def _request_detect(self) -> None:
        if not (self._det_proc.running
                or self._proc_running_external(EXT_DETECT,
                                               self._running_cmdlines())):
            self._set_status("원격검출 브리지가 꺼져 있음 — 먼저 '원격검출 브리지(Pi) Start'.")
            return
        if self._detreq_proc.state() != QProcess.NotRunning:
            self._set_status("검출요청 진행 중 — 잠시 후 다시.")
            return
        cmd = self._detect_request_cmd()
        self._log.clear()
        self._append(f"$ {cmd[-1]}\n")
        self._set_status("검출요청 전송 중… (Pi 추론)")
        self._detreq_proc.start(cmd[0], cmd[1:])

    def _on_detreq_output(self) -> None:
        text = bytes(self._detreq_proc.readAllStandardOutput()).decode(errors="replace")
        if text:
            self._append(text.rstrip("\n"))

    def _on_detreq_finished(self, code, _status) -> None:
        self._set_status(f"검출요청 종료 (exit {code}).")

    def _start_backend(self) -> None:
        if self._backend_proc.running:
            self._set_status("정렬 백엔드: 이미 실행 중(이 탭).")
            return
        key = self._backend_key()
        if self._backend_proc.start(BACKEND_LAUNCH[key]):
            self._set_status(
                f"{key} box_align 기동 → {BACKENDS[key]['action']} "
                f"(로그 {self._backend_proc.log_path})")
        self._refresh_status()

    def _stop_backend(self) -> None:
        self._backend_proc.stop()
        self._set_status("정렬 백엔드 종료.")
        self._refresh_status()

    # ----------------------------------------------------------- status poll
    @staticmethod
    def _running_cmdlines() -> list:
        """Snapshot every process command line via one /proc read (no
        subprocess). Replaces per-pattern `pgrep -f`, which blocked the GUI
        thread on the 2 s status timer. A substring test reproduces
        `pgrep -f`'s literal-match semantics.
        """
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

    @staticmethod
    def _proc_running_external(pattern: str, cmdlines: list) -> bool:
        return any(pattern in c for c in cmdlines)

    def _set_run_label(self, lbl: QLabel, managed, ext_pattern: str,
                       cmdlines: list) -> None:
        if managed.running:
            lbl.setText("● Running (this tab)")
            lbl.setStyleSheet("color:#080; font-weight:bold;")
        elif self._proc_running_external(ext_pattern, cmdlines):
            lbl.setText("● Running (external)")
            lbl.setStyleSheet("color:#d80;")
        else:
            lbl.setText("● Stopped")
            lbl.setStyleSheet("color:gray;")

    def _refresh_status(self) -> None:
        cmdlines = self._running_cmdlines()
        self._set_run_label(self._lbl_cam, self._cam_proc,
                            "realsense2_camera", cmdlines)
        self._set_run_label(self._lbl_det, self._det_proc, EXT_DETECT, cmdlines)
        self._set_run_label(self._lbl_be, self._backend_proc,
                            BACKEND_EXT[self._backend_key()], cmdlines)

    # --------------------------------------------------------------- process
    def _on_output(self) -> None:
        text = bytes(self._proc.readAllStandardOutput()).decode(errors="replace")
        if text:
            self._append(text.rstrip("\n"))

    def _on_finished(self, code, _status) -> None:
        self._set_status(f"골 종료 (exit {code}).")

    def shutdown(self) -> None:
        if hasattr(self, "_status_timer"):
            self._status_timer.stop()
        if self._proc.state() != QProcess.NotRunning:
            self._proc.kill()
            self._proc.waitForFinished(1500)
        if self._detreq_proc.state() != QProcess.NotRunning:
            self._detreq_proc.kill()
            self._detreq_proc.waitForFinished(1500)
        # UI 종료 시 이 탭이 띄운 카메라/검출/백엔드 launch 프로세스그룹도 함께 종료한다
        # (a2-scenario 종료 후 /camera/camera, d435_center_to_camera_link_tf, yolov8_node,
        # box_align 노드가 잔존하지 않도록). ManagedProcess.stop 이 killpg 로 그룹 전체 정리.
        self._cam_proc.stop()
        self._det_proc.stop()
        self._backend_proc.stop()
