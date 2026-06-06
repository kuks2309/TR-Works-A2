"""Launch Manager tab for openarmx_scenario_ui.

A dedicated tab to start/stop the common launch files and nodes of the
openarmx workspace, each with its own Start/Stop button and a live status
indicator. Before starting (and continuously thereafter) it detects whether
the same launch/node is ALREADY running — either started by this tab or by an
external terminal — by checking `ros2 node list` and process patterns, so the
user is warned against launching duplicates.
"""

from __future__ import annotations

import os
import re
import signal
import subprocess
import time
from datetime import datetime

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QApplication, QCheckBox, QGridLayout, QGroupBox, QHBoxLayout, QLabel,
    QLineEdit, QMessageBox, QPushButton, QScrollArea, QTextEdit, QVBoxLayout,
    QWidget,
)

from openarmx_scenario_ui.managed_process import ManagedProcess

# Scenario RViz config — resolve the SRC path via realpath so the symlink-
# installed module still points at the checked-in .rviz (project rule:
# RViz config loaded from src, not install/share).
_OPENARMX_ROS2 = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.realpath(__file__))))
_SCENARIO_RVIZ = os.path.join(
    _OPENARMX_ROS2, "openarmx_scenario_player", "config", "openarmx_scenario.rviz")

# Start = green (go), Stop = red. Larger + colored with hover/pressed states.
_BTN_START_QSS = (
    "QPushButton { background-color:#43A047; color:white; font-weight:bold; "
    "border:none; border-radius:4px; padding:6px 16px; }"
    "QPushButton:hover { background-color:#388E3C; }"
    "QPushButton:pressed { background-color:#2E7D32; }"
)
_BTN_STOP_QSS = (
    "QPushButton { background-color:#E53935; color:white; font-weight:bold; "
    "border:none; border-radius:4px; padding:6px 16px; }"
    "QPushButton:hover { background-color:#C62828; }"
    "QPushButton:pressed { background-color:#B71C1C; }"
)
_BTN_W = 90      # button min width
_BTN_H = 34      # button min height


# Each preset: key, group (section header), label, cmd (argv list),
# confirm (warn before start), nodes (ros2 node names = running),
# procs (pgrep -f patterns = running), sweep (extra kill patterns on stop).
# Grouped by role so the purpose of each target is obvious.
PRESETS = [
    # ---- L0 하드웨어 (Hardware only) — controller_manager + 인터페이스 + RSP,
    #      컨트롤러는 스폰 안 함. 새 자기완결 launch(fork 미수정). 택1. ----
    {
        "key": "sil_hw",
        "group": "L0 · 하드웨어 (Hardware) — 택1",
        "label": "SIL 하드웨어 — 시뮬 (fake HW, 컨트롤러 X)",
        "cmd": ["ros2", "launch", "openarmx_scenario_player",
                "openarmx_hardware.launch.py", "use_fake_hardware:=true"],
        "confirm": False,
        "nodes": [],
        "procs": ["use_fake_hardware:=true"],
        "sweep": ["openarmx_hardware.launch", "ros2_control_node"],
    },
    {
        "key": "hw_hw",
        "group": "L0 · 하드웨어 (Hardware) — 택1",
        "label": "HW 하드웨어 — 실로봇 (CAN 모터, 컨트롤러 X)",
        "cmd": ["ros2", "launch", "openarmx_scenario_player",
                "openarmx_hardware.launch.py", "use_fake_hardware:=false",
                "control_mode:=mit",
                # follower 팔 = can2(right)/can3(left). can0/can1 은 leader(텔레옵 입력)라
                # pick(AlignToBoxes)은 follower 에서 수행해야 한다.
                "right_can_interface:=can2", "left_can_interface:=can3",
                "can_fd:=false"],
        "confirm": True,
        "nodes": [],
        "procs": ["use_fake_hardware:=false"],
        "sweep": ["openarmx_hardware.launch", "ros2_control_node"],
    },
    # ---- L1 컨트롤러 (Controllers) — 실행 중 controller_manager에 스폰 ----
    {
        "key": "controllers",
        "group": "L1 · 컨트롤러 (Controllers) — L0 필요",
        "label": "컨트롤러 스폰 — joint_state_broadcaster + 좌우 관절궤적 컨트롤러 + 그리퍼",
        "cmd": ["ros2", "run", "controller_manager", "spawner",
                "joint_state_broadcaster",
                "left_joint_trajectory_controller",
                "right_joint_trajectory_controller",
                "left_gripper_controller", "right_gripper_controller",
                "-c", "/controller_manager", "--unload-on-kill"],
        "confirm": False,
        "nodes": ["/left_joint_trajectory_controller",
                  "/right_joint_trajectory_controller"],
        "procs": ["controller_manager spawner"],
    },
    # 중력 보상 토글 — Start/Stop 대신 체크박스(control:"checkbox")로 렌더된다.
    # ON  : gravity_comp.launch.py 기동 (좌·우 forward_effort_controller 스폰 후
    #       spawner 종료·컨트롤러 잔존 + gravity_comp_node 양팔 g(q) 피드포워드).
    # OFF : enable_compensation=false 로 토크를 먼저 0 으로 만든 뒤(노드가 0 발행 →
    #       effort 컨트롤러 출력 0) launch 종료. 컨트롤러는 잔존하되 0 을 출력하므로
    #       잔여 토크 없음. (--unload-on-kill 미사용: spawner 사망 시 컨트롤러가
    #       조용히 언로드돼 보상이 무력화되던 결함 차단, 2026-06-07.)
    {
        "key": "gravity_comp",
        "group": "L1 · 컨트롤러 (Controllers) — L0 필요",
        "label": "중력 보상 (Gravity Comp) — 양팔 g(q) 피드포워드 (L1 컨트롤러 필요)",
        "cmd": ["ros2", "launch", "openarmx_gravity_comp",
                "gravity_comp.launch.py", "g_scale:=0.95"],
        "control": "checkbox",
        "confirm": False,
        "nodes": ["/gravity_comp_node"],
        "procs": ["gravity_comp.launch"],
        "sweep": ["gravity_comp_node"],
    },
    # ---- L2 모션 / 플래너 (Motion) — L1 필요 ----
    {
        "key": "move_group",
        "group": "L2 · 모션 / 플래너 (Motion) — L1 필요",
        "label": "MoveIt move_group — Pilz LIN/PTP plan&execute",
        # move_group ONLY (no controllers/RViz) → composes on top of a bringup
        # without the controller_manager conflict that demo.launch.py caused.
        "cmd": ["ros2", "launch", "openarmx_bimanual_moveit_config",
                "move_group.launch.py"],
        "confirm": False,
        "nodes": ["/move_group"],
        "procs": ["move_group.launch"],
    },
    {
        "key": "cyclo_movel",
        "group": "L2 · 모션 / 플래너 (Motion) — L1 필요",
        "label": "cyclo MoveL 컨트롤러 — QP+CBF 직접제어",
        "cmd": ["ros2", "launch", "openarmx_pick",
                "openarmx_movel_bimanual.launch.py"],
        "confirm": False,
        "nodes": ["/openarmx_left_movel_controller",
                  "/openarmx_right_movel_controller"],
        "procs": ["openarmx_movel_bimanual"],
        # [cyclo Stop 종료보장 2026-06-07] launch 파일명 패턴("openarmx_movel_bimanual")은
        # `ros2 launch` 부모 프로세스만 매칭한다. 부모가 먼저 죽고 컨트롤러 노드만 고아로
        # 남으면(외부 터미널 종료 등) 그 패턴이 아무것도 못 잡아 노드가 살아남는다 →
        # Stop 이 안 듣는 것처럼 보임. 실제 노드 실행파일명을 sweep 에 넣어 좌/우
        # omx_movel_controller_node 를 직접(그룹+PID) 종료한다.
        "sweep": ["omx_movel_controller_node"],
    },
    # ---- 티칭 / 시각화 (Teaching / View) ----
    {
        "key": "ee_markers",
        "group": "L3 · 티칭 / 시각화 (Teaching / View) — L0 필요",
        "label": "EE Leader 마커 — RViz 6-DoF 드래그 티칭",
        # The ee_leader_marker launch defaults (base_link / left_end_effector /
        # right_end_effector) are robot-agnostic placeholders that DON'T exist in
        # the openarmx TF tree, so the node's base_frame→controlled_link TF lookup
        # fails forever and the interactive marker is never inserted (RViz shows
        # the display but no marker). Pass the openarmx-specific frames + goal
        # topics — identical to scenario_player_with_ee_leader.launch.py (SSOT).
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
    },
    {
        "key": "scenario_rviz",
        "group": "L3 · 티칭 / 시각화 (Teaching / View) — L0 필요",
        "label": "Scenario RViz — 마커/박스 디스플레이",
        # Rename the node to /openarmx_scenario_rviz so the "nodes" health check
        # below, the Node Health tab, and kill_all_ros2.sh (which maps node names
        # to PIDs via the __node:= remap) all see a consistent name. A bare
        # `rviz2` spawns the node as /rviz, which would never match.
        "cmd": ["rviz2", "-d", _SCENARIO_RVIZ,
                "--ros-args", "-r", "__node:=openarmx_scenario_rviz"],
        "confirm": False,
        "nodes": ["/openarmx_scenario_rviz"],
        "procs": ["openarmx_scenario.rviz"],
        "sweep": ["openarmx_scenario.rviz"],
    },
    # ---- 시나리오 (Scenario) ----
    {
        "key": "scenario_player",
        "group": "L4 · 시나리오 (Scenario) — L2 필요",
        "label": "Scenario Player 노드 (재생 백엔드)",
        "cmd": ["ros2", "run", "openarmx_scenario_player",
                "scenario_player_node.py"],
        "confirm": False,
        "nodes": ["/scenario_player"],
        "procs": ["scenario_player_node"],
    },
]

CUSTOM_KEY = "custom"


class LaunchManagerTab(QWidget):
    def __init__(self, bridge=None, parent=None) -> None:
        super().__init__(parent)
        # In-process ROS bridge — lets node discovery use the rclpy graph API
        # instead of a blocking `ros2 node list` subprocess (which froze the
        # whole GUI ~50% of the time on the 2 s refresh timer). Optional so the
        # tab still constructs standalone (falls back to subprocess).
        self._bridge = bridge
        self._procs = {p["key"]: ManagedProcess(f"launch_{p['key']}") for p in PRESETS}
        self._procs[CUSTOM_KEY] = ManagedProcess("launch_custom")
        self._rows = {}          # key -> (start_btn, stop_btn, status_lbl)
        self._node_cache = []    # last `ros2 node list` result
        self._started_keys = set()     # presets started this session (cleanup)
        self._custom_patterns = set()  # custom launch/run signatures started
        self._build_ui()

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(2000)
        self._refresh()

    # ------------------------------------------------------------------
    # UI
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)

        grp = QGroupBox("Launch / Node Manager")
        grid = QGridLayout(grp)
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(6)
        # Row layout: Target (50%) | [Start][Stop] (25%) | Status (25%)
        grid.setColumnStretch(0, 2)
        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(2, 1)

        grid.addWidget(QLabel("<b>Target</b>"), 0, 0)
        grid.addWidget(QLabel("<b>Start / Stop</b>"), 0, 1)
        grid.addWidget(QLabel("<b>Status</b>"), 0, 2)

        row = 1
        last_group = None
        for preset in PRESETS:
            g = preset.get("group", "")
            if g != last_group:
                hdr = QLabel(f"<b>{g}</b>")
                hdr.setStyleSheet("color:#1565c0; padding-top:8px;")
                grid.addWidget(hdr, row, 0, 1, 3)
                last_group = g
                row += 1
            key = preset["key"]
            name = QLabel(preset["label"])
            name.setStyleSheet("padding-left:10px;")
            grid.addWidget(name, row, 0)
            # Checkbox-controlled target (gravity comp): a single ON/OFF toggle
            # in the buttons cell instead of Start/Stop. _refresh keeps it in
            # sync with the actual node state (blockSignals to avoid recursion).
            if preset.get("control") == "checkbox":
                chk_cell = QWidget()
                ch = QHBoxLayout(chk_cell)
                ch.setContentsMargins(0, 0, 0, 0)
                chk = QCheckBox("실행")
                chk.setMinimumHeight(_BTN_H)
                chk.toggled.connect(self._on_gravity_toggled)
                ch.addWidget(chk)
                ch.addStretch()
                grid.addWidget(chk_cell, row, 1)
                status = QLabel("● Stopped")
                status.setStyleSheet("color:gray;")
                grid.addWidget(status, row, 2)
                self._gravity_check = chk
                self._gravity_status = status
                row += 1
                continue
            # Start + Stop grouped together in the single 25% buttons cell.
            btn_cell = QWidget()
            bh = QHBoxLayout(btn_cell)
            bh.setContentsMargins(0, 0, 0, 0)
            bh.setSpacing(6)
            start = QPushButton("Start")
            start.setMinimumWidth(_BTN_W)
            start.setMinimumHeight(_BTN_H)
            start.setStyleSheet(_BTN_START_QSS)
            stop = QPushButton("Stop")
            stop.setMinimumWidth(_BTN_W)
            stop.setMinimumHeight(_BTN_H)
            stop.setStyleSheet(_BTN_STOP_QSS)
            start.clicked.connect(lambda _c, k=key: self._start(k))
            stop.clicked.connect(lambda _c, k=key: self._stop(k))
            bh.addWidget(start)
            bh.addWidget(stop)
            bh.addStretch()
            grid.addWidget(btn_cell, row, 1)
            status = QLabel("● Stopped")
            status.setStyleSheet("color:gray;")
            grid.addWidget(status, row, 2)
            self._rows[key] = (start, stop, status)
            row += 1

        root.addWidget(grp)

        # Custom command row
        cgrp = QGroupBox("Custom command")
        crow = QHBoxLayout(cgrp)
        self._custom_edit = QLineEdit()
        self._custom_edit.setPlaceholderText(
            "ros2 launch <pkg> <file>.launch.py  /  ros2 run <pkg> <exe>")
        cstart = QPushButton("Start")
        cstart.setMinimumWidth(_BTN_W)
        cstart.setMinimumHeight(_BTN_H)
        cstart.setStyleSheet(_BTN_START_QSS)
        cstop = QPushButton("Stop")
        cstop.setMinimumWidth(_BTN_W)
        cstop.setMinimumHeight(_BTN_H)
        cstop.setStyleSheet(_BTN_STOP_QSS)
        self._custom_status = QLabel("● Stopped")
        self._custom_status.setStyleSheet("color:gray;")
        self._custom_status.setFixedWidth(210)
        cstart.clicked.connect(self._start_custom)
        cstop.clicked.connect(lambda: self._stop(CUSTOM_KEY))
        crow.addWidget(self._custom_edit)
        crow.addWidget(cstart)
        crow.addWidget(cstop)
        crow.addWidget(self._custom_status)
        root.addWidget(cgrp)

        # Global controls + node list
        ctl = QHBoxLayout()
        self.btn_stop_all = QPushButton("Stop All (this tab)")
        self.btn_stop_all.setStyleSheet("color:#c00; font-weight:bold;")
        self.btn_stop_all.clicked.connect(self._stop_all)
        self.btn_refresh = QPushButton("Refresh status")
        self.btn_refresh.clicked.connect(self._refresh)
        ctl.addWidget(self.btn_stop_all)
        ctl.addWidget(self.btn_refresh)
        ctl.addStretch()
        root.addLayout(ctl)

        # (Active ROS2 node list moved to the Node Health tab.)
        root.addStretch(1)

        self._lbl_status = QLabel("Ready")
        self._lbl_status.setStyleSheet("color:#444; padding:2px;")
        root.addWidget(self._lbl_status)

    # ------------------------------------------------------------------
    # Detection
    # ------------------------------------------------------------------

    def _query_nodes(self) -> list:
        """Snapshot of live ROS2 node names.

        Uses the in-process rclpy graph (`bridge.live_node_names()`) so the
        2 s status refresh never spawns/blocks on a `ros2 node list`
        subprocess (the cause of the GUI freezing ~50% of the time, which made
        modal dialogs like Teaching-tab Capture unresponsive). Falls back to
        the subprocess only when no bridge is available.
        """
        if self._bridge is not None:
            try:
                return sorted(self._bridge.live_node_names())
            except Exception:
                pass
        try:
            r = subprocess.run(["ros2", "node", "list"],
                               capture_output=True, text=True, timeout=4)
            return [n.strip() for n in r.stdout.splitlines() if n.strip()]
        except Exception:
            return []

    @staticmethod
    def _running_cmdlines() -> list:
        """Snapshot every process's full command line by reading /proc once.

        Replaces the per-pattern `pgrep -f` subprocesses (which blocked the GUI
        thread): one in-process scan is reused for all preset patterns. Each
        cmdline is the NUL-joined argv flattened to a space-separated string,
        matching what `pgrep -f` sees.
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
    def _proc_running(pattern: str, cmdlines: list) -> bool:
        """True if `pattern` occurs in any process command line.

        `pgrep -f re.escape(pattern)` is a literal-substring match over the
        full cmdline, so a plain substring test reproduces it exactly without
        spawning a subprocess.
        """
        return any(pattern in c for c in cmdlines)

    def _detect_external(self, preset: dict, cmdlines: list) -> bool:
        """True if this target appears to be running anywhere on the system.

        `cmdlines` is a single /proc snapshot (see _running_cmdlines) reused
        across all presets in one refresh pass.
        """
        for node in preset.get("nodes", []):
            if any(node == n or n.endswith(node) for n in self._node_cache):
                return True
        for pat in preset.get("procs", []):
            if self._proc_running(pat, cmdlines):
                return True
        return False

    # ------------------------------------------------------------------
    # Start / Stop
    # ------------------------------------------------------------------

    def _preset(self, key: str) -> dict:
        return next(p for p in PRESETS if p["key"] == key)

    def _start(self, key: str) -> None:
        proc = self._procs[key]
        preset = self._preset(key)
        if proc.running:
            self._set_status(f"{preset['label']}: already running (this tab).")
            return

        # Duplicate detection: same launch/node already up elsewhere?
        self._node_cache = self._query_nodes()
        if self._detect_external(preset, self._running_cmdlines()):
            ans = QMessageBox.question(
                self, "이미 실행 중",
                f"'{preset['label']}' 와(과) 동일한 launch/노드가 이미 실행 중인 것으로 "
                f"감지되었습니다 (다른 터미널/탭에서 실행됨).\n\n"
                f"그래도 새로 실행하시겠습니까? (중복 실행은 컨트롤러 충돌을 일으킬 수 있습니다)",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if ans != QMessageBox.Yes:
                self._set_status(f"{preset['label']}: 실행 취소 (중복 감지).")
                return

        if preset.get("confirm"):
            ans = QMessageBox.warning(
                self, "실행 확인",
                f"'{preset['label']}' 을(를) 실행합니다.\n"
                f"실제 하드웨어/모터가 동작할 수 있습니다. 안전을 확인했습니까?\n\n"
                f"{' '.join(preset['cmd'])}",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if ans != QMessageBox.Yes:
                return

        if proc.start(preset["cmd"]):
            self._started_keys.add(key)
            self._set_status(f"Started: {preset['label']} → {proc.log_path}")
        else:
            self._set_status(f"{preset['label']}: already running.")
        self._refresh()

    def _start_custom(self) -> None:
        text = self._custom_edit.text().strip()
        if not text:
            return
        proc = self._procs[CUSTOM_KEY]
        if proc.running:
            self._set_status("Custom: already running (this tab).")
            return
        if proc.start(text.split()):
            pat = self._launch_signature(text.split())
            if pat:
                self._custom_patterns.add(pat)
            self._set_status(f"Started custom: {text} → {proc.log_path}")
        else:
            self._set_status("Custom: already running.")
        self._refresh()

    def _stop(self, key: str) -> None:
        proc = self._procs[key]
        proc.stop()
        # Also sweep by launch signature so the target is gone even if it left
        # orphans (parent already exited) or was started outside this handle.
        if key == CUSTOM_KEY:
            for pat in self._custom_patterns:
                self._kill_pattern(pat)
        else:
            preset = self._preset(key)
            for pat in preset.get("procs", []) + preset.get("sweep", []):
                self._kill_pattern(pat)
        label = "Custom" if key == CUSTOM_KEY else self._preset(key)["label"]
        self._set_status(f"Stopped: {label}")
        self._refresh()

    def _on_gravity_toggled(self, checked: bool) -> None:
        """Gravity-compensation checkbox: start the launch on check, stop it on
        uncheck. _refresh re-syncs the checkbox to the real node state, so a
        cancelled start (duplicate prompt) or external start self-corrects."""
        key = "gravity_comp"
        if checked:
            # _start does duplicate detection (warns if gravity_comp_node is
            # already up elsewhere). Needs L1 controllers — the launch's spawner
            # loads the forward_effort_controllers onto controller_manager.
            self._start(key)
        else:
            # Clear the feedforward FIRST: set enable_compensation=false so the
            # node publishes zeros (effort controllers output 0), THEN stop the
            # launch. The effort controllers persist (no --unload-on-kill), but
            # they now output 0 → no stale motor torque. This is the sole stop
            # safety; do NOT rely on the spawner to unload (it may already be gone).
            try:
                subprocess.run(
                    ["ros2", "param", "set", "/gravity_comp_node",
                     "enable_compensation", "false"],
                    capture_output=True, timeout=5)
            except Exception:
                pass
            time.sleep(0.3)
            self._stop(key)

    def _stop_all(self) -> None:
        # [종료 낙하 방지 2026-06-07] HW proc 를 내리기 전에 팔을 HOME 으로 보내 도착 대기.
        # proc.stop() 이 컨트롤러를 죽이면 토크가 끊겨 팔이 낙하하기 때문. (컨트롤러 active 시만)
        if self._bridge is not None:
            try:
                self._bridge.park_arms_home(tick=QApplication.processEvents)
            except Exception:
                pass
        for proc in self._procs.values():
            proc.stop()
        self._set_status("Stop All (this tab) executed.")
        self._refresh()

    # ------------------------------------------------------------------
    # Status refresh
    # ------------------------------------------------------------------

    def _refresh(self) -> None:
        self._node_cache = self._query_nodes()
        cmdlines = self._running_cmdlines()
        for key, (_start, _stop, status) in self._rows.items():
            preset = self._preset(key)
            if self._procs[key].running:
                status.setText("● Running (this tab)")
                status.setStyleSheet("color:#080; font-weight:bold;")
            elif self._detect_external(preset, cmdlines):
                status.setText("● Running (external)")
                status.setStyleSheet("color:#d80;")
            else:
                status.setText("● Stopped")
                status.setStyleSheet("color:gray;")
        # custom
        if self._procs[CUSTOM_KEY].running:
            self._custom_status.setText("● Running (this tab)")
            self._custom_status.setStyleSheet("color:#080; font-weight:bold;")
        else:
            self._custom_status.setText("● Stopped")
            self._custom_status.setStyleSheet("color:gray;")
        # gravity comp checkbox (rendered outside _rows) — keep the toggle in
        # sync with the real node state; blockSignals so setChecked doesn't
        # re-fire _on_gravity_toggled.
        if hasattr(self, "_gravity_check"):
            gp = self._preset("gravity_comp")
            this_tab = self._procs["gravity_comp"].running
            running = this_tab or self._detect_external(gp, cmdlines)
            self._gravity_check.blockSignals(True)
            self._gravity_check.setChecked(running)
            self._gravity_check.blockSignals(False)
            if this_tab:
                self._gravity_status.setText("● Running (this tab)")
                self._gravity_status.setStyleSheet("color:#080; font-weight:bold;")
            elif running:
                self._gravity_status.setText("● Running (external)")
                self._gravity_status.setStyleSheet("color:#d80;")
            else:
                self._gravity_status.setText("● Stopped")
                self._gravity_status.setStyleSheet("color:gray;")

    def _set_status(self, text: str) -> None:
        ts = datetime.now().strftime("%H:%M:%S")
        self._lbl_status.setText(f"{ts}  {text}")

    def shutdown(self) -> None:
        self._timer.stop()
        for proc in self._procs.values():
            proc.stop()
        self._sweep_launched()

    @staticmethod
    def _launch_signature(argv: list) -> str:
        """Distinctive token to match a launched process (launch file or exe)."""
        for tok in argv:
            if tok.endswith(".launch.py") or tok.endswith(".launch.xml"):
                return tok
        if "run" in argv:
            i = argv.index("run")
            if i + 2 < len(argv):
                return argv[i + 2]
        return ""

    def _sweep_launched(self) -> None:
        """On exit, kill any process still alive from launches THIS tab started.
        ros2 exposes no node->PID map, so we find them by their launch/run
        process signature (the same patterns used for duplicate detection)."""
        patterns = set(self._custom_patterns)
        for key in self._started_keys:
            preset = self._preset(key)
            patterns.update(preset.get("procs", []))
            patterns.update(preset.get("sweep", []))
        for pat in patterns:
            self._kill_pattern(pat)

    def _kill_pattern(self, pattern: str) -> None:
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
        # Resolve to process GROUPS so launch children (controller_manager,
        # robot_state_publisher, rviz, spawners) die with their parent — and
        # even orphaned children (parent already gone) are reaped via their
        # surviving group id.
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
        for p in pids:  # belt-and-suspenders for stragglers
            try:
                os.kill(p, signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass
