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
    QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit, QMessageBox,
    QPushButton, QScrollArea, QTextEdit, QVBoxLayout, QWidget,
)

from openarmx_scenario_ui.managed_process import ManagedProcess


# Each preset: key, label, cmd (argv list), confirm (warn before start),
# nodes (ros2 node names that indicate it is running),
# procs (pgrep -f patterns that indicate it is running).
# Core targets for the scenario workflow only.
PRESETS = [
    {
        "key": "sil_bringup",
        "label": "SIL Bringup (fake HW + ctrl)",
        "cmd": ["ros2", "launch", "openarmx_bringup", "openarmx.bimanual.launch.py",
                "use_fake_hardware:=true",
                "robot_controller:=joint_trajectory_controller",
                "start_rviz:=false"],
        "confirm": False,
        "nodes": [],
        # specific arg distinguishes SIL from HW (both share openarmx.bimanual.launch)
        "procs": ["use_fake_hardware:=true"],
        # child node executables to also reap on stop (orphan-proof cleanup)
        "sweep": ["robot_state_publisher", "ros2_control_node", "rviz2"],
    },
    {
        "key": "hw_bringup",
        "label": "HW Bringup (real robot)",
        "cmd": ["ros2", "launch", "openarmx_bringup", "openarmx.bimanual.launch.py",
                "use_fake_hardware:=false",
                "robot_controller:=joint_trajectory_controller",
                "control_mode:=mit",
                "right_can_interface:=can0", "left_can_interface:=can1",
                "can_fd:=false",
                "start_rviz:=false"],
        "confirm": True,
        "nodes": [],
        "procs": ["use_fake_hardware:=false"],
        "sweep": ["robot_state_publisher", "ros2_control_node", "rviz2"],
    },
    {
        "key": "moveit_demo",
        "label": "MoveIt Demo (move_group + RViz)",
        "cmd": ["ros2", "launch", "openarmx_bimanual_moveit_config", "demo.launch.py"],
        "confirm": False,
        "nodes": ["/move_group"],
        "procs": ["demo.launch"],
    },
    {
        "key": "scenario_player",
        "label": "Scenario Player node (backend only)",
        "cmd": ["ros2", "run", "openarmx_scenario_player", "scenario_player_node.py"],
        "confirm": False,
        "nodes": [],
        "procs": ["scenario_player_node"],
    },
]
# NOTE: trimmed to the 4 core scenario-workflow targets. Other launches/nodes
# (Display/RViz, MoveIt variants, Preview Bringup, teach/teleop/gravity/lerobot)
# can be run ad-hoc via the Custom command row.

CUSTOM_KEY = "custom"


class LaunchManagerTab(QWidget):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
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
        grid.addWidget(QLabel("<b>Target</b>"), 0, 0)
        grid.addWidget(QLabel("<b>Start</b>"), 0, 1)
        grid.addWidget(QLabel("<b>Stop</b>"), 0, 2)
        grid.addWidget(QLabel("<b>Status</b>"), 0, 3)

        for row, preset in enumerate(PRESETS, start=1):
            key = preset["key"]
            grid.addWidget(QLabel(preset["label"]), row, 0)
            start = QPushButton("Start")
            stop = QPushButton("Stop")
            stop.setStyleSheet("color:#c00;")
            status = QLabel("● Stopped")
            status.setStyleSheet("color:gray;")
            start.clicked.connect(lambda _c, k=key: self._start(k))
            stop.clicked.connect(lambda _c, k=key: self._stop(k))
            grid.addWidget(start, row, 1)
            grid.addWidget(stop, row, 2)
            grid.addWidget(status, row, 3)
            self._rows[key] = (start, stop, status)
        grid.setColumnStretch(0, 1)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(grp)
        scroll.setMinimumHeight(320)
        root.addWidget(scroll, stretch=1)

        # Custom command row
        cgrp = QGroupBox("Custom command")
        crow = QHBoxLayout(cgrp)
        self._custom_edit = QLineEdit()
        self._custom_edit.setPlaceholderText(
            "ros2 launch <pkg> <file>.launch.py  /  ros2 run <pkg> <exe>")
        cstart = QPushButton("Start")
        cstop = QPushButton("Stop")
        cstop.setStyleSheet("color:#c00;")
        self._custom_status = QLabel("● Stopped")
        self._custom_status.setStyleSheet("color:gray;")
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
        self.btn_refresh = QPushButton("Refresh node list")
        self.btn_refresh.clicked.connect(self._refresh)
        ctl.addWidget(self.btn_stop_all)
        ctl.addWidget(self.btn_refresh)
        ctl.addStretch()
        root.addLayout(ctl)

        root.addWidget(QLabel("Active ROS2 nodes (ros2 node list):"))
        self._node_view = QTextEdit()
        self._node_view.setReadOnly(True)
        self._node_view.setMaximumHeight(160)
        root.addWidget(self._node_view)

        self._lbl_status = QLabel("Ready")
        self._lbl_status.setStyleSheet("color:#444; padding:2px;")
        root.addWidget(self._lbl_status)

    # ------------------------------------------------------------------
    # Detection
    # ------------------------------------------------------------------

    def _query_nodes(self) -> list:
        """Snapshot of `ros2 node list` (best-effort, short timeout)."""
        try:
            r = subprocess.run(["ros2", "node", "list"],
                               capture_output=True, text=True, timeout=4)
            return [n.strip() for n in r.stdout.splitlines() if n.strip()]
        except Exception:
            return []

    def _proc_running(self, pattern: str) -> bool:
        # Escape regex metacharacters so dots/dashes match literally
        # (pgrep -f treats the pattern as an extended regex).
        literal = re.escape(pattern)
        try:
            r = subprocess.run(["pgrep", "-f", literal],
                               capture_output=True, text=True, timeout=3)
            return r.returncode == 0 and bool(r.stdout.strip())
        except Exception:
            return False

    def _detect_external(self, preset: dict) -> bool:
        """True if this target appears to be running anywhere on the system."""
        for node in preset.get("nodes", []):
            if any(node == n or n.endswith(node) for n in self._node_cache):
                return True
        for pat in preset.get("procs", []):
            if self._proc_running(pat):
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
        if self._detect_external(preset):
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

    def _stop_all(self) -> None:
        for proc in self._procs.values():
            proc.stop()
        self._set_status("Stop All (this tab) executed.")
        self._refresh()

    # ------------------------------------------------------------------
    # Status refresh
    # ------------------------------------------------------------------

    def _refresh(self) -> None:
        self._node_cache = self._query_nodes()
        for key, (_start, _stop, status) in self._rows.items():
            preset = self._preset(key)
            if self._procs[key].running:
                status.setText("● Running (this tab)")
                status.setStyleSheet("color:#080; font-weight:bold;")
            elif self._detect_external(preset):
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
        self._node_view.setPlainText(
            "\n".join(self._node_cache) if self._node_cache else "(no nodes / ros2 unavailable)")

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
