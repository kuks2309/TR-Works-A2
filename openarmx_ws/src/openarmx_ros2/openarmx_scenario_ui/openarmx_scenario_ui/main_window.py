"""Main window for openarmx_scenario_ui — loads ui/scenario_ui.ui and wires
ManagedProcess buttons + ScenarioRosBridge action client.
"""

from __future__ import annotations

import os
import subprocess
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from PyQt5 import uic
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QFileDialog, QMainWindow, QMessageBox, QTabWidget,
)

from openarmx_scenario_ui.cartesian_control_tab import CartesianControlTab
from openarmx_scenario_ui.diagnostics_tab import DiagnosticsTab
from openarmx_scenario_ui.joint_control_tab import JointControlTab
from openarmx_scenario_ui.launch_manager_tab import LaunchManagerTab
from openarmx_scenario_ui.managed_process import ManagedProcess
from openarmx_scenario_ui.scenario_action_client import ScenarioRosBridge


HW_LAUNCH_CMD = [
    "ros2", "launch", "openarmx_bringup", "openarmx.bimanual.launch.py",
    "use_fake_hardware:=false",
    "robot_controller:=joint_trajectory_controller",
    "control_mode:=mit",
    "right_can_interface:=can0",
    "left_can_interface:=can1",
    "can_fd:=false",
]
PLAYER_RUN_CMD = ["ros2", "run", "openarmx_scenario_player", "scenario_player_node.py"]
KILL_ALL_SCRIPT = os.path.expanduser("~/kill_all_ros2.sh")


def _find_scenarios_dir() -> str:
    """Resolve the default Scenarios directory.

    Order: $OPENARMX_SCENARIOS_DIR → ~/openarmx_ws/scenarios →
    10-level walk up from this file looking for scenarios/ or Scenarios/.
    """
    env = os.environ.get("OPENARMX_SCENARIOS_DIR")
    if env and os.path.isdir(env):
        return env

    home_default = os.path.expanduser("~/openarmx_ws/scenarios")
    if os.path.isdir(home_default):
        return home_default

    here = os.path.abspath(__file__)
    for _ in range(10):
        here = os.path.dirname(here)
        for name in ("scenarios", "Scenarios"):
            candidate = os.path.join(here, name)
            if os.path.isdir(candidate):
                return candidate
    return home_default


AUTO_HW_DELAY_MS = 500
AUTO_PLAYER_DELAY_MS = 5000
AUTO_REFRESH_DELAY_MS = 10000


class ScenarioMainWindow(QMainWindow):
    def __init__(self, auto_start: bool = True) -> None:
        super().__init__()
        ui_path = os.path.join(
            get_package_share_directory("openarmx_scenario_ui"),
            "ui", "scenario_ui.ui",
        )
        uic.loadUi(ui_path, self)

        self._hw = ManagedProcess("hardware_bringup")
        self._player = ManagedProcess("scenario_player")
        self._bridge = ScenarioRosBridge(parent=self)

        # ---- wrap the loaded scenario widget + Joint Control into tabs ----
        scenario_widget = self.takeCentralWidget()
        self._tabs = QTabWidget()
        self._tabs.addTab(scenario_widget, "Scenario Player")
        self._joint_tab = JointControlTab(self._bridge, parent=self)
        self._tabs.addTab(self._joint_tab, "Joint Control")
        self._cart_tab = CartesianControlTab(self._bridge, parent=self)
        self._tabs.addTab(self._cart_tab, "Cartesian Control")
        self._launch_tab = LaunchManagerTab(parent=self)
        self._tabs.addTab(self._launch_tab, "Launch Manager")
        self._diag_tab = DiagnosticsTab(self._bridge, parent=self)
        self._tabs.addTab(self._diag_tab, "Diagnostics")
        self.setCentralWidget(self._tabs)
        self.resize(1080, 840)

        # ---- bridge signals ----
        self._bridge.sig_status_event.connect(self._on_status_event)
        self._bridge.sig_feedback.connect(self._on_feedback)
        self._bridge.sig_result.connect(self._on_result)
        self._bridge.sig_error.connect(self._on_error)

        # ---- button signals ----
        self.btnHwStart.clicked.connect(self._start_hw)
        self.btnHwStop.clicked.connect(self._stop_hw)
        self.btnPlayerStart.clicked.connect(self._start_player)
        self.btnPlayerStop.clicked.connect(self._stop_player)
        self.btnStopAll.clicked.connect(self._stop_all)
        self.btnBrowsePath.clicked.connect(self._browse_scenario_path)
        self.btnRefresh.clicked.connect(self._refresh_scenarios)
        self.cmbScenario.currentIndexChanged.connect(self._on_scenario_changed)
        self.btnPlay.clicked.connect(self._play)
        self.btnCancel.clicked.connect(self._cancel)
        self.btnClearLog.clicked.connect(self.txtLog.clear)

        # ---- default scenario path ----
        self.txtScenarioPath.setText(_find_scenarios_dir())

        # ---- periodic status refresh ----
        self._status_timer = QTimer(self)
        self._status_timer.timeout.connect(self._refresh_proc_status)
        self._status_timer.start(1500)

        self._refresh_proc_status()

        # ---- auto-start sequence ----
        if auto_start:
            self._log("Auto-start enabled — launching HW → Player → Refresh ...")
            QTimer.singleShot(AUTO_HW_DELAY_MS, self._auto_start_hw)
        else:
            self._log("UI ready — Start hardware + scenario player, then Refresh scenarios.")

    # ------------------------------------------------------------------
    # Auto-start sequence: HW (0.5s) → Player (5s) → Refresh (10s)
    # ------------------------------------------------------------------

    def _auto_start_hw(self) -> None:
        self._start_hw()
        self._log(f"  → Player will start in {AUTO_PLAYER_DELAY_MS // 1000}s ...")
        QTimer.singleShot(AUTO_PLAYER_DELAY_MS, self._auto_start_player)

    def _auto_start_player(self) -> None:
        self._start_player()
        self._log(f"  → Scenario refresh in {AUTO_REFRESH_DELAY_MS // 1000}s ...")
        QTimer.singleShot(AUTO_REFRESH_DELAY_MS, self._auto_refresh)

    _AUTO_REFRESH_RETRIES = 3
    _AUTO_REFRESH_RETRY_MS = 5000

    def _auto_refresh(self, attempt: int = 1) -> None:
        self._refresh_scenarios()
        if self.cmbScenario.count() > 0:
            self._log("Auto-start sequence complete.")
            return
        if attempt < self._AUTO_REFRESH_RETRIES:
            self._log(f"  → No scenarios yet, retry {attempt + 1}/{self._AUTO_REFRESH_RETRIES} "
                      f"in {self._AUTO_REFRESH_RETRY_MS // 1000}s ...")
            QTimer.singleShot(self._AUTO_REFRESH_RETRY_MS,
                              lambda: self._auto_refresh(attempt + 1))
        else:
            self._log("Auto-refresh exhausted — press Refresh manually when player is ready.")

    # ------------------------------------------------------------------
    # Path selection
    # ------------------------------------------------------------------

    def _browse_scenario_path(self) -> None:
        current = self.txtScenarioPath.text().strip()
        start = current if os.path.isdir(current) else os.path.expanduser("~")
        d = QFileDialog.getExistingDirectory(self, "Select Scenarios Directory", start)
        if d:
            self.txtScenarioPath.setText(d)
            self._log(f"Scenarios path set to: {d}")

    def _scenario_search_path(self) -> str:
        return self.txtScenarioPath.text().strip() or _find_scenarios_dir()

    # ------------------------------------------------------------------
    # Process control
    # ------------------------------------------------------------------

    def _start_hw(self) -> None:
        if self._hw.start(HW_LAUNCH_CMD):
            self._log(f"Hardware bringup started → log: {self._hw.log_path}")
        else:
            self._log("Hardware already running.")
        self._refresh_proc_status()

    def _stop_hw(self) -> None:
        self._hw.stop()
        self._log("Hardware bringup stopped.")
        self._refresh_proc_status()

    def _start_player(self) -> None:
        cmd = list(PLAYER_RUN_CMD) + [
            "--ros-args", "-p",
            f"scenario_search_path:={self._scenario_search_path()}",
        ]
        if self._player.start(cmd):
            self._log(f"Scenario player started (path={self._scenario_search_path()})")
            self._log(f"  → log: {self._player.log_path}")
        else:
            self._log("Scenario player already running.")
        self._refresh_proc_status()

    def _stop_player(self) -> None:
        self._player.stop()
        self._log("Scenario player stopped.")
        self._refresh_proc_status()

    def _stop_all(self) -> None:
        ans = QMessageBox.question(
            self, "Stop All",
            f"Run {KILL_ALL_SCRIPT} to terminate ALL ROS2 processes "
            "(incl. RViz, joint_control_ui, etc.)?",
        )
        if ans != QMessageBox.Yes:
            return
        try:
            subprocess.run(
                [KILL_ALL_SCRIPT], timeout=15.0, check=False,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )
        except Exception as e:
            self._log(f"{os.path.basename(KILL_ALL_SCRIPT)} failed: {e}")
        self._hw.stop()
        self._player.stop()
        self._log("STOP ALL executed.")
        self._refresh_proc_status()

    def _refresh_proc_status(self) -> None:
        if self._hw.running:
            self.lblHwStatus.setText("● Running")
            self.lblHwStatus.setStyleSheet("color:#080;")
        else:
            self.lblHwStatus.setText("● Stopped")
            self.lblHwStatus.setStyleSheet("color:gray;")
        if self._player.running:
            self.lblPlayerStatus.setText("● Running")
            self.lblPlayerStatus.setStyleSheet("color:#080;")
        else:
            self.lblPlayerStatus.setText("● Stopped")
            self.lblPlayerStatus.setStyleSheet("color:gray;")

    # ------------------------------------------------------------------
    # Scenario list / play / cancel
    # ------------------------------------------------------------------

    def _refresh_scenarios(self) -> None:
        self._log("Refreshing scenarios from /scenario_player/list ...")
        info = self._bridge.list_scenarios(timeout_sec=3.0)
        self.cmbScenario.clear()
        if not info or not info.get("scenarios"):
            self._log("(no scenarios — is scenario_player running?)")
            return
        for s in info["scenarios"]:
            self.cmbScenario.addItem(
                f"{s['name']}  ({s['sub_count']} subs)", userData=s)
        self._log(f"Found {len(info['scenarios'])} scenarios.")

    def _on_scenario_changed(self, _idx: int) -> None:
        s = self.cmbScenario.currentData()
        self.cmbSub.clear()
        self.cmbSub.addItem("All", userData=(0, -1))
        if not s:
            return
        for i, sub_name in enumerate(s["sub_names"], start=1):
            self.cmbSub.addItem(f"{i}: {sub_name}", userData=(i, i))

    def _play(self) -> None:
        s = self.cmbScenario.currentData()
        if not s:
            QMessageBox.warning(self, "Play", "Pick a scenario first (Refresh).")
            return
        sub_data = self.cmbSub.currentData() or (0, -1)
        start_sub, end_sub = sub_data
        speed = float(self.spnSpeed.value())
        dry_run = self.chkDryRun.isChecked()

        self._log(
            f"Play '{s['name']}' sub={start_sub}-{end_sub} speed={speed} dry_run={dry_run}")
        self.prgOverall.setValue(0)
        self.lblProgressText.setText("Starting ...")
        self._bridge.play(s["name"], start_sub, end_sub, speed, dry_run)

    def _cancel(self) -> None:
        self._log("Cancel requested.")
        self._bridge.cancel()

    # ------------------------------------------------------------------
    # ROS bridge slots
    # ------------------------------------------------------------------

    def _on_status_event(self, payload: dict) -> None:
        ev = payload.get("event", "?")
        if ev == "started":
            self._log(
                f"[STATUS] started: {payload.get('scenario')} "
                f"subs={payload.get('subs')} steps={payload.get('total_steps')}")
        elif ev == "step":
            self._log(
                f"[STATUS] step: sub={payload.get('sub')} '{payload.get('sub_name')}' "
                f"step={payload.get('step')} '{payload.get('step_name')}' "
                f"({payload.get('duration_sec'):.1f}s)")
        elif ev == "completed":
            self._log(
                f"[STATUS] completed: {payload.get('scenario')} "
                f"subs={payload.get('subs_completed')}/{payload.get('subs_total','?')} "
                f"steps={payload.get('steps_completed')} "
                f"duration={payload.get('duration_sec'):.2f}s")
        elif ev == "canceled":
            self._log(
                f"[STATUS] canceled at sub='{payload.get('sub_name')}' "
                f"step='{payload.get('step_name')}' phase={payload.get('phase')}")
        elif ev == "failed":
            self._log(
                f"[STATUS] failed at step='{payload.get('step_name')}' "
                f"reason={payload.get('reason')} msg={payload.get('message','')}")
        elif ev == "idle":
            self._log("[STATUS] player idle.")
        else:
            self._log(f"[STATUS] {ev}: {payload}")

    def _on_feedback(self, fb: dict) -> None:
        pct = int(fb["overall_progress"] * 100)
        self.prgOverall.setValue(pct)
        self.lblProgressText.setText(
            f"Sub {fb['current_sub']}/{fb['total_subs']} "
            f"'{fb['sub_name']}'  Step {fb['current_step']+1}/{fb['total_steps']} "
            f"'{fb['step_name']}'  [{fb['phase']}]  "
            f"elapsed={fb['elapsed_sec']:.1f}s")

    def _on_result(self, r: dict) -> None:
        msg = (f"Result: success={r['success']}  "
               f"subs={r['subs_completed']}/{r['subs_total']}  "
               f"steps={r['steps_completed']}  "
               f"duration={r['total_duration_sec']:.2f}s")
        self._log(msg)
        if r.get("message"):
            self._log(f"  → {r['message']}")
        if r["success"]:
            self.prgOverall.setValue(100)
            self.lblProgressText.setText("✅ Completed")
        else:
            self.lblProgressText.setText("❌ Failed / Canceled")

    def _on_error(self, msg: str) -> None:
        self._log(f"[ERROR] {msg}")

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _log(self, line: str) -> None:
        ts = datetime.now().strftime("%H:%M:%S")
        self.txtLog.append(f"{ts}  {line}")
        sb = self.txtLog.verticalScrollBar()
        sb.setValue(sb.maximum())

    def closeEvent(self, event) -> None:
        self._status_timer.stop()
        self._joint_tab.shutdown()
        self._cart_tab.shutdown()
        self._diag_tab.shutdown()
        self._launch_tab.shutdown()
        self._hw.stop()
        self._player.stop()
        self._bridge.shutdown()
        event.accept()
