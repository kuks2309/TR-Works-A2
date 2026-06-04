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

from openarmx_scenario_ui.camera_tab import CameraTab
from openarmx_scenario_ui.cartesian_control_tab import CartesianControlTab
from openarmx_scenario_ui.diagnostics_tab import DiagnosticsTab
from openarmx_scenario_ui.joint_control_tab import JointControlTab
from openarmx_scenario_ui.launch_manager_tab import LaunchManagerTab
from openarmx_scenario_ui.managed_process import ManagedProcess
from openarmx_scenario_ui.pick_and_place_tab import PickAndPlaceTab
from openarmx_scenario_ui.scenario_action_client import ScenarioRosBridge
from openarmx_scenario_ui.teaching_tab import TeachingTab


HW_LAUNCH_CMD = [
    "ros2", "launch", "openarmx_bringup", "openarmx.bimanual.launch.py",
    "use_fake_hardware:=false",
    "robot_controller:=joint_trajectory_controller",
    "control_mode:=mit",
    # follower 팔 = can2(right)/can3(left). can0/can1 은 leader(텔레옵 입력).
    "right_can_interface:=can2",
    "left_can_interface:=can3",
    "can_fd:=false",
    # The scenario_player workflow launch (_scenario_workflow_cmd) already owns
    # the RViz instance (openarmx_scenario.rviz). Without this, HW bringup also
    # spawns its own RViz (bimanual.rviz) → two RViz windows on a2-scenario.
    "start_rviz:=false",
]
PLAYER_RUN_CMD = ["ros2", "run", "openarmx_scenario_player", "scenario_player_node.py"]


def _scenario_workflow_cmd() -> list:
    """RViz + ee_leader markers. ALWAYS spawned regardless of --follower —
    a missing cyclo dependency must not take down RViz."""
    return ["ros2", "launch", "openarmx_scenario_player",
            "openarmx_scenario_workflow.launch.py"]


def _cyclo_extras_cmd() -> list:
    """Optional cyclo_sim + vr_controller (QP+CBF follower). Only spawned
    when --follower=cyclo. Requires cyclo_motion_controller_ros built."""
    return ["ros2", "launch", "openarmx_scenario_player",
            "openarmx_cyclo_extras.launch.py"]


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
    def __init__(self, auto_start: bool = True, with_rviz: bool = True,
                 follower: str = "cyclo") -> None:
        super().__init__()
        ui_path = os.path.join(
            get_package_share_directory("openarmx_scenario_ui"),
            "ui", "scenario_ui.ui",
        )
        uic.loadUi(ui_path, self)

        self._hw = ManagedProcess("hardware_bringup")
        self._player = ManagedProcess("scenario_player")
        self._workflow = ManagedProcess("scenario_workflow")
        self._cyclo_extras = ManagedProcess("cyclo_extras")
        self._bridge = ScenarioRosBridge(parent=self)

        # RViz + ee_leader markers (always — never gated by follower mode).
        # scenario_player is NOT auto-started — it's only needed AFTER a
        # scenario JSON is registered. Pass --no-rviz to opt out.
        self._follower = follower
        if with_rviz:
            try:
                self._workflow.start(_scenario_workflow_cmd())
            except Exception:
                pass
            # follower=cyclo: also spawn cyclo_sim + vr_controller. Failure
            # here (e.g. missing cyclo_motion_controller_ros) does NOT take
            # down RViz/markers — those already started above.
            if follower == "cyclo":
                try:
                    self._cyclo_extras.start(_cyclo_extras_cmd())
                except Exception:
                    pass

        # ---- wrap the loaded scenario widget + Joint Control into tabs ----
        scenario_widget = self.takeCentralWidget()
        self._tabs = QTabWidget()
        self._tabs.addTab(scenario_widget, "Scenario Player")
        self._joint_tab = JointControlTab(self._bridge, parent=self)
        self._tabs.addTab(self._joint_tab, "Joint Control")
        self._cart_tab = CartesianControlTab(self._bridge, parent=self)
        self._tabs.addTab(self._cart_tab, "Cartesian Control")
        self._teaching_tab = TeachingTab(self._bridge, parent=self)
        self._tabs.addTab(self._teaching_tab, "Teaching")
        self._pnp_tab = PickAndPlaceTab(parent=self)
        self._tabs.addTab(self._pnp_tab, "Pick and Place")
        self._camera_tab = CameraTab(self._bridge, parent=self)
        self._tabs.addTab(self._camera_tab, "Camera")
        self._launch_tab = LaunchManagerTab(parent=self)
        self._tabs.addTab(self._launch_tab, "Launch Manager")
        # Diagnostics split into two focused tabs (same DiagnosticsTab widget,
        # filtered): Node Health = node alive/down + raw node list;
        # Pipe Health = topic publish rate / latched / descriptions.
        self._node_health_tab = DiagnosticsTab(self._bridge, parent=self, show="node")
        self._tabs.addTab(self._node_health_tab, "Node Health")
        self._pipe_health_tab = DiagnosticsTab(self._bridge, parent=self, show="topic")
        self._tabs.addTab(self._pipe_health_tab, "Pipe Health")
        self.setCentralWidget(self._tabs)
        # Minimum size pinned to the user's current working window size so the
        # UI cannot be shrunk below the point where tab headers / combos start
        # truncating. Cartesian Jog has step + velocity rows per arm group.
        self.setMinimumSize(1962, 1365)
        self.resize(1962, 1365)

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

    @staticmethod
    def _clear_ros2_daemon() -> None:
        """Flush the ros2 discovery daemon so SIGKILL'd nodes stop showing up
        as ghosts in `ros2 node list`.

        A hard-killed node never destroys its DDS participant, so the daemon
        keeps advertising it (this is the "shares an exact name" warning and
        the duplicate /robot_state_publisher after a restart). `ros2 daemon
        stop` is non-destructive — it does NOT touch any live node; the next
        ros2 command transparently restarts the daemon and re-discovers only
        the participants that are actually still alive.
        """
        try:
            subprocess.run(
                ["ros2", "daemon", "stop"], timeout=8.0, check=False,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )
        except Exception:
            pass

    def closeEvent(self, event) -> None:
        self._status_timer.stop()
        self._joint_tab.shutdown()
        self._cart_tab.shutdown()
        self._teaching_tab.shutdown()
        self._pnp_tab.shutdown()
        self._camera_tab.shutdown()
        self._node_health_tab.shutdown()
        self._pipe_health_tab.shutdown()
        self._launch_tab.shutdown()
        self._hw.stop()
        self._player.stop()
        self._workflow.stop()
        self._cyclo_extras.stop()
        self._bridge.shutdown()
        # Processes are down — now flush the daemon so the ones we SIGKILL'd
        # don't linger as ghost nodes in `ros2 node list`.
        self._clear_ros2_daemon()
        event.accept()
