"""Box-style scenario player window.

Load a scenario.json, render each sub-scenario as a numbered button, click a
button to play only that sub via the existing ScenarioRosBridge.

Expects the hardware bringup (full.launch.py) and scenario_player to be
started externally in separate terminals before using this UI.
"""

from __future__ import annotations

import json
import os
from datetime import datetime
from typing import List, Tuple

from ament_index_python.packages import get_package_share_directory
from PyQt5 import uic
from PyQt5.QtWidgets import (
    QFileDialog,
    QGridLayout,
    QMainWindow,
    QMessageBox,
    QPushButton,
)

from tr_works_scenario_ui.scenario_action_client import ScenarioRosBridge


BOX_COLUMNS = 4
BOX_STYLE = (
    "QPushButton { background-color:#eef6ff; border:1px solid #88a; "
    "border-radius:6px; padding:12px; font-weight:bold; text-align:center; } "
    "QPushButton:hover { background-color:#dce9f7; } "
    "QPushButton:pressed { background-color:#c8d9ec; } "
    "QPushButton:disabled { background-color:#eee; color:#888; }"
)
BOX_ACTIVE_STYLE = (
    "QPushButton { background-color:#ffe; border:2px solid #c80; "
    "border-radius:6px; padding:12px; font-weight:bold; text-align:center; }"
)


class ScenarioBoxWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        ui_path = os.path.join(
            get_package_share_directory("tr_works_scenario_ui"),
            "ui", "scenario_box_ui.ui",
        )
        uic.loadUi(ui_path, self)

        self._bridge = ScenarioRosBridge(parent=self)
        self._bridge.sig_status_event.connect(self._on_status_event)
        self._bridge.sig_feedback.connect(self._on_feedback)
        self._bridge.sig_result.connect(self._on_result)
        self._bridge.sig_error.connect(self._on_error)

        self.btnLoad.clicked.connect(self._on_load_clicked)
        self.btnCancel.clicked.connect(self._on_cancel_clicked)
        self.btnClearLog.clicked.connect(self.txtLog.clear)

        self._scenario_path: str = ""
        self._scenario_name: str = ""
        self._motions: List[str] = []
        self._buttons: List[QPushButton] = []
        self._active_index: int = -1

        self._log("Ready — load scenario.json (HW + scenario_player should already be running).")

    # ------------------------------------------------------------------
    # Load scenario
    # ------------------------------------------------------------------

    def _on_load_clicked(self) -> None:
        start_dir = self._default_start_dir()
        path, _ = QFileDialog.getOpenFileName(
            self, "Select scenario.json", start_dir,
            "Scenario JSON (scenario.json *.json);;All files (*)",
        )
        if not path:
            return
        try:
            motions, name = self._parse_scenario(path)
        except Exception as e:
            QMessageBox.critical(self, "Load failed", str(e))
            self._log(f"[ERROR] Load failed: {e}")
            return

        self._scenario_path = path
        self._scenario_name = name
        self._motions = motions
        self.lblScenarioName.setText(name)
        self.lblScenarioPath.setText(f"Path: {path}")
        self._log(f"Loaded '{name}' with {len(motions)} motions.")
        self._rebuild_boxes()

    def _default_start_dir(self) -> str:
        if self._scenario_path and os.path.isfile(self._scenario_path):
            return os.path.dirname(self._scenario_path)
        env = os.environ.get("TR_WORKS_SCENARIOS_DIR")
        if env and os.path.isdir(env):
            return env
        here = os.path.abspath(__file__)
        for _ in range(10):
            here = os.path.dirname(here)
            cand = os.path.join(here, "Scenarios")
            if os.path.isdir(cand):
                return cand
        return os.path.expanduser("~")

    @staticmethod
    def _parse_scenario(path: str) -> Tuple[List[str], str]:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        if not isinstance(data, dict):
            raise ValueError("scenario.json root must be an object")
        name = str(data.get("name") or os.path.basename(os.path.dirname(path)) or "scenario")
        seq = data.get("sequence")
        if isinstance(seq, list) and seq:
            return [str(x) for x in seq], name

        if isinstance(data.get("steps"), list):
            sibling = os.path.join(os.path.dirname(path), "scenario.json")
            hint = (
                f"\n\nTry loading:\n  {sibling}"
                if os.path.isfile(sibling) and os.path.abspath(sibling) != os.path.abspath(path)
                else ""
            )
            raise ValueError(
                "This file looks like a sub-scenario (has 'steps' but no "
                "'sequence'). scenario_player can only play a top-level "
                "scenario.json with a 'sequence' list." + hint
            )
        raise ValueError(
            "No 'sequence' array found — pick a scenario.json that defines "
            "'sequence': [...]."
        )

    # ------------------------------------------------------------------
    # Box grid
    # ------------------------------------------------------------------

    def _rebuild_boxes(self) -> None:
        grid: QGridLayout = self.boxesGrid
        while grid.count():
            item = grid.takeAt(0)
            w = item.widget()
            if w is not None:
                w.setParent(None)
                w.deleteLater()
        self._buttons.clear()
        self._active_index = -1

        for i, motion in enumerate(self._motions, start=1):
            btn = QPushButton(f"{i}\n{motion}")
            btn.setMinimumHeight(70)
            btn.setStyleSheet(BOX_STYLE)
            btn.clicked.connect(lambda _=False, idx=i: self._on_box_clicked(idx))
            row, col = divmod(i - 1, BOX_COLUMNS)
            grid.addWidget(btn, row, col)
            self._buttons.append(btn)

    def _on_box_clicked(self, idx: int) -> None:
        if not self._scenario_path:
            QMessageBox.warning(self, "Play", "Load a scenario first.")
            return
        if self._active_index > 0:
            QMessageBox.information(
                self, "Busy",
                f"Motion {self._active_index} is still running — cancel first.",
            )
            return
        motion = self._motions[idx - 1]
        speed = self._current_speed()
        dry_run = self.chkDryRun.isChecked()
        self._log(
            f"Play motion {idx}: '{motion}'  speed={speed}  dry_run={dry_run}"
        )
        self.prgOverall.setValue(0)
        self.lblProgressText.setText(f"Starting motion {idx}: {motion}")
        ok = self._bridge.play(self._scenario_path, idx, idx, speed, dry_run)
        if not ok:
            self._log("[ERROR] Play call failed (is scenario_player running?)")
            self.lblProgressText.setText("❌ Play call failed")
            return
        self._mark_active(idx)

    def _current_speed(self) -> float:
        text = self.cmbSpeed.currentText().strip().rstrip("xX")
        try:
            return float(text)
        except ValueError:
            return 0.5

    def _mark_active(self, idx: int) -> None:
        self._active_index = idx
        for i, btn in enumerate(self._buttons, start=1):
            if i == idx:
                btn.setStyleSheet(BOX_ACTIVE_STYLE)
            else:
                btn.setStyleSheet(BOX_STYLE)
                btn.setEnabled(False)

    def _clear_active(self) -> None:
        self._active_index = -1
        for btn in self._buttons:
            btn.setStyleSheet(BOX_STYLE)
            btn.setEnabled(True)

    # ------------------------------------------------------------------
    # Cancel
    # ------------------------------------------------------------------

    def _on_cancel_clicked(self) -> None:
        if self._active_index <= 0:
            self._log("Cancel: nothing is running.")
            return
        self._log(f"Cancel motion {self._active_index} requested.")
        self._bridge.cancel()

    # ------------------------------------------------------------------
    # Bridge slots
    # ------------------------------------------------------------------

    def _on_status_event(self, payload: dict) -> None:
        ev = payload.get("event", "?")
        if ev == "step":
            self._log(
                f"[STATUS] step sub={payload.get('sub')} '{payload.get('sub_name')}' "
                f"step='{payload.get('step_name')}' "
                f"({payload.get('duration_sec', 0):.1f}s)"
            )
        elif ev in ("started", "completed", "canceled", "failed", "idle"):
            self._log(f"[STATUS] {ev}: {payload}")

    def _on_feedback(self, fb: dict) -> None:
        pct = int(fb["overall_progress"] * 100)
        self.prgOverall.setValue(pct)
        self.lblProgressText.setText(
            f"Motion {self._active_index} '{fb['sub_name']}'  "
            f"step {fb['current_step'] + 1}/{fb['total_steps']} "
            f"'{fb['step_name']}'  [{fb['phase']}]  "
            f"elapsed={fb['elapsed_sec']:.1f}s"
        )

    def _on_result(self, r: dict) -> None:
        idx = self._active_index
        self._log(
            f"Result motion {idx}: success={r['success']}  "
            f"steps={r['steps_completed']}  "
            f"duration={r['total_duration_sec']:.2f}s  "
            f"msg={r.get('message', '')}"
        )
        if r["success"]:
            self.prgOverall.setValue(100)
            self.lblProgressText.setText(f"✅ Motion {idx} completed")
        else:
            self.lblProgressText.setText(f"❌ Motion {idx} failed / canceled")
        self._clear_active()

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
        self._bridge.shutdown()
        event.accept()
