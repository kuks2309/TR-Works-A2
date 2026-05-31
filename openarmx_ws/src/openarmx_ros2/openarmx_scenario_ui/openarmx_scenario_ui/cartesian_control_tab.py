"""Cartesian Control tab for openarmx_scenario_ui.

Drives MoveIt2's MoveGroup action (Pilz LIN/PTP/CIRC or OMPL) from Qt:
6-DOF discrete jog, manual delta/absolute pose targets, plan/execute, and
"Capture current pose as MoveL step" for scenario building.

Workflow reference: openarmx_bimanual_moveit_config/docs/RUN.md.
"""

from __future__ import annotations

import json
import math
import os
from datetime import datetime

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QButtonGroup, QComboBox, QDoubleSpinBox, QGridLayout, QGroupBox,
    QHBoxLayout, QInputDialog, QLabel, QMessageBox, QPushButton, QRadioButton,
    QVBoxLayout, QWidget,
)


LIN_STEPS_MM = [1, 5, 10, 50, 100]
ANG_STEPS_DEG = [1, 5, 10, 30]

ARMS = ["right", "left"]
PLANNERS = [
    ("PILZ_LIN", "Pilz LIN  (직선)"),
    ("PILZ_PTP", "Pilz PTP  (관절)"),
    ("PILZ_CIRC", "Pilz CIRC (원호)"),
    ("OMPL", "OMPL (default)"),
]


def _poses_dir() -> str:
    env = os.environ.get("OPENARMX_SCENARIOS_DIR")
    base = env if env and os.path.isdir(env) else os.path.expanduser(
        "~/openarmx_ws/scenarios")
    return os.path.join(base, "poses")


class CartesianControlTab(QWidget):
    def __init__(self, bridge, parent=None) -> None:
        super().__init__(parent)
        self._bridge = bridge
        self._current_pose = None
        self._busy = False

        self._build_ui()
        self._bridge.sig_motion_result.connect(self._on_motion_result)

        # auto-refresh current pose every 500ms (cheap TF lookup)
        self._pose_timer = QTimer(self)
        self._pose_timer.timeout.connect(self._refresh_pose)
        self._pose_timer.start(500)
        self._refresh_pose()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.addWidget(self._build_header())
        root.addWidget(self._build_current_pose())
        root.addWidget(self._build_jog())
        root.addWidget(self._build_target())
        root.addWidget(self._build_actions())
        self._lbl_status = QLabel("Ready")
        self._lbl_status.setStyleSheet(
            "color:#444; padding:4px; font-family:monospace;")
        root.addWidget(self._lbl_status)
        root.addStretch()

    def _build_header(self) -> QGroupBox:
        grp = QGroupBox("Arm / Planner / Frame")
        h = QHBoxLayout(grp)
        h.addWidget(QLabel("Arm:"))
        self.cmbArm = QComboBox()
        for a in ARMS:
            self.cmbArm.addItem(f"{a}_arm", userData=a)
        self.cmbArm.currentIndexChanged.connect(self._on_arm_changed)
        h.addWidget(self.cmbArm)

        h.addSpacing(20)
        h.addWidget(QLabel("Planner:"))
        self.cmbPlanner = QComboBox()
        for pid, label in PLANNERS:
            self.cmbPlanner.addItem(label, userData=pid)
        h.addWidget(self.cmbPlanner)

        h.addSpacing(20)
        h.addWidget(QLabel("Frame:"))
        self.cmbFrame = QComboBox()
        self.cmbFrame.currentIndexChanged.connect(self._refresh_pose)
        h.addWidget(self.cmbFrame)
        self._refresh_frame_choices()
        h.addStretch()
        return grp

    def _build_current_pose(self) -> QGroupBox:
        grp = QGroupBox("Current EE Pose (link7 in selected frame)")
        v = QVBoxLayout(grp)
        row = QHBoxLayout()
        self.lblPos = QLabel("x ---   y ---   z ---")
        self.lblPos.setStyleSheet("font-family:monospace; font-size:12px;")
        row.addWidget(self.lblPos)
        row.addStretch()
        btn = QPushButton("Refresh from TF")
        btn.clicked.connect(self._refresh_pose)
        row.addWidget(btn)
        v.addLayout(row)
        self.lblRot = QLabel("R ---°  P ---°  Y ---°")
        self.lblRot.setStyleSheet("font-family:monospace; font-size:12px;")
        v.addWidget(self.lblRot)
        return grp

    def _build_jog(self) -> QGroupBox:
        grp = QGroupBox("Jog (current frame, discrete step → plan & execute)")
        v = QVBoxLayout(grp)

        srow = QHBoxLayout()
        srow.addWidget(QLabel("Linear step:"))
        self.cmbLinStep = QComboBox()
        for mm in LIN_STEPS_MM:
            self.cmbLinStep.addItem(f"{mm} mm", userData=mm / 1000.0)
        self.cmbLinStep.setCurrentIndex(2)   # 10mm default
        srow.addWidget(self.cmbLinStep)
        srow.addSpacing(20)
        srow.addWidget(QLabel("Angular step:"))
        self.cmbAngStep = QComboBox()
        for d in ANG_STEPS_DEG:
            self.cmbAngStep.addItem(f"{d} °", userData=math.radians(d))
        self.cmbAngStep.setCurrentIndex(1)   # 5° default
        srow.addWidget(self.cmbAngStep)
        srow.addStretch()
        v.addLayout(srow)

        grid = QGridLayout()
        grid.addWidget(QLabel("<b>Translation</b>"), 0, 0, 1, 2)
        self._add_jog_button(grid, "-X", ("trans", "x", -1), 1, 0)
        self._add_jog_button(grid, "+X", ("trans", "x", +1), 1, 1)
        self._add_jog_button(grid, "-Y", ("trans", "y", -1), 2, 0)
        self._add_jog_button(grid, "+Y", ("trans", "y", +1), 2, 1)
        self._add_jog_button(grid, "-Z", ("trans", "z", -1), 3, 0)
        self._add_jog_button(grid, "+Z", ("trans", "z", +1), 3, 1)

        grid.addWidget(QLabel("<b>Rotation</b>"), 0, 3, 1, 2)
        self._add_jog_button(grid, "-RX", ("rot", "x", -1), 1, 3)
        self._add_jog_button(grid, "+RX", ("rot", "x", +1), 1, 4)
        self._add_jog_button(grid, "-RY", ("rot", "y", -1), 2, 3)
        self._add_jog_button(grid, "+RY", ("rot", "y", +1), 2, 4)
        self._add_jog_button(grid, "-RZ", ("rot", "z", -1), 3, 3)
        self._add_jog_button(grid, "+RZ", ("rot", "z", +1), 3, 4)
        grid.setColumnStretch(2, 1)
        grid.setColumnStretch(5, 1)
        v.addLayout(grid)
        return grp

    def _add_jog_button(self, grid: QGridLayout, label: str, axis,
                        row: int, col: int) -> None:
        b = QPushButton(label)
        b.setMinimumWidth(60)
        b.setMinimumHeight(32)
        b.clicked.connect(lambda _=False, a=axis: self._on_jog(a))
        grid.addWidget(b, row, col)

    def _build_target(self) -> QGroupBox:
        grp = QGroupBox("Manual target")
        v = QVBoxLayout(grp)
        mrow = QHBoxLayout()
        mrow.addWidget(QLabel("Mode:"))
        self.rdoDelta = QRadioButton("Delta from current")
        self.rdoAbs = QRadioButton("Absolute (in frame)")
        self.rdoDelta.setChecked(True)
        grp_mode = QButtonGroup(self)
        grp_mode.addButton(self.rdoDelta)
        grp_mode.addButton(self.rdoAbs)
        mrow.addWidget(self.rdoDelta)
        mrow.addWidget(self.rdoAbs)
        btn_copy = QPushButton("Copy current → Absolute")
        btn_copy.clicked.connect(self._on_copy_to_abs)
        mrow.addWidget(btn_copy)
        mrow.addStretch()
        v.addLayout(mrow)

        grid = QGridLayout()
        self.spnX = self._mkpos()
        self.spnY = self._mkpos()
        self.spnZ = self._mkpos()
        self.spnRX = self._mkrot()
        self.spnRY = self._mkrot()
        self.spnRZ = self._mkrot()
        for col, (lbl, w) in enumerate((("Δx", self.spnX), ("Δy", self.spnY),
                                        ("Δz", self.spnZ))):
            grid.addWidget(QLabel(lbl), 0, col * 2)
            grid.addWidget(w, 0, col * 2 + 1)
        for col, (lbl, w) in enumerate((("ΔR", self.spnRX), ("ΔP", self.spnRY),
                                        ("ΔY", self.spnRZ))):
            grid.addWidget(QLabel(lbl), 1, col * 2)
            grid.addWidget(w, 1, col * 2 + 1)
        v.addLayout(grid)
        return grp

    def _mkpos(self) -> QDoubleSpinBox:
        s = QDoubleSpinBox()
        s.setRange(-2.0, 2.0)
        s.setSingleStep(0.001)
        s.setDecimals(4)
        s.setSuffix(" m")
        s.setValue(0.0)
        return s

    def _mkrot(self) -> QDoubleSpinBox:
        s = QDoubleSpinBox()
        s.setRange(-180.0, 180.0)
        s.setSingleStep(0.5)
        s.setDecimals(2)
        s.setSuffix(" °")
        s.setValue(0.0)
        return s

    def _build_actions(self) -> QGroupBox:
        grp = QGroupBox("Plan / Execute")
        v = QVBoxLayout(grp)
        srow = QHBoxLayout()
        srow.addWidget(QLabel("vel:"))
        self.spnVel = QDoubleSpinBox()
        self.spnVel.setRange(0.01, 1.0)
        self.spnVel.setSingleStep(0.05)
        self.spnVel.setDecimals(2)
        self.spnVel.setValue(0.10)
        srow.addWidget(self.spnVel)
        srow.addWidget(QLabel("acc:"))
        self.spnAcc = QDoubleSpinBox()
        self.spnAcc.setRange(0.01, 1.0)
        self.spnAcc.setSingleStep(0.05)
        self.spnAcc.setDecimals(2)
        self.spnAcc.setValue(0.10)
        srow.addWidget(self.spnAcc)
        srow.addWidget(QLabel("plan time:"))
        self.spnPlanTime = QDoubleSpinBox()
        self.spnPlanTime.setRange(0.5, 60.0)
        self.spnPlanTime.setValue(5.0)
        self.spnPlanTime.setSuffix(" s")
        srow.addWidget(self.spnPlanTime)
        srow.addStretch()
        v.addLayout(srow)

        brow = QHBoxLayout()
        self.btnPlan = QPushButton("Plan (preview)")
        self.btnExec = QPushButton("Plan && Execute")
        self.btnExec.setStyleSheet(
            "QPushButton { background-color:#4CAF50; color:white; font-weight:bold; }")
        self.btnStop = QPushButton("Stop")
        self.btnStop.setStyleSheet(
            "QPushButton { background-color:#c00; color:white; font-weight:bold; }")
        self.btnCapture = QPushButton("Capture → MoveL Step")
        self.btnPlan.clicked.connect(lambda: self._submit_target(execute=False))
        self.btnExec.clicked.connect(lambda: self._submit_target(execute=True))
        self.btnStop.clicked.connect(self._on_stop)
        self.btnCapture.clicked.connect(self._on_capture)
        for w in (self.btnPlan, self.btnExec, self.btnStop, self.btnCapture):
            brow.addWidget(w)
        v.addLayout(brow)
        return grp

    # ------------------------------------------------------------------
    # Frame / arm management
    # ------------------------------------------------------------------

    def _on_arm_changed(self, _idx: int) -> None:
        self._refresh_frame_choices()
        self._refresh_pose()

    def _refresh_frame_choices(self) -> None:
        arm = self.cmbArm.currentData() or "right"
        self.cmbFrame.blockSignals(True)
        self.cmbFrame.clear()
        self.cmbFrame.addItem(f"openarmx_{arm}_link0 (base)",
                              userData=f"openarmx_{arm}_link0")
        self.cmbFrame.addItem(f"openarmx_{arm}_link7 (tool)",
                              userData=f"openarmx_{arm}_link7")
        self.cmbFrame.addItem("world", userData="world")
        self.cmbFrame.blockSignals(False)

    def _current_arm(self) -> str:
        return self.cmbArm.currentData() or "right"

    def _current_frame(self) -> str:
        return self.cmbFrame.currentData() or f"openarmx_{self._current_arm()}_link0"

    # ------------------------------------------------------------------
    # Current pose
    # ------------------------------------------------------------------

    def _refresh_pose(self) -> None:
        pose = self._bridge.get_ee_pose(self._current_arm(), self._current_frame())
        if pose is None:
            self.lblPos.setText("x ---   y ---   z ---   (no TF)")
            self.lblRot.setText("R ---°   P ---°   Y ---°")
            self._current_pose = None
            return
        self._current_pose = pose
        self.lblPos.setText(
            f"x {pose['x']:+.4f} m   y {pose['y']:+.4f} m   z {pose['z']:+.4f} m")
        self.lblRot.setText(
            f"R {math.degrees(pose['roll']):+7.2f}°  "
            f"P {math.degrees(pose['pitch']):+7.2f}°  "
            f"Y {math.degrees(pose['yaw']):+7.2f}°")

    def _on_copy_to_abs(self) -> None:
        if not self._current_pose:
            QMessageBox.warning(self, "Copy", "Current pose unavailable. Refresh first.")
            return
        p = self._current_pose
        self.rdoAbs.setChecked(True)
        self.spnX.setValue(p["x"])
        self.spnY.setValue(p["y"])
        self.spnZ.setValue(p["z"])
        self.spnRX.setValue(math.degrees(p["roll"]))
        self.spnRY.setValue(math.degrees(p["pitch"]))
        self.spnRZ.setValue(math.degrees(p["yaw"]))

    # ------------------------------------------------------------------
    # Jog (single click → delta in current frame → plan&execute)
    # ------------------------------------------------------------------

    def _on_jog(self, axis_spec) -> None:
        if self._busy:
            self._set_status("Busy — wait for current motion to finish.")
            return
        if not self._current_pose:
            self._set_status("No current pose — Refresh from TF first.")
            return
        kind, axis, sign = axis_spec
        delta = {"x": 0.0, "y": 0.0, "z": 0.0,
                 "rx": 0.0, "ry": 0.0, "rz": 0.0}
        if kind == "trans":
            step_m = self.cmbLinStep.currentData()
            delta[axis] = sign * float(step_m)
        else:
            step_r = self.cmbAngStep.currentData()
            delta[f"r{axis}"] = sign * float(step_r)
        target = self._apply_delta(self._current_pose, delta)
        self._send_target(target, execute=True, source=f"jog {kind}.{axis}{sign:+d}")

    @staticmethod
    def _apply_delta(current: dict, delta: dict) -> dict:
        return {
            "x": current["x"] + delta["x"],
            "y": current["y"] + delta["y"],
            "z": current["z"] + delta["z"],
            "roll":  current["roll"]  + delta["rx"],
            "pitch": current["pitch"] + delta["ry"],
            "yaw":   current["yaw"]   + delta["rz"],
        }

    # ------------------------------------------------------------------
    # Manual target submit
    # ------------------------------------------------------------------

    def _submit_target(self, execute: bool = True) -> None:
        if self._busy:
            self._set_status("Busy — wait for current motion to finish.")
            return
        if self.rdoAbs.isChecked():
            target = {
                "x": self.spnX.value(),
                "y": self.spnY.value(),
                "z": self.spnZ.value(),
                "roll":  math.radians(self.spnRX.value()),
                "pitch": math.radians(self.spnRY.value()),
                "yaw":   math.radians(self.spnRZ.value()),
            }
        else:
            if not self._current_pose:
                self._set_status("Delta mode requires current pose — Refresh first.")
                return
            delta = {
                "x":  self.spnX.value(),
                "y":  self.spnY.value(),
                "z":  self.spnZ.value(),
                "rx": math.radians(self.spnRX.value()),
                "ry": math.radians(self.spnRY.value()),
                "rz": math.radians(self.spnRZ.value()),
            }
            target = self._apply_delta(self._current_pose, delta)
        self._send_target(target, execute=execute, source="manual")

    def _send_target(self, target: dict, execute: bool, source: str) -> None:
        arm = self._current_arm()
        planner = self.cmbPlanner.currentData() or "PILZ_LIN"
        frame = self._current_frame()
        vel = self.spnVel.value()
        acc = self.spnAcc.value()
        plan_time = self.spnPlanTime.value()
        ok = self._bridge.plan_and_execute_cartesian(
            arm=arm, planner_id=planner, target_pose=target,
            frame_id=frame, vel_scale=vel, acc_scale=acc,
            plan_time=plan_time, execute=execute,
        )
        if not ok:
            self._set_status("Submit failed (action server unavailable?)")
            return
        self._busy = True
        self._set_buttons_enabled(False)
        verb = "Execute" if execute else "Plan"
        self._set_status(
            f"{verb} sent: arm={arm} planner={planner} frame={frame} "
            f"vel={vel:.2f} acc={acc:.2f} [{source}]")

    def _on_stop(self) -> None:
        self._bridge.cancel_motion()
        self._set_status("Cancel requested.")

    def _on_motion_result(self, r: dict) -> None:
        self._busy = False
        self._set_buttons_enabled(True)
        self._set_status(
            f"[{r.get('phase','?')}]  success={r.get('success')}  "
            f"err={r.get('error_code','?')}  {r.get('message','')}")

    def _set_buttons_enabled(self, enabled: bool) -> None:
        for w in (self.btnPlan, self.btnExec, self.btnCapture):
            w.setEnabled(enabled)

    # ------------------------------------------------------------------
    # Capture → step JSON (scenarios/poses/<name>.json)
    # ------------------------------------------------------------------

    def _on_capture(self) -> None:
        if not self._current_pose:
            QMessageBox.warning(self, "Capture", "Current pose unavailable.")
            return
        name, ok = QInputDialog.getText(self, "Capture as MoveL Step", "Step name:")
        if not ok or not name.strip():
            return
        name = name.strip().replace(" ", "_")
        p = self._current_pose
        planner = self.cmbPlanner.currentData() or "PILZ_LIN"
        step = {
            "name": name,
            "type": "movel" if planner.startswith("PILZ_LIN") else "movej",
            "arm": self._current_arm(),
            "frame_id": self._current_frame(),
            "target_pose": {
                "position":         [p["x"], p["y"], p["z"]],
                "orientation_xyzw": [p["qx"], p["qy"], p["qz"], p["qw"]],
            },
            "planner": planner.lower(),
            "max_vel_scale": self.spnVel.value(),
            "max_acc_scale": self.spnAcc.value(),
            "duration_sec": 3.0,
            "captured": datetime.now().isoformat(timespec="seconds"),
        }
        d = _poses_dir()
        os.makedirs(d, exist_ok=True)
        path = os.path.join(d, f"{name}.json")
        with open(path, "w", encoding="utf-8") as f:
            json.dump(step, f, indent=2, ensure_ascii=False)
        self._set_status(f"Captured → {path}")

    # ------------------------------------------------------------------

    def _set_status(self, text: str) -> None:
        self._lbl_status.setText(text)

    def shutdown(self) -> None:
        self._pose_timer.stop()
