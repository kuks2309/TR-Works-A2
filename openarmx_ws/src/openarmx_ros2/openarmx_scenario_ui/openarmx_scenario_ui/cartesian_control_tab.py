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

import numpy as np
from ament_index_python.packages import get_package_share_directory
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QButtonGroup, QCheckBox, QComboBox, QDoubleSpinBox, QGridLayout, QGroupBox,
    QHBoxLayout, QInputDialog, QLabel, QMessageBox, QPushButton, QRadioButton,
    QTabWidget, QVBoxLayout, QWidget,
)

from openarmx_scenario_ui import joint_data as jd
from openarmx_scenario_ui.ik_check import LinearReachabilityChecker


LIN_STEPS_MM = [1, 5, 10, 20, 50, 100]
ANG_STEPS_DEG = [1, 5, 10, 30]
# Jog is a velocity command (Cartesian linear/angular speed). The UI exposes
# step + velocity; duration sent to cyclo is step/velocity so the controller
# knows how fast to interpolate.
LIN_VELS_MM_S = [10, 25, 50, 100]
ANG_VELS_DEG_S = [5, 10, 30, 60]
# MoveIt vel_scale (0..1) base: cmbLinVel / base = vel_scale.
# 100 mm/s (top of LIN_VELS_MM_S) maps to vel_scale 1.0 = robot max.
MOVEIT_JOG_LIN_BASE_MPS = 0.1
MOVEIT_JOG_ANG_BASE_RPS = math.radians(60)

# "Drag release" detection: no new marker pose for this long → assume user
# released the mouse. publish_while_dragging=true emits every motion update.
DRAG_RELEASE_DEBOUNCE_MS = 250

ARMS = ["right", "left"]
PLANNERS = [
    ("PILZ_LIN", "Pilz LIN  (직선)"),
    ("PILZ_PTP", "Pilz PTP  (관절)"),
    ("PILZ_CIRC", "Pilz CIRC (원호)"),
    ("OMPL", "OMPL (default)"),
]


class CartesianControlTab(QWidget):
    def __init__(self, bridge, parent=None) -> None:
        super().__init__(parent)
        self._bridge = bridge
        self._current_pose = None
        self._busy = False

        # per-arm Jog tab joint angle labels (filled in by _build_arm_jog_group)
        self._jog_joint_labels = {"left": {}, "right": {}}

        # IK pre-check (per-arm Pinocchio numerical solver), lazy-loaded.
        # Cache the latest joint state in radians so we can warm-start IK at
        # the robot's actual configuration.
        self._ik_checkers: dict = {"left": None, "right": None}
        self._latest_arm_q_rad: dict = {"left": {}, "right": {}}
        self._ik_n_steps = 10

        # press-hold streaming jog state (cyclo backend: continuous publish
        # while a jog button is held; release stops).
        JOG_STREAM_RATE_HZ = 20
        self._jog_active = {"left": None, "right": None}
        self._jog_timers = {}
        for arm in ("left", "right"):
            t = QTimer(self)
            t.setInterval(int(1000 / JOG_STREAM_RATE_HZ))
            t.timeout.connect(lambda a=arm: self._on_jog_tick(a))
            self._jog_timers[arm] = t
        self._jog_stream_rate_hz = JOG_STREAM_RATE_HZ

        self._build_ui()
        self._bridge.sig_motion_result.connect(self._on_motion_result)
        self._bridge.sig_marker_pose.connect(self._on_marker_pose)
        self._bridge.sig_joint_state.connect(self._on_jog_joint_state)

        # debounce timer — fires on "drag release" (no new pose for N ms)
        self._drag_debounce = QTimer(self)
        self._drag_debounce.setSingleShot(True)
        self._drag_debounce.timeout.connect(self._on_drag_release)

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

        # Common header (applies to all sub-tabs).
        root.addWidget(self._build_header())
        # Current EE Pose group hidden — joint angles per arm already shown in
        # Jog sub-tab. Build it anyway so _refresh_pose (timer + signals) still
        # has lblPos/lblRot to update without raising AttributeError.
        self._current_pose_grp = self._build_current_pose()
        self._current_pose_grp.hide()

        # Three input methods as sub-tabs.
        self._subtabs = QTabWidget()
        self._subtabs.addTab(self._wrap_subtab(self._build_ee_leader()), "Marker")
        self._subtabs.addTab(self._wrap_subtab(self._build_jog()), "Jog")
        self._subtabs.addTab(self._build_manual_subtab(), "Manual")
        root.addWidget(self._subtabs, 1)

        # Common status footer.
        self._lbl_status = QLabel("Ready")
        self._lbl_status.setStyleSheet(
            "color:#444; padding:4px; font-family:monospace;")
        root.addWidget(self._lbl_status)

    @staticmethod
    def _wrap_subtab(group_box: QGroupBox) -> QWidget:
        """Embed a single QGroupBox as a sub-tab page (with bottom stretch)."""
        w = QWidget()
        v = QVBoxLayout(w)
        v.addWidget(group_box)
        v.addStretch()
        return w

    def _build_manual_subtab(self) -> QWidget:
        """Manual numeric pose target + Plan/Execute footer (vel/acc/time)."""
        w = QWidget()
        v = QVBoxLayout(w)
        v.addWidget(self._build_target())
        v.addWidget(self._build_actions())
        v.addStretch()
        return w

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

    def _build_ee_leader(self) -> QGroupBox:
        grp = QGroupBox("EE Leader Marker (RViz)")
        v = QVBoxLayout(grp)
        trow = QHBoxLayout()
        self.chkUseMarker = QCheckBox(
            f"Use EE Leader Marker  "
            f"(RViz 6-DoF drag → arm auto-follows on release, ≤{DRAG_RELEASE_DEBOUNCE_MS} ms debounce)")
        self.chkUseMarker.toggled.connect(self._on_marker_use_toggled)
        trow.addWidget(self.chkUseMarker)
        trow.addStretch()
        v.addLayout(trow)

        # Per-arm marker pose display (both arms shown side-by-side, consistent
        # with the Jog sub-tab's per-arm Joint angles layout).
        self._lbl_marker_pos = {"left": None, "right": None}
        self._lbl_marker_rot = {"left": None, "right": None}
        for arm in ("left", "right"):
            grp_arm = QGroupBox(f"{arm.capitalize()} Arm")
            varm = QVBoxLayout(grp_arm)
            self._lbl_marker_pos[arm] = QLabel("x ---   y ---   z ---")
            self._lbl_marker_pos[arm].setStyleSheet(
                "font-family:monospace; font-size:12px; color:#555;")
            varm.addWidget(self._lbl_marker_pos[arm])
            self._lbl_marker_rot[arm] = QLabel("R ---°  P ---°  Y ---°  (no data)")
            self._lbl_marker_rot[arm].setStyleSheet(
                "font-family:monospace; font-size:12px; color:#555;")
            varm.addWidget(self._lbl_marker_rot[arm])
            v.addWidget(grp_arm)
        # Back-compat aliases for any external code still referring to single
        # labels: point them at the right-arm row.
        self.lblMarkerPos = self._lbl_marker_pos["right"]
        self.lblMarkerRot = self._lbl_marker_rot["right"]

        brow = QHBoxLayout()
        self.btnCaptureMarker = QPushButton("Capture marker → MoveL Step")
        self.btnCaptureMarker.clicked.connect(self._on_capture_marker)
        self.btnCaptureMarker.setEnabled(False)
        brow.addWidget(self.btnCaptureMarker)
        self.btnMarkerToTarget = QPushButton("Marker → Absolute Target")
        self.btnMarkerToTarget.clicked.connect(self._on_marker_to_target)
        self.btnMarkerToTarget.setEnabled(False)
        brow.addWidget(self.btnMarkerToTarget)
        brow.addStretch()
        v.addLayout(brow)
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
        grp = QGroupBox("Jog (both arms — discrete step → plan & execute)")
        v = QVBoxLayout(grp)

        # ---- Shared settings (apply to BOTH arms) ----
        brow = QHBoxLayout()
        brow.addWidget(QLabel("Backend:"))
        self.cmbJogBackend = QComboBox()
        self.cmbJogBackend.addItem("MoveIt (MoveGroup plan&execute)",
                                   userData="moveit")
        self.cmbJogBackend.addItem("cyclo MoveL (QP+CBF, direct)",
                                   userData="cyclo_movel")
        brow.addWidget(self.cmbJogBackend)
        brow.addStretch()
        v.addLayout(brow)

        srow = QHBoxLayout()
        srow.addWidget(QLabel("Linear step:"))
        self.cmbLinStep = QComboBox()
        for mm in LIN_STEPS_MM:
            self.cmbLinStep.addItem(f"{mm} mm", userData=mm / 1000.0)
        self.cmbLinStep.setCurrentIndex(2)
        srow.addWidget(self.cmbLinStep)
        srow.addSpacing(20)
        srow.addWidget(QLabel("Angular step:"))
        self.cmbAngStep = QComboBox()
        for d in ANG_STEPS_DEG:
            self.cmbAngStep.addItem(f"{d} °", userData=math.radians(d))
        self.cmbAngStep.setCurrentIndex(1)
        srow.addWidget(self.cmbAngStep)
        srow.addStretch()
        v.addLayout(srow)

        # Velocity row — jog is a velocity command; duration = step / velocity.
        vrow = QHBoxLayout()
        vrow.addWidget(QLabel("Linear velocity:"))
        self.cmbLinVel = QComboBox()
        for v_mm in LIN_VELS_MM_S:
            self.cmbLinVel.addItem(f"{v_mm} mm/s", userData=v_mm / 1000.0)
        self.cmbLinVel.setCurrentIndex(1)  # default 25 mm/s
        vrow.addWidget(self.cmbLinVel)
        vrow.addSpacing(20)
        vrow.addWidget(QLabel("Angular velocity:"))
        self.cmbAngVel = QComboBox()
        for d in ANG_VELS_DEG_S:
            self.cmbAngVel.addItem(f"{d} °/s", userData=math.radians(d))
        self.cmbAngVel.setCurrentIndex(1)
        vrow.addWidget(self.cmbAngVel)
        vrow.addStretch()
        v.addLayout(vrow)

        # ---- Per-arm jog (Left | Right) ----
        arms_row = QHBoxLayout()
        arms_row.addWidget(self._build_arm_jog_group("Left Arm",  "left"))
        arms_row.addWidget(self._build_arm_jog_group("Right Arm", "right"))
        v.addLayout(arms_row)
        return grp

    def _build_arm_jog_group(self, title: str, arm: str) -> QGroupBox:
        grp = QGroupBox(title)
        v = QVBoxLayout(grp)

        # Per-arm frame dropdown (independent of the shared header Arm dropdown
        # — Jog operates on this arm regardless of which arm is "selected" up
        # top, since both arms get their own button columns here).
        frow = QHBoxLayout()
        frow.addWidget(QLabel("Frame:"))
        cmb_frame = QComboBox()
        cmb_frame.addItem("openarmx_body_link0 (bimanual base)",
                          userData="openarmx_body_link0")
        cmb_frame.addItem(f"openarmx_{arm}_link0 (arm base)",
                          userData=f"openarmx_{arm}_link0")
        cmb_frame.addItem(f"openarmx_{arm}_link7 (link7)",
                          userData=f"openarmx_{arm}_link7")
        cmb_frame.addItem(f"openarmx_{arm}_hand_tcp (TCP)",
                          userData=f"openarmx_{arm}_hand_tcp")
        cmb_frame.addItem("world", userData="world")
        frow.addWidget(cmb_frame)
        frow.addStretch()
        v.addLayout(frow)
        if arm == "left":
            self.cmbLeftJogFrame = cmb_frame
        else:
            self.cmbRightJogFrame = cmb_frame

        grid = QGridLayout()
        grid.addWidget(QLabel("<b>Translation</b>"), 0, 0, 1, 2)
        self._add_jog_button(grid, "-X", (arm, "trans", "x", -1), 1, 0)
        self._add_jog_button(grid, "+X", (arm, "trans", "x", +1), 1, 1)
        self._add_jog_button(grid, "-Y", (arm, "trans", "y", -1), 2, 0)
        self._add_jog_button(grid, "+Y", (arm, "trans", "y", +1), 2, 1)
        self._add_jog_button(grid, "-Z", (arm, "trans", "z", -1), 3, 0)
        self._add_jog_button(grid, "+Z", (arm, "trans", "z", +1), 3, 1)
        grid.addWidget(QLabel("<b>Rotation</b>"), 0, 3, 1, 2)
        self._add_jog_button(grid, "-RX", (arm, "rot", "x", -1), 1, 3)
        self._add_jog_button(grid, "+RX", (arm, "rot", "x", +1), 1, 4)
        self._add_jog_button(grid, "-RY", (arm, "rot", "y", -1), 2, 3)
        self._add_jog_button(grid, "+RY", (arm, "rot", "y", +1), 2, 4)
        self._add_jog_button(grid, "-RZ", (arm, "rot", "z", -1), 3, 3)
        self._add_jog_button(grid, "+RZ", (arm, "rot", "z", +1), 3, 4)
        grid.setColumnStretch(2, 1)
        grid.setColumnStretch(5, 1)
        v.addLayout(grid)

        # Live joint angles for this arm (read-only). Updated by
        # sig_joint_state → _on_jog_joint_state. arms in deg.
        joints_grid = QGridLayout()
        joints_grid.addWidget(QLabel("<b>Joint angles (deg)</b>"),
                              0, 0, 1, 4)
        for i in range(1, 8):
            jn = f"openarmx_{arm}_joint{i}"
            r, c = divmod(i - 1, 4)
            lbl_name = QLabel(f"j{i}")
            lbl_val = QLabel("---")
            lbl_val.setStyleSheet(
                "font-family:monospace; min-width:55px; "
                "padding:2px 6px; background:#f0f0f0;")
            cell = QHBoxLayout()
            cell.addWidget(lbl_name)
            cell.addWidget(lbl_val)
            wrap = QWidget()
            wrap.setLayout(cell)
            joints_grid.addWidget(wrap, r + 1, c)
            self._jog_joint_labels[arm][jn] = lbl_val
        v.addLayout(joints_grid)
        return grp

    def _on_jog_joint_state(self, positions: dict) -> None:
        for arm in ("left", "right"):
            for jn, lbl in self._jog_joint_labels[arm].items():
                if jn in positions:
                    lbl.setText(f"{positions[jn]:+7.2f}°")
                    # Cache in radians for IK pre-check warm-start.
                    self._latest_arm_q_rad[arm][jn] = math.radians(positions[jn])

    def _get_ik_checker(self, arm: str):
        """Lazy-load the per-arm LinearReachabilityChecker (Pinocchio)."""
        if self._ik_checkers.get(arm) is not None:
            return self._ik_checkers[arm]
        try:
            share = get_package_share_directory("openarmx_pick")
            urdf = os.path.join(share, "urdf", f"openarmx_{arm}_solver.urdf")
            tcp = f"openarmx_{arm}_hand_tcp"
            self._ik_checkers[arm] = LinearReachabilityChecker(urdf, tcp)
        except Exception as exc:
            self._set_status(f"IK checker init failed ({arm}): {exc}")
            self._ik_checkers[arm] = None
        return self._ik_checkers[arm]

    def _get_q_seed_rad(self, arm: str):
        """Return 7-vector of current arm joint angles (rad) or None."""
        cache = self._latest_arm_q_rad.get(arm, {})
        ordered = [f"openarmx_{arm}_joint{i}" for i in range(1, 8)]
        if not all(j in cache for j in ordered):
            return None
        return np.array([cache[j] for j in ordered])

    def _add_jog_button(self, grid: QGridLayout, label: str, axis,
                        row: int, col: int) -> None:
        b = QPushButton(label)
        b.setMinimumWidth(50)
        b.setMinimumHeight(28)
        # pressed/released → streaming for cyclo backend; click → single-shot
        # for MoveIt (plan&execute is discrete). _on_jog_press picks the path.
        b.pressed.connect(lambda a=axis: self._on_jog_press(a))
        b.released.connect(lambda a=axis: self._on_jog_release(a))
        grid.addWidget(b, row, col)

    def _on_jog_press(self, axis_spec) -> None:
        # debug: always show press in status so signal/slot wiring is visible
        self._set_status(f"PRESS {axis_spec}  backend={self.cmbJogBackend.currentData()}")
        backend = self.cmbJogBackend.currentData() or "moveit"
        if backend == "cyclo_movel":
            # Single-shot: publish ONE target = cur + full step in chosen frame,
            # then let cyclo's streaming converge. Re-press to step again.
            # (Streaming-rate publish caused cyclo to reset every tick → no
            #  meaningful motion. Single-shot lets the controller's internal
            #  loop do the actual interpolation.)
            arm, kind, axis, sign = axis_spec
            user_frame = (self.cmbLeftJogFrame if arm == "left"
                          else self.cmbRightJogFrame).currentData() or f"openarmx_{arm}_link0"
            controlled_link = f"openarmx_{arm}_hand_tcp"
            cur = self._bridge.get_ee_pose(arm, user_frame, controlled_link)
            if cur is None:
                self._set_status(f"cyclo jog: TF lookup failed ({user_frame})")
                return
            delta = {"x": 0.0, "y": 0.0, "z": 0.0,
                     "rx": 0.0, "ry": 0.0, "rz": 0.0}
            if kind == "trans":
                delta[axis] = sign * float(self.cmbLinStep.currentData())
            else:
                delta[f"r{axis}"] = sign * float(self.cmbAngStep.currentData())
            target = self._apply_delta(cur, delta)
            cyclo_base = jd.CYCLO_BASE_FRAME
            target_base = self._bridge.transform_pose(target, user_frame, cyclo_base)
            if target_base is None:
                self._set_status(
                    f"cyclo jog: TF transform {user_frame}→{cyclo_base} failed")
                return

            # Linear-path IK pre-check: cyclo's QP silently falls back via slack
            # on unreachable directions (no controller_error), so verify the
            # straight-line path is IK-feasible at every waypoint before publish.
            checker = self._get_ik_checker(arm)
            q_seed = self._get_q_seed_rad(arm)
            if checker is not None and q_seed is not None:
                start_base = self._bridge.get_ee_pose(
                    arm, cyclo_base, controlled_link)
                if start_base is not None:
                    res = checker.check_linear_path(
                        start_base, target_base, q_seed,
                        n_steps=self._ik_n_steps)
                    if not res.reachable:
                        self._set_status(
                            f"cyclo {arm} {kind}.{axis}{sign:+d}  "
                            f"UNREACHABLE: {res.reason} "
                            f"@ waypoint {res.waypoint_index}/{res.n_waypoints}")
                        return

            # Jog is a velocity command — duration = step / velocity gives cyclo
            # the per-step time horizon to interpolate along.
            if kind == "trans":
                distance = abs(float(self.cmbLinStep.currentData()))
                velocity = float(self.cmbLinVel.currentData())
            else:
                distance = abs(float(self.cmbAngStep.currentData()))
                velocity = float(self.cmbAngVel.currentData())
            duration_sec = max(0.05, distance / max(velocity, 1e-6))
            ok = self._bridge.publish_cyclo_movel(
                arm, target_base, cyclo_base, duration_sec=duration_sec)
            if not ok:
                self._set_status(f"cyclo jog: publish failed (arm={arm})")
                return
            self._set_status(
                f"cyclo {arm} {kind}.{axis}{sign:+d}  v={velocity*1000:.0f}{'mm' if kind=='trans' else 'rad'}/s  dur={duration_sec:.2f}s  "
                f"cur({user_frame})=({cur['x']:+.3f},{cur['y']:+.3f},{cur['z']:+.3f})  "
                f"→ tgt(body_link0)=({target_base['x']:+.3f},{target_base['y']:+.3f},{target_base['z']:+.3f})")
        else:
            # MoveIt path stays single-shot on click (plan&execute is discrete)
            self._on_jog(axis_spec)

    def _on_jog_release(self, axis_spec) -> None:
        # single-shot mode: nothing to stop. (Timer-based streaming was
        # removed because cyclo resets every tick and never converges.)
        arm = axis_spec[0]
        self._jog_timers[arm].stop()
        self._jog_active[arm] = None
        self._jog_status_shown = False

    def _on_jog_tick(self, arm: str) -> None:
        spec = self._jog_active.get(arm)
        if spec is None:
            return
        _arm, kind, axis, sign = spec
        backend = self.cmbJogBackend.currentData() or "moveit"
        # User-selected jog frame (axis basis for the delta). For cyclo we
        # still respect the user's frame choice — we apply the delta in that
        # frame, then transform the resulting target to cyclo's base_frame
        # (openarmx_body_link0) before publishing (cyclo ignores frame_id).
        user_frame = (self.cmbLeftJogFrame if arm == "left"
                      else self.cmbRightJogFrame).currentData() or f"openarmx_{arm}_link0"
        if backend == "cyclo_movel":
            controlled_link = f"openarmx_{arm}_hand_tcp"
        else:
            controlled_link = ""   # default link7
        cur = self._bridge.get_ee_pose(arm, user_frame, controlled_link)
        if cur is None:
            self._set_status(f"jog tick: TF lookup failed for {arm} in {user_frame}")
            self._jog_timers[arm].stop()   # stop the bleed; no point streaming blind
            return
        # step dropdown value is interpreted as "distance per second" while
        # streaming; per-tick increment = step / rate.
        rate = float(self._jog_stream_rate_hz)
        delta = {"x": 0.0, "y": 0.0, "z": 0.0,
                 "rx": 0.0, "ry": 0.0, "rz": 0.0}
        if kind == "trans":
            per_sec_m = float(self.cmbLinStep.currentData())
            delta[axis] = sign * (per_sec_m / rate)
        else:
            per_sec_r = float(self.cmbAngStep.currentData())
            delta[f"r{axis}"] = sign * (per_sec_r / rate)
        target = self._apply_delta(cur, delta)
        # cyclo requires the pose expressed in its base_frame; transform
        # the delta-applied target from the user-selected frame.
        cyclo_base = "openarmx_body_link0"
        target_base = self._bridge.transform_pose(target, user_frame, cyclo_base)
        if target_base is None:
            self._set_status(
                f"jog tick: TF transform {user_frame}→{cyclo_base} failed")
            self._jog_timers[arm].stop()
            return
        ok = self._bridge.publish_cyclo_movel(
            arm, target_base, cyclo_base, duration_sec=2.0 / rate)
        if not ok:
            self._set_status(f"jog tick: publish_cyclo_movel failed (arm={arm})")
            self._jog_timers[arm].stop()
            return
        # one-shot status on first publish (avoid spamming at 20Hz)
        if not getattr(self, "_jog_status_shown", False):
            self._set_status(
                f"cyclo streaming → {arm} {kind}.{axis}{sign:+d}  [{user_frame}]")
            self._jog_status_shown = True

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
        self._refresh_marker_display()

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

    # ------------------------------------------------------------------
    # EE Leader Marker — toggle + pose display + capture/copy actions
    # ------------------------------------------------------------------

    def _on_marker_use_toggled(self, checked: bool) -> None:
        self.btnCaptureMarker.setEnabled(checked)
        self.btnMarkerToTarget.setEnabled(checked)
        if not checked:
            self._drag_debounce.stop()
        self._refresh_marker_display()
        self._set_status(
            "EE Leader Marker: enabled (drag → release → auto plan&execute)"
            if checked else "EE Leader Marker: disabled")

    def _on_marker_pose(self, arm: str, _pose: dict) -> None:
        # Refresh both arms' displays on every marker update (so left+right
        # rows stay in sync). Auto-follow debounce only fires for the currently
        # selected arm so a passive left-arm refresh doesn't drive the right.
        self._refresh_marker_display()
        if self.chkUseMarker.isChecked() and arm == self._current_arm():
            self._drag_debounce.start(DRAG_RELEASE_DEBOUNCE_MS)

    def _on_drag_release(self) -> None:
        if self._busy:
            # motion already in flight from a previous release; skip this one
            self._set_status("Auto follow skipped: previous motion still running.")
            return
        arm = self._current_arm()
        pose = self._bridge.get_marker_pose(arm)
        if pose is None:
            return
        target = {
            "x": pose["x"], "y": pose["y"], "z": pose["z"],
            "roll":  pose["roll"],
            "pitch": pose["pitch"],
            "yaw":   pose["yaw"],
        }
        # Use the marker's own frame_id (e.g. openarmx_body_link0) — NOT the
        # UI's Cartesian Frame dropdown — so the pose is interpreted correctly.
        self._send_target(target, execute=True,
                          source=f"auto-follow @ {pose['frame_id']}",
                          frame_id_override=pose["frame_id"])

    def _refresh_marker_display(self) -> None:
        toggle_on = self.chkUseMarker.isChecked()
        for arm in ("left", "right"):
            lbl_pos = self._lbl_marker_pos[arm]
            lbl_rot = self._lbl_marker_rot[arm]
            if not toggle_on:
                lbl_pos.setText("x ---   y ---   z ---")
                lbl_rot.setText("R ---°  P ---°  Y ---°  (toggle off)")
                continue
            pose = self._bridge.get_marker_pose(arm)
            if pose is None:
                lbl_pos.setText("x ---   y ---   z ---")
                lbl_rot.setText("R ---°  P ---°  Y ---°  (no data — drag the marker in RViz)")
                continue
            lbl_pos.setText(
                f"x {pose['x']:+.4f} m   y {pose['y']:+.4f} m   "
                f"z {pose['z']:+.4f} m   [{pose['frame_id']}]")
            lbl_rot.setText(
                f"R {math.degrees(pose['roll']):+7.2f}°  "
                f"P {math.degrees(pose['pitch']):+7.2f}°  "
                f"Y {math.degrees(pose['yaw']):+7.2f}°")

    def _on_marker_to_target(self) -> None:
        pose = self._bridge.get_marker_pose(self._current_arm())
        if pose is None:
            QMessageBox.warning(self, "Marker → Target",
                                "No marker pose received yet. Drag the marker in RViz first.")
            return
        self.rdoAbs.setChecked(True)
        self.spnX.setValue(pose["x"])
        self.spnY.setValue(pose["y"])
        self.spnZ.setValue(pose["z"])
        self.spnRX.setValue(math.degrees(pose["roll"]))
        self.spnRY.setValue(math.degrees(pose["pitch"]))
        self.spnRZ.setValue(math.degrees(pose["yaw"]))
        self._set_status(
            f"Marker pose copied to Absolute target (frame: {pose['frame_id']}). "
            f"NOTE: change Frame dropdown to match if needed.")

    def _on_capture_marker(self) -> None:
        arm = self._current_arm()
        pose = self._bridge.get_marker_pose(arm)
        if pose is None:
            QMessageBox.warning(self, "Capture marker",
                                "No marker pose received yet. Drag the marker in RViz first.")
            return
        name, ok = QInputDialog.getText(
            self, "Capture marker as MoveL Step", "Step name:")
        if not ok or not name.strip():
            return
        name = name.strip().replace(" ", "_")
        planner = self.cmbPlanner.currentData() or "PILZ_LIN"
        step = {
            "name": name,
            "type": "movel" if planner.startswith("PILZ_LIN") else "movej",
            "arm": arm,
            "frame_id": pose["frame_id"],
            "source": "ee_leader_marker",
            "target_pose": {
                "position":         [pose["x"], pose["y"], pose["z"]],
                "orientation_xyzw": [pose["qx"], pose["qy"], pose["qz"], pose["qw"]],
            },
            "planner": planner.lower(),
            "max_vel_scale": self.spnVel.value(),
            "max_acc_scale": self.spnAcc.value(),
            "duration_sec": 3.0,
            "captured": datetime.now().isoformat(timespec="seconds"),
        }
        d = jd._poses_dir()
        os.makedirs(d, exist_ok=True)
        path = os.path.join(d, f"{name}.json")
        with open(path, "w", encoding="utf-8") as f:
            json.dump(step, f, indent=2, ensure_ascii=False)
        self._set_status(f"Marker captured → {path}")

    # ------------------------------------------------------------------

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
        # axis_spec = (arm, kind, axis, sign) — Jog tab has its own L/R columns
        # independent of the shared header Arm dropdown.
        if self._busy:
            self._set_status("Busy — wait for current motion to finish.")
            return
        arm, kind, axis, sign = axis_spec
        frame = (self.cmbLeftJogFrame if arm == "left"
                 else self.cmbRightJogFrame).currentData() or f"openarmx_{arm}_link0"

        cur = self._bridge.get_ee_pose(arm, frame)
        if cur is None:
            self._set_status(
                f"No TF for {arm} in {frame} — bringup not up?")
            return

        delta = {"x": 0.0, "y": 0.0, "z": 0.0,
                 "rx": 0.0, "ry": 0.0, "rz": 0.0}
        if kind == "trans":
            delta[axis] = sign * float(self.cmbLinStep.currentData())
        else:
            delta[f"r{axis}"] = sign * float(self.cmbAngStep.currentData())
        target = self._apply_delta(cur, delta)
        source = f"jog {arm}.{kind}.{axis}{sign:+d}"

        backend = self.cmbJogBackend.currentData() or "moveit"
        if backend == "cyclo_movel":
            duration = (float(self.spnPlanTime.value())
                        if hasattr(self, "spnPlanTime") else 2.0)
            ok = self._bridge.publish_cyclo_movel(arm, target, frame, duration)
            if not ok:
                self._set_status(f"cyclo MoveL publish failed (arm={arm})")
                return
            self._set_status(
                f"cyclo MoveL → {arm} [{frame}]  ({source})")
            return
        # MoveIt path: derive vel_scale from the user's chosen jog velocity
        # cap (cmbLinVel mm/s, cmbAngVel deg/s) instead of the global spnVel.
        if kind == "trans":
            v_mps = float(self.cmbLinVel.currentData())
            vel_scale = v_mps / MOVEIT_JOG_LIN_BASE_MPS
        else:
            v_rps = float(self.cmbAngVel.currentData())
            vel_scale = v_rps / MOVEIT_JOG_ANG_BASE_RPS
        self._send_target(target, execute=True, source=source,
                          frame_id_override=frame, arm_override=arm,
                          vel_override=vel_scale)

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

    def _send_target(self, target: dict, execute: bool, source: str,
                     frame_id_override: str = "",
                     arm_override: str = "",
                     vel_override: float = -1.0) -> None:
        arm = arm_override or self._current_arm()
        planner = self.cmbPlanner.currentData() or "PILZ_LIN"
        frame = frame_id_override or self._current_frame()
        # vel_override > 0 lets Jog supply a per-step velocity derived from the
        # user's chosen mm/s (or deg/s) cap instead of the global spnVel slider.
        vel = vel_override if vel_override > 0 else self.spnVel.value()
        vel = max(0.01, min(1.0, vel))
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
        d = jd._poses_dir()
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
        self._drag_debounce.stop()
