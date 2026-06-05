"""Joint Control tab for openarmx_scenario_ui.

A PyQt5 widget (ported from the Ti5 `tr_works_joint_control_ui` main window)
adapted to the openarmx bimanual arm: per-joint sliders + spinboxes, SIL/HIL
modes with a hardware safety gate, L<->R mirror, Save Pose, and a small Node
Manager. It drives the shared ScenarioRosBridge (no second rclpy context).
"""

from __future__ import annotations

import json
import math
import os
import time
from datetime import datetime

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QCheckBox, QComboBox, QDoubleSpinBox, QGridLayout, QGroupBox, QHBoxLayout,
    QInputDialog, QLabel, QMessageBox, QPushButton, QSlider, QVBoxLayout,
    QWidget,
)

from openarmx_scenario_ui import joint_data as jd
from openarmx_scenario_ui.cartesian_control_tab import (
    AXIS_LABEL_QSS, VALUE_CELL_QSS,
)

# Reference frame for the bimanual Cartesian readout: the shared bimanual base,
# so both arms' poses are expressed in one comparable frame.
_CART_REF_FRAME = "openarmx_body_link0"

# Bringup/launch is handled by the dedicated "Launch Manager" tab — this tab
# no longer owns its own bringup processes (avoids duplicate /controller_manager).

ARM_SCALE = jd.ARM_SCALE
GRIP_SCALE = jd.GRIP_SCALE


class JointControlTab(QWidget):
    def __init__(self, bridge, parent=None) -> None:
        super().__init__(parent)
        self._bridge = bridge

        # command widgets (deg for arms, meters for grippers)
        self._arm_spins = {}      # {joint: QDoubleSpinBox}  (deg)
        self._arm_sliders = {}    # {joint: QSlider}
        self._arm_actual = {}     # {joint: QDoubleSpinBox}  (deg, read-only)
        self._grip_spins = {}     # {joint: QDoubleSpinBox}  (meters)
        self._grip_sliders = {}
        self._grip_actual = {}

        # Live bimanual Cartesian readout cells: {arm: {x/y/z/R/P/Y: QLabel}}.
        self._cart_labels = {"left": {}, "right": {}}

        self._current = {}        # {joint: value} latest received (deg / m)
        self._last_recv_time = 0.0
        self._hardware_detected = False

        self._auto_publish = True
        self._mirror = False
        self._ctrl_confirmed = False  # one-time safety confirm before controller cmds
        self._ctrl_synced = False     # command spinboxes synced to actual once
        self._updating_mirror = False

        self._build_ui()

        # bridge feedback
        self._bridge.sig_joint_state.connect(self._on_joint_state_received)

        # SIL publish (20 Hz) keeps robot_state_publisher fed.
        self._sil_timer = QTimer(self)
        self._sil_timer.timeout.connect(self._on_sil_tick)
        self._sil_timer.start(50)

        # Actual-angle display (10 Hz) + hardware watchdog.
        self._display_timer = QTimer(self)
        self._display_timer.timeout.connect(self._on_display_tick)
        self._display_timer.start(100)

        # Debounce controller commands so dragging a slider doesn't flood the
        # trajectory controller with a goal every few milliseconds.
        self._cmd_debounce = QTimer(self)
        self._cmd_debounce.setSingleShot(True)
        self._cmd_debounce.timeout.connect(self._send_to_controllers)

        # Live Cartesian readout refresh (2 Hz) — cheap TF lookups per arm.
        self._cart_timer = QTimer(self)
        self._cart_timer.timeout.connect(self._refresh_cart)
        self._cart_timer.start(500)
        self._refresh_cart()

        self._set_status("SIL Mode — Ready")

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)

        arms_row = QHBoxLayout()
        arms_row.addWidget(self._build_arm_group("Left Arm", jd.LEFT_ARM_JOINTS))
        arms_row.addWidget(self._build_arm_group("Right Arm", jd.RIGHT_ARM_JOINTS))
        root.addLayout(arms_row)

        root.addWidget(self._build_gripper_group())
        root.addWidget(self._build_cartesian_group())
        root.addLayout(self._build_controls())

        hint = QLabel("ℹ Bringup/launch 실행은 'Launch Manager' 탭에서 합니다.")
        hint.setStyleSheet("color:#666;")
        root.addWidget(hint)

        self._lbl_status = QLabel("Ready")
        self._lbl_status.setStyleSheet("color:#444; padding:2px;")
        root.addWidget(self._lbl_status)
        root.addStretch()

    def _build_arm_group(self, title: str, joints: list) -> QGroupBox:
        grp = QGroupBox(title)
        grid = QGridLayout(grp)
        grid.addWidget(QLabel("<b>Joint</b>"), 0, 0)
        grid.addWidget(QLabel("<b>Slider</b>"), 0, 1)
        grid.addWidget(QLabel("<b>Cmd (°)</b>"), 0, 2)
        grid.addWidget(QLabel("<b>Actual (°)</b>"), 0, 3)
        for row, name in enumerate(joints, start=1):
            lo, hi = jd.JOINT_LIMITS_DEG[name]
            grid.addWidget(QLabel(name.replace("openarmx_", "")), row, 0)

            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(int(lo * ARM_SCALE))
            slider.setMaximum(int(hi * ARM_SCALE))
            grid.addWidget(slider, row, 1)

            spin = QDoubleSpinBox()
            spin.setRange(lo, hi)
            spin.setDecimals(1)
            spin.setSuffix(" °")
            grid.addWidget(spin, row, 2)

            actual = QDoubleSpinBox()
            actual.setRange(-360.0, 360.0)
            actual.setDecimals(1)
            actual.setSuffix(" °")
            actual.setReadOnly(True)
            actual.setButtonSymbols(QDoubleSpinBox.NoButtons)
            actual.setEnabled(False)
            grid.addWidget(actual, row, 3)

            self._arm_spins[name] = spin
            self._arm_sliders[name] = slider
            self._arm_actual[name] = actual

            slider.valueChanged.connect(
                lambda val, s=spin, n=name: self._slider_to_spin(
                    val, s, n, ARM_SCALE))
            spin.valueChanged.connect(
                lambda val, sl=slider: self._spin_to_slider(val, sl, ARM_SCALE))
            spin.valueChanged.connect(
                lambda val, n=name: self._on_arm_changed(n, val))
        grid.setRowStretch(len(joints) + 1, 1)
        return grp

    def _build_gripper_group(self) -> QGroupBox:
        grp = QGroupBox("Grippers (meters)")
        grid = QGridLayout(grp)
        for col, name in enumerate(jd.GRIPPER_NAMES):
            lo, hi = jd.JOINT_LIMITS[name]
            label = "Left" if "left" in name else "Right"
            grid.addWidget(QLabel(f"{label} gripper"), 0, col * 3)

            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(int(lo * GRIP_SCALE))
            slider.setMaximum(int(hi * GRIP_SCALE))
            grid.addWidget(slider, 0, col * 3 + 1)

            spin = QDoubleSpinBox()
            spin.setRange(lo, hi)
            spin.setDecimals(3)
            spin.setSingleStep(0.001)
            spin.setSuffix(" m")
            grid.addWidget(spin, 0, col * 3 + 2)

            actual = QDoubleSpinBox()
            actual.setRange(-1.0, 1.0)
            actual.setDecimals(3)
            actual.setSuffix(" m")
            actual.setReadOnly(True)
            actual.setButtonSymbols(QDoubleSpinBox.NoButtons)
            actual.setEnabled(False)
            grid.addWidget(actual, 1, col * 3 + 2)

            self._grip_spins[name] = spin
            self._grip_sliders[name] = slider
            self._grip_actual[name] = actual

            slider.valueChanged.connect(
                lambda val, s=spin: self._slider_to_spin(val, s, None, GRIP_SCALE))
            spin.valueChanged.connect(
                lambda val, sl=slider: self._spin_to_slider(val, sl, GRIP_SCALE))
            spin.valueChanged.connect(lambda _val: self._maybe_publish())
        return grp

    def _build_cartesian_group(self) -> QGroupBox:
        """Bimanual live Cartesian readout for both arms, with a shared link7/TCP
        EE-point selector. Poses are expressed in the bimanual base frame."""
        grp = QGroupBox(f"Cartesian pose (in {_CART_REF_FRAME})")
        v = QVBoxLayout(grp)

        hdr = QHBoxLayout()
        hdr.addWidget(QLabel("EE point:"))
        self.cmb_cart_point = QComboBox()
        self.cmb_cart_point.addItem("link7", userData="link7")
        self.cmb_cart_point.addItem("TCP", userData="hand_tcp")
        self.cmb_cart_point.currentIndexChanged.connect(self._refresh_cart)
        hdr.addWidget(self.cmb_cart_point)
        hdr.addStretch()
        v.addLayout(hdr)

        arms = QHBoxLayout()
        arms.addLayout(self._build_arm_cart_block("Left", "left"))
        arms.addLayout(self._build_arm_cart_block("Right", "right"))
        v.addLayout(arms)
        return grp

    def _build_arm_cart_block(self, title: str, arm: str) -> QVBoxLayout:
        box = QVBoxLayout()
        box.addWidget(QLabel(f"<b>{title}</b>"))
        grid = QGridLayout()
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(6)
        grid.addWidget(QLabel("<b>Position (m)</b>"), 0, 0, 1, 3)
        for c, ax in enumerate(("x", "y", "z")):
            self._cart_labels[arm][ax] = self._cart_cell(grid, ax, 1, c)
        grid.addWidget(QLabel("<b>Orientation (deg)</b>"), 2, 0, 1, 3)
        for c, ax in enumerate(("R", "P", "Y")):
            self._cart_labels[arm][ax] = self._cart_cell(grid, ax, 3, c)
        box.addLayout(grid)
        return box

    def _cart_cell(self, grid: QGridLayout, name: str, row: int, col: int):
        """One '<name> <value>' readout cell, styled like the Cartesian tab."""
        lbl_name = QLabel(name)
        lbl_name.setStyleSheet(AXIS_LABEL_QSS)
        lbl_val = QLabel("---")
        lbl_val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        lbl_val.setStyleSheet(VALUE_CELL_QSS)
        cell = QHBoxLayout()
        cell.setContentsMargins(0, 0, 0, 0)
        cell.setSpacing(6)
        cell.addWidget(lbl_name)
        cell.addWidget(lbl_val, 1)
        wrap = QWidget()
        wrap.setLayout(cell)
        grid.addWidget(wrap, row, col)
        return lbl_val

    def _refresh_cart(self) -> None:
        """Update both arms' Cartesian readouts via TF (link7 or hand TCP)."""
        if not hasattr(self, "cmb_cart_point"):
            return
        point = self.cmb_cart_point.currentData() or "link7"
        for arm in ("left", "right"):
            labels = self._cart_labels[arm]
            controlled = f"openarmx_{arm}_{point}"
            pose = self._bridge.get_ee_pose(arm, _CART_REF_FRAME, controlled)
            if not pose:
                for lbl in labels.values():
                    lbl.setText("---")
                continue
            labels["x"].setText(f"{pose['x']:+.4f}")
            labels["y"].setText(f"{pose['y']:+.4f}")
            labels["z"].setText(f"{pose['z']:+.4f}")
            labels["R"].setText(f"{math.degrees(pose['roll']):+.2f}")
            labels["P"].setText(f"{math.degrees(pose['pitch']):+.2f}")
            labels["Y"].setText(f"{math.degrees(pose['yaw']):+.2f}")

    def _build_controls(self) -> QHBoxLayout:
        row = QHBoxLayout()
        def _btn(text, bg, hover, pressed):
            b = QPushButton(text)
            b.setMinimumWidth(96)
            b.setMinimumHeight(34)
            b.setStyleSheet(
                f"QPushButton {{ background-color:{bg}; color:white; font-weight:bold; "
                f"border:none; border-radius:4px; padding:6px 14px; }}"
                f"QPushButton:hover {{ background-color:{hover}; }}"
                f"QPushButton:pressed {{ background-color:{pressed}; }}")
            return b

        # All four buttons same size; semantic colors (blue=home, orange=init,
        # green=execute/send, blue-grey=save/record).
        self.btn_home = _btn("Home", "#2196F3", "#1976D2", "#1565C0")
        self.btn_init = _btn("Init", "#FF9800", "#FB8C00", "#EF6C00")
        self.btn_send = _btn("Send", "#4CAF50", "#43A047", "#2E7D32")
        self.btn_save = _btn("Save Pose", "#607D8B", "#546E7A", "#455A64")
        self.chk_auto = QCheckBox("Auto publish")
        self.chk_auto.setChecked(True)
        self.chk_mirror = QCheckBox("Mirror L→R")
        self.spin_duration = QDoubleSpinBox()
        self.spin_duration.setRange(0.1, 60.0)
        self.spin_duration.setValue(0.5)  # short horizon → responsive jogging
        self.spin_duration.setSuffix(" s")

        self.btn_home.clicked.connect(self._on_home)
        self.btn_init.clicked.connect(self._on_init)
        self.btn_send.clicked.connect(self._on_send)
        self.btn_save.clicked.connect(self._on_save_pose)
        self.chk_auto.toggled.connect(self._on_auto_toggled)
        self.chk_mirror.toggled.connect(self._on_mirror_toggled)

        for w in (self.btn_home, self.btn_init, self.btn_send, self.btn_save,
                  self.chk_auto, self.chk_mirror):
            row.addWidget(w)
        row.addWidget(QLabel("Duration:"))
        row.addWidget(self.spin_duration)
        row.addStretch()
        return row

    # ------------------------------------------------------------------
    # Slider / spinbox sync
    # ------------------------------------------------------------------

    def _slider_to_spin(self, slider_val, spin, joint_name, scale) -> None:
        spin.blockSignals(True)
        spin.setValue(slider_val / scale)
        spin.blockSignals(False)
        if joint_name is not None:
            self._on_arm_changed(joint_name, slider_val / scale)
        else:
            self._maybe_publish()

    def _spin_to_slider(self, spin_val, slider, scale) -> None:
        slider.blockSignals(True)
        slider.setValue(int(spin_val * scale))
        slider.blockSignals(False)

    def _on_arm_changed(self, joint_name, value) -> None:
        if self._mirror and not self._updating_mirror:
            pair_info = jd.MIRROR_PAIRS.get(joint_name)
            if pair_info and pair_info[0] in self._arm_spins:
                pair, sign = pair_info
                self._updating_mirror = True
                self._set_single_arm(pair, self._clamp(pair, sign * value))
                self._updating_mirror = False
        self._maybe_publish()

    def _clamp(self, joint_name, value) -> float:
        lo, hi = jd.JOINT_LIMITS_DEG.get(joint_name, (-360.0, 360.0))
        return max(lo, min(hi, value))

    def _set_single_arm(self, name, value) -> None:
        spin = self._arm_spins.get(name)
        if spin:
            spin.blockSignals(True)
            spin.setValue(value)
            spin.blockSignals(False)
        slider = self._arm_sliders.get(name)
        if slider:
            slider.blockSignals(True)
            slider.setValue(int(value * ARM_SCALE))
            slider.blockSignals(False)

    # ------------------------------------------------------------------
    # State accessors
    # ------------------------------------------------------------------

    def _get_arm_deg(self) -> dict:
        return {n: s.value() for n, s in self._arm_spins.items()}

    def _get_gripper_m(self) -> dict:
        return {n: s.value() for n, s in self._grip_spins.items()}

    def _set_arm_deg(self, positions: dict) -> None:
        for name, spin in self._arm_spins.items():
            val = positions.get(name, spin.value())
            spin.blockSignals(True)
            spin.setValue(val)
            spin.blockSignals(False)
            slider = self._arm_sliders.get(name)
            if slider:
                slider.blockSignals(True)
                slider.setValue(int(val * ARM_SCALE))
                slider.blockSignals(False)

    def _set_gripper_m(self, positions: dict) -> None:
        for name, spin in self._grip_spins.items():
            if name not in positions:
                continue
            val = positions[name]
            spin.blockSignals(True)
            spin.setValue(val)
            spin.blockSignals(False)
            slider = self._grip_sliders.get(name)
            if slider:
                slider.blockSignals(True)
                slider.setValue(int(val * GRIP_SCALE))
                slider.blockSignals(False)

    # ------------------------------------------------------------------
    # Presets / buttons
    # ------------------------------------------------------------------

    def _on_home(self) -> None:
        self._set_arm_deg(jd.HOME_POSITION_DEG)
        self._set_gripper_m(jd.HOME_GRIPPER_M)
        self._publish()   # always command (move), regardless of Auto publish
        self._set_status("Home position")

    def _on_init(self) -> None:
        self._set_arm_deg(jd.INIT_POSITION_DEG)
        self._set_gripper_m(jd.INIT_GRIPPER_M)
        self._publish()   # always command (move), regardless of Auto publish
        self._set_status("Init position")

    def _on_send(self) -> None:
        self._publish()
        self._set_status("Sent")

    def _on_save_pose(self) -> None:
        name, ok = QInputDialog.getText(self, "Save Pose", "Pose name:")
        if not ok or not name.strip():
            return
        name = name.strip().replace(" ", "_")
        arm = self._get_arm_deg()
        pose = {
            "name": name,
            "created": datetime.now().isoformat(timespec="seconds"),
            "left_deg": [arm[n] for n in jd.LEFT_ARM_JOINTS],
            "right_deg": [arm[n] for n in jd.RIGHT_ARM_JOINTS],
            "gripper_m": [self._grip_spins[n].value() for n in jd.GRIPPER_NAMES],
        }
        d = jd._poses_dir()
        os.makedirs(d, exist_ok=True)
        path = os.path.join(d, f"{name}.json")
        with open(path, "w") as f:
            json.dump(pose, f, indent=2, ensure_ascii=False)
        self._set_status(f"Pose saved: {path}")

    def _on_auto_toggled(self, checked) -> None:
        self._auto_publish = checked
        self._set_status(f"Auto publish: {'ON' if checked else 'OFF'}")

    def _on_mirror_toggled(self, checked) -> None:
        self._mirror = checked
        self._set_status(f"Mirror L→R: {'ON' if checked else 'OFF'}")

    # ------------------------------------------------------------------
    # Controller-command gate (one-time safety confirm)
    # ------------------------------------------------------------------

    def _ensure_ctrl_ok(self) -> bool:
        """Confirm once per session before commanding active controllers.
        With fake HW it's harmless; with real HW it drives motors."""
        if self._ctrl_confirmed:
            return True
        reply = QMessageBox.warning(
            self, "Send to controllers",
            "활성 트래젝토리 컨트롤러로 명령을 전송합니다.\n\n"
            "⚠ 실제 하드웨어 bringup이면 모터가 구동됩니다. (fake HW면 안전)\n"
            "계속하시겠습니까?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply != QMessageBox.Yes:
            return False
        self._ctrl_confirmed = True
        self._bridge.clear_self_echo()
        return True

    # ------------------------------------------------------------------
    # Publish
    # ------------------------------------------------------------------

    def _controllers_active(self) -> bool:
        """True when a trajectory controller is subscribed (ros2_control bringup
        is up). Then /joint_states is owned by joint_state_broadcaster and the
        robot can only be moved by commanding the controllers."""
        return self._bridge.traj_subscriber_count() > 0

    def _clamped_arm_grip(self) -> tuple:
        arm = {n: self._clamp(n, v) for n, v in self._get_arm_deg().items()}
        grip = {}
        for n, v in self._get_gripper_m().items():
            lo, hi = jd.JOINT_LIMITS[n]
            grip[n] = max(lo, min(hi, v))
        return arm, grip

    def _maybe_publish(self) -> None:
        """Called on every command change (slider/spinbox). Auto-routes:
        controllers active → command them (debounced); else → /joint_states."""
        if not self._auto_publish:
            return
        if self._controllers_active():
            if not self._ensure_ctrl_ok():
                return
            self._cmd_debounce.start(120)   # debounced trajectory send
        else:
            arm, grip = self._clamped_arm_grip()
            self._bridge.publish_joint_states(arm, grip)

    def _send_to_controllers(self) -> None:
        """Send a trajectory + gripper goal to the active controllers."""
        arm, grip = self._clamped_arm_grip()
        duration = self.spin_duration.value()
        self._bridge.send_trajectory(arm, duration)
        self._bridge.send_gripper(grip)
        self._set_status(f"Sent to controllers (duration={duration:.1f}s)")

    def _publish(self) -> None:
        """Explicit Send button: route by controller presence."""
        if self._controllers_active():
            if not self._ensure_ctrl_ok():
                return
            self._send_to_controllers()
        else:
            arm, grip = self._clamped_arm_grip()
            self._bridge.publish_joint_states(arm, grip)
            self._set_status("Published /joint_states (SIL)")

    def _on_sil_tick(self) -> None:
        # When a trajectory controller is active the joint_state_broadcaster owns
        # /joint_states, so SIL must NOT publish (dual-publish). Also DROP the
        # stale self-echo filter: it was armed by our last SIL publish before the
        # controllers came up, and if the broadcaster's real feedback happens to
        # match those values (e.g. at the home/zero pose) it would be silently
        # dropped — freezing the joint readout and breaking Teaching Capture.
        if self._bridge.traj_subscriber_count() > 0:
            self._bridge.clear_self_echo()
            return
        arm = {n: self._clamp(n, v) for n, v in self._get_arm_deg().items()}
        self._bridge.publish_joint_states(arm, self._get_gripper_m())

    # ------------------------------------------------------------------
    # Feedback display
    # ------------------------------------------------------------------

    def _on_joint_state_received(self, positions: dict) -> None:
        for name, val in positions.items():
            if name in self._arm_actual or name in self._grip_actual:
                self._current[name] = val
        self._last_recv_time = time.monotonic()

    def _on_display_tick(self) -> None:
        now = time.monotonic()
        hw_present = (now - self._last_recv_time) < 1.5 and self._last_recv_time > 0
        if hw_present != self._hardware_detected:
            self._hardware_detected = hw_present
            for spin in list(self._arm_actual.values()) + list(self._grip_actual.values()):
                spin.setEnabled(hw_present)
        if not hw_present:
            return
        for name, spin in self._arm_actual.items():
            if name in self._current and abs(spin.value() - self._current[name]) > 0.05:
                spin.setValue(self._current[name])
        for name, spin in self._grip_actual.items():
            if name in self._current and abs(spin.value() - self._current[name]) > 0.0005:
                spin.setValue(self._current[name])

        # When controllers are active, sync command spinboxes to actual ONCE so
        # the first slider command doesn't yank untouched joints from the real
        # pose. Resets if controllers go away.
        if self._controllers_active():
            if not self._ctrl_synced and self._current:
                self._set_arm_deg({n: v for n, v in self._current.items()
                                   if n in self._arm_spins})
                self._set_gripper_m({n: v for n, v in self._current.items()
                                     if n in self._grip_spins})
                self._ctrl_synced = True
        else:
            self._ctrl_synced = False

    # ------------------------------------------------------------------

    def _set_status(self, text: str) -> None:
        self._lbl_status.setText(text)

    def shutdown(self) -> None:
        self._sil_timer.stop()
        self._display_timer.stop()
        self._cmd_debounce.stop()
        self._cart_timer.stop()
