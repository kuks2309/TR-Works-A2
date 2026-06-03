"""Teaching tab for openarmx_scenario_ui.

Captures bimanual waypoints (per-arm joint angles + gripper opening AND the
per-arm Cartesian EE pose) into an editable two-table list, then exports a
self-contained scenario (top scenario.json with "sequence" + a "main" sub with
"steps") that the Scenario Player can replay.

Data model: a single list `self._waypoints` is the source of truth; both tables
are a render of it (`_render_all`), so the joint table and the Cartesian table
stay in lock-step. Inline editing IS allowed (user decision) — hand-edited
joint values are clamped to JOINT_LIMITS, hand-edited RPY recomputes the stored
quaternion. No FK/IK coupling between the two tables (a deliberate, documented
divergence the user accepts).
"""

from __future__ import annotations

import json
import math
import os

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QAbstractItemView, QButtonGroup, QComboBox, QFileDialog, QGroupBox,
    QHBoxLayout, QHeaderView, QInputDialog, QLabel, QLineEdit, QMessageBox,
    QPushButton, QRadioButton, QSplitter, QTableWidget, QTableWidgetItem,
    QVBoxLayout, QWidget,
)

from openarmx_scenario_ui import joint_data as jd
from openarmx_scenario_ui.cartesian_control_tab import (
    AXIS_LABEL_QSS, VALUE_CELL_QSS,
)
from openarmx_scenario_ui.geometry_utils import quat_to_rpy, rpy_to_quat


# --- Joint table layout: Pose명 | L j1..L j7 | R j1..R j7 | L grip | R grip ---
# 1 + 7 + 7 + 2 = 17 columns (designer's "18" was a miscount; verified 17).
_JOINT_HEADERS = (
    ["Pose명"]
    + [f"L j{i}" for i in range(1, 8)]
    + [f"R j{i}" for i in range(1, 8)]
    + ["L grip", "R grip"]
)
JOINT_COL_COUNT = len(_JOINT_HEADERS)  # 17

# --- Cartesian table layout: Pose명 | L x,y,z,R,P,Y | R x,y,z,R,P,Y ---
# 1 + 6 + 6 = 13 columns.
_CART_HEADERS = (
    ["Pose명"]
    + [f"L {k}" for k in ("x", "y", "z", "R", "P", "Y")]
    + [f"R {k}" for k in ("x", "y", "z", "R", "P", "Y")]
)
CART_COL_COUNT = len(_CART_HEADERS)  # 13

# Uniform width (px) for every value column in BOTH tables. The name column (0)
# stretches to absorb the remaining width; all numeric columns share this exact
# width so the joint/gripper and Cartesian grids line up instead of being sized
# per-content (which made the grid jagged).
_VALUE_COL_W = 72

# Capture-frame choices. "" sentinel = resolve per-arm to openarmx_<arm>_link0.
_FRAME_CHOICES = [
    ("(arm base) openarmx_<arm>_link0", ""),
    ("openarmx_body_link0", "openarmx_body_link0"),
    ("world", "world"),
]

_LIVENESS_FRESH_SEC = 1.5
_NOTE_MANUAL_EDIT = (
    "수동 편집됨 — 조인트/카테시안 표가 물리적으로 불일치할 수 있음.")


def _btn(text, bg, hover, pressed):
    """Colored action button — same look as joint_control_tab._btn."""
    b = QPushButton(text)
    b.setMinimumWidth(96)
    b.setMinimumHeight(34)
    b.setStyleSheet(
        f"QPushButton {{ background-color:{bg}; color:white; font-weight:bold; "
        f"border:none; border-radius:4px; padding:6px 14px; }}"
        f"QPushButton:hover {{ background-color:{hover}; }}"
        f"QPushButton:pressed {{ background-color:{pressed}; }}")
    return b


class TeachingTab(QWidget):
    def __init__(self, bridge, parent=None) -> None:
        super().__init__(parent)
        self._bridge = bridge

        # Source of truth — both tables render from this list.
        self._waypoints: list[dict] = []

        # Latest joint state (deg arms / m grippers), cached from the bridge.
        self._latest_joints: dict = {}
        self._latest_joints_t = 0.0

        # Guards against itemChanged recursion during programmatic fills.
        self._loading = False
        # Guards against selection-mirror recursion between the two tables.
        self._syncing_sel = False

        self._build_ui()

        self._bridge.sig_joint_state.connect(self._on_joint_state)

        # Liveness indicator (500 ms): /joint_states + EE TF freshness.
        self._live_timer = QTimer(self)
        self._live_timer.timeout.connect(self._update_liveness)
        self._live_timer.start(500)
        self._update_liveness()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)

        root.addLayout(self._build_header())

        splitter = QSplitter(Qt.Vertical)
        splitter.addWidget(self._build_joint_group())
        splitter.addWidget(self._build_cart_group())
        splitter.setSizes([500, 500])
        root.addWidget(splitter, 1)

        root.addLayout(self._build_row_ops())
        root.addLayout(self._build_save_row())

        self._lbl_status = QLabel("Ready — 0 waypoints.")
        self._lbl_status.setStyleSheet(
            "color:#444; padding:4px; font-family:monospace;")
        root.addWidget(self._lbl_status)

    def _build_header(self) -> QHBoxLayout:
        h = QHBoxLayout()
        h.addWidget(QLabel("Capture frame:"))
        self.cmbFrame = QComboBox()
        for label, data in _FRAME_CHOICES:
            self.cmbFrame.addItem(label, userData=data)
        self.cmbFrame.setCurrentIndex(0)  # arm-base default
        h.addWidget(self.cmbFrame)
        h.addSpacing(24)

        self.lblLiveJS = QLabel("● /joint_states")
        self.lblLiveTF = QLabel("● EE TF")
        for lbl in (self.lblLiveJS, self.lblLiveTF):
            lbl.setStyleSheet("color:#9e9e9e; font-weight:bold;")
            h.addWidget(lbl)
        h.addStretch()
        return h

    def _make_table(self, headers: list) -> QTableWidget:
        t = QTableWidget(0, len(headers))
        t.setHorizontalHeaderLabels(headers)
        t.setSelectionBehavior(QAbstractItemView.SelectRows)
        t.setSelectionMode(QAbstractItemView.SingleSelection)
        t.verticalHeader().setDefaultSectionSize(26)
        hh = t.horizontalHeader()
        # Uniform value-column widths (project request): every numeric/value
        # column is the same fixed width (_VALUE_COL_W) in both tables instead of
        # per-content sizing; the name column stretches to take the slack.
        hh.setMinimumSectionSize(0)
        for c in range(1, len(headers)):
            hh.setSectionResizeMode(c, QHeaderView.Fixed)
            t.setColumnWidth(c, _VALUE_COL_W)
        hh.setSectionResizeMode(0, QHeaderView.Stretch)         # name col
        # Value cells use the shared gray look (like the Jog/Marker grids);
        # column headers (the per-axis labels) use the shared bold axis style.
        # Explicit dark text + a light-blue selected state. Without this, the
        # QTableWidget::item background override removes Qt's selection color and
        # the selected row keeps the default white HighlightedText → white text
        # on the #f0f0f0 cell = invisible.
        t.setStyleSheet(
            f"QTableWidget::item {{ {VALUE_CELL_QSS} color:#222; }}"
            "QTableWidget::item:selected { background:#cfe3fb; color:#111; }"
            f"QHeaderView::section {{ {AXIS_LABEL_QSS} }}")
        t.itemChanged.connect(
            lambda item, tbl=t: self._on_item_changed(tbl, item))
        t.itemSelectionChanged.connect(
            lambda tbl=t: self._on_selection_changed(tbl))
        return t

    def _build_joint_group(self) -> QGroupBox:
        grp = QGroupBox("조인트 + 그리퍼 웨이포인트 (deg / m)")
        v = QVBoxLayout(grp)
        self._tbl_joint = self._make_table(_JOINT_HEADERS)
        v.addWidget(self._tbl_joint)
        return grp

    def _build_cart_group(self) -> QGroupBox:
        grp = QGroupBox("카테시안 웨이포인트 (m / deg)")
        v = QVBoxLayout(grp)
        self._tbl_cart = self._make_table(_CART_HEADERS)
        v.addWidget(self._tbl_cart)
        return grp

    def _build_row_ops(self) -> QHBoxLayout:
        h = QHBoxLayout()
        self.btnCapture = _btn("Capture", "#4CAF50", "#43A047", "#2E7D32")
        self.btnDelete = _btn("Delete", "#c62828", "#b71c1c", "#8e0000")
        self.btnUp = _btn("Move Up", "#607D8B", "#546E7A", "#455A64")
        self.btnDown = _btn("Move Down", "#607D8B", "#546E7A", "#455A64")
        self.btnClear = _btn("Clear", "#9e9e9e", "#757575", "#616161")
        self.btnGoto = _btn("Go to Pose", "#FF9800", "#FB8C00", "#EF6C00")

        self.btnCapture.clicked.connect(self._on_capture)
        self.btnDelete.clicked.connect(self._on_delete)
        self.btnUp.clicked.connect(lambda: self._on_move(-1))
        self.btnDown.clicked.connect(lambda: self._on_move(+1))
        self.btnClear.clicked.connect(self._on_clear)
        self.btnGoto.clicked.connect(self._on_goto)

        for b in (self.btnCapture, self.btnDelete, self.btnUp,
                  self.btnDown, self.btnClear):
            h.addWidget(b)
        h.addStretch()
        h.addWidget(self.btnGoto)
        return h

    def _build_save_row(self) -> QHBoxLayout:
        h = QHBoxLayout()
        h.addWidget(QLabel("Scenario root:"))
        self.edtRoot = QLineEdit(self._default_scenario_root())
        h.addWidget(self.edtRoot, 1)
        self.btnBrowse = QPushButton("Browse…")
        self.btnBrowse.clicked.connect(self._on_browse)
        h.addWidget(self.btnBrowse)

        h.addSpacing(16)
        h.addWidget(QLabel("Sub명:"))
        self.edtSub = QLineEdit("teach_seq")
        self.edtSub.setMaximumWidth(160)
        h.addWidget(self.edtSub)

        self.rdoMovel = QRadioButton("movel")
        self.rdoMovej = QRadioButton("movej")
        self.rdoMovej.setChecked(True)  # movej default
        grp_type = QButtonGroup(self)
        grp_type.addButton(self.rdoMovel)
        grp_type.addButton(self.rdoMovej)
        h.addWidget(self.rdoMovel)
        h.addWidget(self.rdoMovej)

        self.btnSave = _btn("Save", "#2196F3", "#1976D2", "#1565C0")
        self.btnLoad = _btn("Load", "#607D8B", "#546E7A", "#455A64")
        self.btnSave.clicked.connect(self._on_save)
        self.btnLoad.clicked.connect(self._on_load)
        h.addWidget(self.btnSave)
        h.addWidget(self.btnLoad)
        return h

    @staticmethod
    def _default_scenario_root() -> str:
        # Reuse the canonical resolver (env → ~/openarmx_ws/scenarios → walk-up
        # to the repo scenarios/) so Save lands exactly where the Scenario Player
        # reads. Lazy import avoids a circular import (main_window imports this).
        try:
            from openarmx_scenario_ui.main_window import _find_scenarios_dir
            return _find_scenarios_dir()
        except Exception:
            env = os.environ.get("OPENARMX_SCENARIOS_DIR")
            if env and os.path.isdir(env):
                return env
            return os.path.expanduser("~/openarmx_ws/scenarios")

    # ------------------------------------------------------------------
    # Liveness + joint-state cache
    # ------------------------------------------------------------------

    def _on_joint_state(self, positions: dict) -> None:
        self._latest_joints = dict(positions)
        self._latest_joints_t = _now()

    def _update_liveness(self) -> None:
        now = _now()
        js_fresh = (self._latest_joints_t > 0
                    and (now - self._latest_joints_t) < _LIVENESS_FRESH_SEC)
        self._set_dot(self.lblLiveJS, js_fresh)
        # EE TF freshness: a left-arm pose lookup succeeding means TF is live.
        tf_fresh = self._bridge.get_ee_pose("left", "") is not None
        self._set_dot(self.lblLiveTF, tf_fresh)

    @staticmethod
    def _set_dot(lbl: QLabel, fresh: bool) -> None:
        color = "#2e7d32" if fresh else "#9e9e9e"
        lbl.setStyleSheet(f"color:{color}; font-weight:bold;")

    # ------------------------------------------------------------------
    # Capture
    # ------------------------------------------------------------------

    def _resolve_frame(self, arm: str) -> str:
        sel = self.cmbFrame.currentData()
        return sel if sel else f"openarmx_{arm}_link0"

    def _on_capture(self) -> None:
        if not self._latest_joints:
            QMessageBox.warning(
                self, "Capture",
                "/joint_states 미수신 — 로봇/시뮬레이션이 떠 있는지 확인하세요.")
            return

        poses = {}
        for arm in ("left", "right"):
            frame = self._resolve_frame(arm)
            p = self._bridge.get_ee_pose(arm, frame)
            if p is None:
                QMessageBox.warning(
                    self, "Capture",
                    f"{arm} arm EE TF lookup 실패 (frame={frame}). "
                    "bringup/TF 확인.")
                return
            poses[arm] = p

        default = f"pose_{len(self._waypoints)}"
        name, ok = QInputDialog.getText(
            self, "Capture waypoint", "Pose 이름:", text=default)
        if not ok:
            return
        name = (name.strip() or default).replace(" ", "_")

        wp = {
            "name": name,
            "left_deg": [self._latest_joints.get(j, 0.0)
                         for j in jd.LEFT_ARM_JOINTS],
            "right_deg": [self._latest_joints.get(j, 0.0)
                          for j in jd.RIGHT_ARM_JOINTS],
            "gripper_m": [self._latest_joints.get(jd.LEFT_GRIPPER, 0.0),
                          self._latest_joints.get(jd.RIGHT_GRIPPER, 0.0)],
            "left_cart": _pose_to_cart(poses["left"]),
            "right_cart": _pose_to_cart(poses["right"]),
        }
        self._waypoints.append(wp)
        self._render_all()
        self._select_row(len(self._waypoints) - 1)
        self._set_status(
            f"Captured '{name}' — {len(self._waypoints)} waypoints.")

    # ------------------------------------------------------------------
    # Row operations (mutate list → _render_all → reselect)
    # ------------------------------------------------------------------

    def _current_row(self) -> int:
        r = self._tbl_joint.currentRow()
        if r < 0:
            r = self._tbl_cart.currentRow()
        return r

    def _on_delete(self) -> None:
        r = self._current_row()
        if r < 0 or r >= len(self._waypoints):
            self._set_status("Delete: 선택된 웨이포인트 없음.")
            return
        name = self._waypoints[r]["name"]
        del self._waypoints[r]
        self._render_all()
        if self._waypoints:
            self._select_row(min(r, len(self._waypoints) - 1))
        self._set_status(
            f"Deleted '{name}' — {len(self._waypoints)} waypoints.")

    def _on_move(self, direction: int) -> None:
        r = self._current_row()
        if r < 0 or r >= len(self._waypoints):
            self._set_status("Move: 선택된 웨이포인트 없음.")
            return
        new = r + direction
        if new < 0 or new >= len(self._waypoints):
            return
        self._waypoints[r], self._waypoints[new] = (
            self._waypoints[new], self._waypoints[r])
        self._render_all()
        self._select_row(new)
        self._set_status(f"Moved to position {new}.")

    def _on_clear(self) -> None:
        if not self._waypoints:
            return
        ans = QMessageBox.question(
            self, "Clear", "모든 웨이포인트를 삭제하시겠습니까?")
        if ans != QMessageBox.Yes:
            return
        self._waypoints.clear()
        self._render_all()
        self._set_status("Cleared — 0 waypoints.")

    # ------------------------------------------------------------------
    # Render (both tables from the list) + selection sync
    # ------------------------------------------------------------------

    def _render_all(self) -> None:
        self._loading = True
        try:
            self._render_joint_table()
            self._render_cart_table()
        finally:
            self._loading = False

    def _render_joint_table(self) -> None:
        t = self._tbl_joint
        t.setRowCount(len(self._waypoints))
        for r, wp in enumerate(self._waypoints):
            t.setItem(r, 0, _cell(wp["name"], editable=True))
            col = 1
            for v in wp["left_deg"]:
                t.setItem(r, col, _cell(f"{v:+.1f}", editable=True))
                col += 1
            for v in wp["right_deg"]:
                t.setItem(r, col, _cell(f"{v:+.1f}", editable=True))
                col += 1
            for v in wp["gripper_m"]:
                t.setItem(r, col, _cell(f"{v:+.3f}", editable=True))
                col += 1

    def _render_cart_table(self) -> None:
        t = self._tbl_cart
        t.setRowCount(len(self._waypoints))
        for r, wp in enumerate(self._waypoints):
            t.setItem(r, 0, _cell(wp["name"], editable=True))
            col = 1
            for side in ("left_cart", "right_cart"):
                c = wp[side]
                roll, pitch, yaw = quat_to_rpy(
                    c["qx"], c["qy"], c["qz"], c["qw"])
                t.setItem(r, col, _cell(f"{c['x']:+.4f}", editable=True)); col += 1
                t.setItem(r, col, _cell(f"{c['y']:+.4f}", editable=True)); col += 1
                t.setItem(r, col, _cell(f"{c['z']:+.4f}", editable=True)); col += 1
                t.setItem(r, col, _cell(f"{math.degrees(roll):+.2f}", editable=True)); col += 1
                t.setItem(r, col, _cell(f"{math.degrees(pitch):+.2f}", editable=True)); col += 1
                t.setItem(r, col, _cell(f"{math.degrees(yaw):+.2f}", editable=True)); col += 1

    def _select_row(self, row: int) -> None:
        if row < 0 or row >= len(self._waypoints):
            return
        self._syncing_sel = True
        try:
            self._tbl_joint.selectRow(row)
            self._tbl_cart.selectRow(row)
        finally:
            self._syncing_sel = False

    def _on_selection_changed(self, table: QTableWidget) -> None:
        if self._syncing_sel:
            return
        row = table.currentRow()
        if row < 0:
            return
        other = self._tbl_cart if table is self._tbl_joint else self._tbl_joint
        self._syncing_sel = True
        try:
            other.blockSignals(True)
            other.selectRow(row)
            other.blockSignals(False)
        finally:
            self._syncing_sel = False

    # ------------------------------------------------------------------
    # Inline editing (user-approved divergence: no FK/IK coupling)
    # ------------------------------------------------------------------

    def _on_item_changed(self, table: QTableWidget, item) -> None:
        if self._loading:
            return
        row, col = item.row(), item.column()
        if row < 0 or row >= len(self._waypoints):
            return
        wp = self._waypoints[row]

        if col == 0:
            self._edit_name(wp, table, item)
            return
        if table is self._tbl_joint:
            self._edit_joint_cell(wp, table, item, row, col)
        else:
            self._edit_cart_cell(wp, table, item, row, col)

    def _edit_name(self, wp: dict, table: QTableWidget, item) -> None:
        new = item.text().strip().replace(" ", "_")
        wp["name"] = new
        # Mirror to the other table's name column (blockSignals to avoid loop).
        other = self._tbl_cart if table is self._tbl_joint else self._tbl_joint
        self._set_cell_text(table, item.row(), 0, new, item)
        self._set_cell_text(other, item.row(), 0, new)
        self._set_status(f"Renamed → '{new}'.")

    def _edit_joint_cell(self, wp: dict, table: QTableWidget, item,
                         row: int, col: int) -> None:
        idx = col - 1  # 0..15
        try:
            val = float(item.text())
        except ValueError:
            self._revert_joint_cell(table, item, wp, idx)
            self._set_status("편집 실패: 숫자가 아님 — 원래 값으로 복원.")
            return

        if idx < 7:                       # left arm joint
            joint = jd.LEFT_ARM_JOINTS[idx]
            lo, hi = jd.JOINT_LIMITS.get(joint, (-360.0, 360.0))
            val = max(lo, min(hi, val))
            wp["left_deg"][idx] = val
            fmt = f"{val:+.1f}"
        elif idx < 14:                    # right arm joint
            j = idx - 7
            joint = jd.RIGHT_ARM_JOINTS[j]
            lo, hi = jd.JOINT_LIMITS.get(joint, (-360.0, 360.0))
            val = max(lo, min(hi, val))
            wp["right_deg"][j] = val
            fmt = f"{val:+.1f}"
        else:                             # gripper (idx 14=L, 15=R)
            g = idx - 14
            joint = jd.GRIPPER_NAMES[g]
            lo, hi = jd.JOINT_LIMITS.get(joint, (0.0, 0.044))
            val = max(lo, min(hi, val))
            wp["gripper_m"][g] = val
            fmt = f"{val:+.3f}"

        self._set_cell_text(table, row, col, fmt, item)
        self._set_status(_NOTE_MANUAL_EDIT)

    def _edit_cart_cell(self, wp: dict, table: QTableWidget, item,
                        row: int, col: int) -> None:
        idx = col - 1  # 0..11
        side = "left_cart" if idx < 6 else "right_cart"
        local = idx % 6  # 0:x 1:y 2:z 3:R 4:P 5:Y
        cart = wp[side]
        try:
            val = float(item.text())
        except ValueError:
            self._revert_cart_cell(table, item, cart, local)
            self._set_status("편집 실패: 숫자가 아님 — 원래 값으로 복원.")
            return

        if local < 3:                     # position x/y/z (m)
            key = ("x", "y", "z")[local]
            cart[key] = val
            self._set_cell_text(table, row, col, f"{val:+.4f}", item)
        else:                             # orientation R/P/Y (deg) → quat
            roll, pitch, yaw = quat_to_rpy(
                cart["qx"], cart["qy"], cart["qz"], cart["qw"])
            rpy = [roll, pitch, yaw]
            rpy[local - 3] = math.radians(val)
            qx, qy, qz, qw = rpy_to_quat(rpy[0], rpy[1], rpy[2])
            cart["qx"], cart["qy"], cart["qz"], cart["qw"] = qx, qy, qz, qw
            self._set_cell_text(table, row, col, f"{val:+.2f}", item)
        self._set_status(_NOTE_MANUAL_EDIT)

    def _revert_joint_cell(self, table, item, wp, idx) -> None:
        if idx < 7:
            self._set_cell_text(table, item.row(), item.column(),
                                f"{wp['left_deg'][idx]:+.1f}", item)
        elif idx < 14:
            self._set_cell_text(table, item.row(), item.column(),
                                f"{wp['right_deg'][idx - 7]:+.1f}", item)
        else:
            self._set_cell_text(table, item.row(), item.column(),
                                f"{wp['gripper_m'][idx - 14]:+.3f}", item)

    def _revert_cart_cell(self, table, item, cart, local) -> None:
        if local < 3:
            key = ("x", "y", "z")[local]
            self._set_cell_text(table, item.row(), item.column(),
                                f"{cart[key]:+.4f}", item)
        else:
            roll, pitch, yaw = quat_to_rpy(
                cart["qx"], cart["qy"], cart["qz"], cart["qw"])
            deg = math.degrees((roll, pitch, yaw)[local - 3])
            self._set_cell_text(table, item.row(), item.column(),
                                f"{deg:+.2f}", item)

    def _set_cell_text(self, table: QTableWidget, row: int, col: int,
                       text: str, item=None) -> None:
        """Set a cell's text without retriggering itemChanged."""
        was = self._loading
        self._loading = True
        try:
            cell = item if (item is not None and item.tableWidget() is table
                            and item.row() == row and item.column() == col) \
                else table.item(row, col)
            if cell is not None:
                cell.setText(text)
        finally:
            self._loading = was

    # ------------------------------------------------------------------
    # Save (self-contained scenario)
    # ------------------------------------------------------------------

    def _step_type(self) -> str:
        return "movej" if self.rdoMovej.isChecked() else "movel"

    def _on_save(self) -> None:
        if not self._waypoints:
            self._set_status("Save: 웨이포인트 없음.")
            return
        root = self.edtRoot.text().strip()
        sub = (self.edtSub.text().strip() or "teach_seq").replace(" ", "_")
        if not root:
            self._set_status("Save: Scenario root 비어 있음.")
            return

        step_type = self._step_type()
        planner = "pilz_ptp" if step_type == "movej" else "pilz_lin"
        steps = []
        for wp in self._waypoints:
            wp_name = wp["name"]
            for arm, cart_key, deg_key in (
                ("left", "left_cart", "left_deg"),
                ("right", "right_cart", "right_deg"),
            ):
                c = wp[cart_key]
                steps.append({
                    "name": f"{wp_name}_{arm}",
                    "type": step_type,
                    "arm": arm,
                    "frame_id": f"openarmx_{arm}_link0",
                    "target_pose": {
                        "position": [c["x"], c["y"], c["z"]],
                        "orientation_xyzw": [c["qx"], c["qy"],
                                             c["qz"], c["qw"]],
                    },
                    "planner": planner,
                    "max_vel_scale": 0.1,
                    "max_acc_scale": 0.1,
                    "plan_time": 5.0,
                    "joint_record": {f"{arm}_deg": list(wp[deg_key])},
                })
            for arm, gi in (("left", 0), ("right", 1)):
                steps.append({
                    "name": f"{wp_name}_grip_{arm}",
                    "type": "gripper",
                    "arm": arm,
                    "position": wp["gripper_m"][gi],
                    "max_effort": 14.0,
                })

        sub_dir = os.path.join(root, sub)
        main_dir = os.path.join(sub_dir, "main")
        try:
            os.makedirs(main_dir, exist_ok=True)
            with open(os.path.join(sub_dir, "scenario.json"),
                      "w", encoding="utf-8") as f:
                json.dump({
                    "name": sub,
                    "description": "Taught via Teaching tab",
                    "sequence": ["main"],
                }, f, indent=2, ensure_ascii=False)
            with open(os.path.join(main_dir, "scenario.json"),
                      "w", encoding="utf-8") as f:
                json.dump({"name": "main", "steps": steps},
                          f, indent=2, ensure_ascii=False)
        except OSError as exc:
            self._set_status(f"Save 실패: {exc}")
            return
        self._set_status(
            f"Saved {len(steps)} steps → {sub_dir}. "
            "Refresh Scenario Player to play.")

    # ------------------------------------------------------------------
    # Load
    # ------------------------------------------------------------------

    def _on_browse(self) -> None:
        start = self.edtRoot.text().strip() or os.path.expanduser("~")
        d = QFileDialog.getExistingDirectory(
            self, "Select Scenario root", start)
        if d:
            self.edtRoot.setText(d)

    def _on_load(self) -> None:
        start = self.edtRoot.text().strip() or os.path.expanduser("~")
        path, _ = QFileDialog.getOpenFileName(
            self, "Load sub scenario.json", start, "JSON (*.json)")
        if not path:
            return
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except (OSError, json.JSONDecodeError) as exc:
            self._set_status(f"Load 실패: {exc}")
            return
        steps = data.get("steps") if isinstance(data, dict) else None
        if not isinstance(steps, list):
            self._set_status("Load 실패: 'steps' 없음 (sub scenario.json 선택).")
            return

        # Re-group steps by pose name. Strip the per-step suffix to recover the
        # common waypoint prefix; preserve first-seen order.
        order: list = []
        groups: dict = {}
        for s in steps:
            if not isinstance(s, dict):
                continue
            name = str(s.get("name", ""))
            stype = (s.get("type") or "").lower()
            arm = (s.get("arm") or "").lower()
            if stype == "gripper":
                suffix = f"_grip_{arm}"
            else:
                suffix = f"_{arm}"
            prefix = name[:-len(suffix)] if suffix and name.endswith(suffix) \
                else name
            if prefix not in groups:
                groups[prefix] = self._blank_wp(prefix)
                order.append(prefix)
            self._fill_wp_from_step(groups[prefix], s, stype, arm)

        had_missing_joints = any(g.pop("_no_joints", False)
                                 for g in groups.values())
        self._waypoints = [groups[p] for p in order]
        self._render_all()
        if self._waypoints:
            self._select_row(0)
        note = " (일부 joint_record 없음 → 0.0)" if had_missing_joints else ""
        self._set_status(
            f"Loaded {len(self._waypoints)} waypoints from "
            f"{os.path.basename(os.path.dirname(path))}{note}.")

    @staticmethod
    def _blank_wp(name: str) -> dict:
        return {
            "name": name,
            "left_deg": [0.0] * 7,
            "right_deg": [0.0] * 7,
            "gripper_m": [0.0, 0.0],
            "left_cart": _identity_cart(),
            "right_cart": _identity_cart(),
            "_no_joints": False,
        }

    def _fill_wp_from_step(self, wp: dict, s: dict,
                           stype: str, arm: str) -> None:
        if arm not in ("left", "right"):
            return
        if stype == "gripper":
            gi = 0 if arm == "left" else 1
            wp["gripper_m"][gi] = float(s.get("position", 0.0) or 0.0)
            return
        # motion step
        tp = s.get("target_pose") or {}
        pos = tp.get("position") or [0.0, 0.0, 0.0]
        quat = tp.get("orientation_xyzw") or [0.0, 0.0, 0.0, 1.0]
        wp[f"{arm}_cart"] = {
            "x": float(pos[0]), "y": float(pos[1]), "z": float(pos[2]),
            "qx": float(quat[0]), "qy": float(quat[1]),
            "qz": float(quat[2]), "qw": float(quat[3]),
        }
        jr = s.get("joint_record") or {}
        deg = jr.get(f"{arm}_deg")
        if isinstance(deg, list) and len(deg) == 7:
            wp[f"{arm}_deg"] = [float(x) for x in deg]
        else:
            wp["_no_joints"] = True

    # ------------------------------------------------------------------
    # Go to Pose (replay one waypoint)
    # ------------------------------------------------------------------

    _goto_confirmed = False

    def _on_goto(self) -> None:
        r = self._current_row()
        if r < 0 or r >= len(self._waypoints):
            self._set_status("Go to Pose: 선택된 웨이포인트 없음.")
            return
        if not self._goto_confirmed:
            ans = QMessageBox.warning(
                self, "Go to Pose",
                "선택한 웨이포인트로 양팔을 이동합니다.\n\n"
                "⚠ 실제 하드웨어면 모터가 구동됩니다. (fake HW면 안전)\n"
                "계속하시겠습니까?",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if ans != QMessageBox.Yes:
                return
            self._goto_confirmed = True

        wp = self._waypoints[r]
        step_type = self._step_type()
        planner = "PILZ_PTP" if step_type == "movej" else "PILZ_LIN"
        sent = 0
        for arm, cart_key in (("left", "left_cart"), ("right", "right_cart")):
            c = wp[cart_key]
            roll, pitch, yaw = quat_to_rpy(c["qx"], c["qy"], c["qz"], c["qw"])
            target = {"x": c["x"], "y": c["y"], "z": c["z"],
                      "roll": roll, "pitch": pitch, "yaw": yaw}
            ok = self._bridge.plan_and_execute_cartesian(
                arm=arm, planner_id=planner, target_pose=target,
                frame_id=f"openarmx_{arm}_link0",
                vel_scale=0.1, acc_scale=0.1, plan_time=5.0, execute=True)
            if ok:
                sent += 1
        self._bridge.send_gripper({
            jd.LEFT_GRIPPER: wp["gripper_m"][0],
            jd.RIGHT_GRIPPER: wp["gripper_m"][1],
        })
        self._set_status(
            f"Go to Pose '{wp['name']}' ({planner}) — {sent}/2 arms dispatched "
            "+ grippers.")

    # ------------------------------------------------------------------

    def _set_status(self, text: str) -> None:
        self._lbl_status.setText(text)

    def shutdown(self) -> None:
        self._live_timer.stop()


# ---------------------------------------------------------------------------
# Module helpers
# ---------------------------------------------------------------------------

def _now() -> float:
    import time
    return time.monotonic()


def _cell(text: str, editable: bool = False) -> QTableWidgetItem:
    item = QTableWidgetItem(text)
    item.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
    if not editable:
        item.setFlags(item.flags() & ~Qt.ItemIsEditable)
    return item


def _pose_to_cart(pose: dict) -> dict:
    return {
        "x": pose["x"], "y": pose["y"], "z": pose["z"],
        "qx": pose["qx"], "qy": pose["qy"],
        "qz": pose["qz"], "qw": pose["qw"],
    }


def _identity_cart() -> dict:
    return {"x": 0.0, "y": 0.0, "z": 0.0,
            "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}
