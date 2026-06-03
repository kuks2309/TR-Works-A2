"""Diagnostics tab — health of the openarmx nodes/topics the GUI relies on.

Node rows show alive/down; topic rows show publish rate (Hz) / latched / no
data. Auto-refreshes and on the "Refresh Now" button. Reads everything from the
shared ScenarioRosBridge (no second rclpy context).
"""

from __future__ import annotations

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QBrush, QColor
from PyQt5.QtWidgets import (
    QAbstractItemView, QHBoxLayout, QHeaderView, QLabel, QPushButton,
    QTableWidget, QTableWidgetItem, QTextEdit, QVBoxLayout, QWidget,
)

from openarmx_scenario_ui import diagnostics_spec as ds

# state -> (dot color, label color)
_COLORS = {
    "ok":   QColor("#2e7d32"),
    "warn": QColor("#ef6c00"),
    "down": QColor("#c62828"),
    "idle": QColor("#9e9e9e"),
}


class DiagnosticsTab(QWidget):
    def __init__(self, bridge, parent=None, show="both") -> None:
        super().__init__(parent)
        self._bridge = bridge
        self._show = show          # "both" | "node" (Node Health) | "topic" (Pipe Health)
        # Static row spec: (kind, category, link, detail, extra)
        #   node:  extra = optional(bool)
        #   topic: extra = (qos_kind, kind)
        self._spec = []
        if show in ("both", "node"):
            for cat, name, detail in ds.NODE_SPECS:
                self._spec.append(("node", cat, name, detail, name in ds.OPTIONAL_NODES))
        if show in ("both", "topic"):
            for cat, topic, _mt, _q, kind, detail in ds.TOPIC_SPECS:
                self._spec.append(("topic", cat, topic, detail, kind))

        self._build_ui()
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(1500)
        self._refresh()

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)

        self._lbl_summary = QLabel("…")
        self._lbl_summary.setAlignment(Qt.AlignCenter)
        self._lbl_summary.setStyleSheet(
            "background:#2b3a42; color:white; padding:5px; font-weight:bold;")
        root.addWidget(self._lbl_summary)

        self._table = QTableWidget(len(self._spec), 5)
        self._table.setHorizontalHeaderLabels(
            ["", "Category", "Link", "Value", "Detail"])
        self._table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self._table.setSelectionMode(QAbstractItemView.NoSelection)
        self._table.verticalHeader().setDefaultSectionSize(26)
        hh = self._table.horizontalHeader()
        hh.setSectionResizeMode(0, QHeaderView.Fixed)
        self._table.setColumnWidth(0, 28)
        hh.setSectionResizeMode(1, QHeaderView.ResizeToContents)
        hh.setSectionResizeMode(2, QHeaderView.ResizeToContents)
        hh.setSectionResizeMode(3, QHeaderView.ResizeToContents)
        hh.setSectionResizeMode(4, QHeaderView.Stretch)

        for row, (_kind, cat, link, detail, _extra) in enumerate(self._spec):
            dot = QTableWidgetItem("●")
            dot.setTextAlignment(Qt.AlignCenter)
            self._table.setItem(row, 0, dot)
            self._table.setItem(row, 1, QTableWidgetItem(cat))
            self._table.setItem(row, 2, QTableWidgetItem(link))
            self._table.setItem(row, 3, QTableWidgetItem(""))
            self._table.setItem(row, 4, QTableWidgetItem(detail))
        # Spec table gets equal vertical weight with the raw node list below
        # (Node Health = 50/50 top/bottom). For Pipe Health it simply fills.
        root.addWidget(self._table, stretch=1)

        # Node Health also shows the raw `ros2 node list` (moved here from the
        # Launch Manager tab) so ALL live nodes are visible, not just spec'd ones.
        self._node_view = None
        if self._show == "node":
            root.addWidget(QLabel("All ROS2 nodes:"))
            self._node_view = QTextEdit()
            self._node_view.setReadOnly(True)
            self._node_view.setMinimumHeight(120)
            root.addWidget(self._node_view, stretch=1)   # 50/50 with the table above

        bottom = QHBoxLayout()
        self._lbl_footer = QLabel("")
        self._btn_refresh = QPushButton("Refresh Now")
        self._btn_refresh.clicked.connect(self._refresh)
        bottom.addWidget(self._lbl_footer)
        bottom.addStretch()
        bottom.addWidget(self._btn_refresh)
        root.addLayout(bottom)

    # ------------------------------------------------------------------

    def _refresh(self) -> None:
        live = self._bridge.live_node_names()
        # diag_consume_rates() RESETS the shared per-topic message counters on
        # read, so it must have exactly ONE consumer. Node Health (show="node")
        # does not display rates — if it also consumed, it would zero the counts
        # between Pipe Health's reads and every streaming topic (e.g. /joint_states,
        # /tf) would show a false 0.0 Hz. Only the topic-showing tab consumes.
        rates = (self._bridge.diag_consume_rates()
                 if self._show in ("both", "topic") else {})
        counts = {"ok": 0, "warn": 0, "down": 0, "idle": 0}
        nodes_alive = nodes_total = 0

        for row, (kind, _cat, link, _detail, extra) in enumerate(self._spec):
            if kind == "node":
                nodes_total += 1
                optional = extra
                if link in live:
                    state, value = "ok", "alive"
                    nodes_alive += 1
                elif optional:
                    state, value = "idle", "off"
                else:
                    state, value = "down", "down"
            else:  # topic
                topic_kind = extra
                r = rates.get(link, {"hz": 0.0, "age": None})
                hz, age = r["hz"], r["age"]
                pub = self._bridge.topic_pub_count(link)
                if topic_kind == "rate":
                    if hz >= ds.RATE_OK_MIN_HZ:
                        state, value = "ok", f"{hz:.1f} Hz"
                    elif pub > 0:
                        state, value = "warn", f"{hz:.1f} Hz"
                    else:
                        state, value = "down", "no pub"
                elif topic_kind == "latched":
                    if age is not None:
                        state, value = "ok", "latched"
                    elif pub > 0:
                        state, value = "idle", "no data"
                    else:
                        state, value = "down", "no pub"
                else:  # event
                    if age is not None and age < 3.0:
                        state, value = "ok", f"{hz:.1f} Hz"
                    elif pub > 0:
                        state, value = "idle", "idle"
                    else:
                        state, value = "down", "no pub"

            counts[state] += 1
            color = _COLORS[state]
            self._table.item(row, 0).setForeground(QBrush(color))
            vitem = self._table.item(row, 3)
            vitem.setText(value)
            vitem.setForeground(QBrush(color))

        self._lbl_summary.setText(
            f"✗ {counts['down']} down   ⚠ {counts['warn']} warn   "
            f"✓ {counts['ok']} ok   ○ {counts['idle']} idle")
        self._lbl_footer.setText(
            f"Nodes: {nodes_alive}/{nodes_total}   Links: {len(rates)}")

        if self._node_view is not None:
            fq = sorted(n for n in self._bridge.live_node_names() if n.startswith("/"))
            self._node_view.setPlainText("\n".join(fq) if fq else "(no nodes)")

    def shutdown(self) -> None:
        self._timer.stop()
