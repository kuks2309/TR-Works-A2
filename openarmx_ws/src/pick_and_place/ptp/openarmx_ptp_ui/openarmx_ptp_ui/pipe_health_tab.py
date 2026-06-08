"""Pipe Health tab for openarmx_ptp_ui.

Shows each ptp-pipeline topic's publish rate (Hz) / latched / age and a status
dot. Modelled on the scenario UI's DiagnosticsTab (topic view). Rate counting in
the bridge is lazy — started on showEvent, stopped on hideEvent — so the high-
rate camera/cloud/joint streams are only deserialised while this tab is visible.
"""

from __future__ import annotations

from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import (
    QHBoxLayout, QHeaderView, QLabel, QPushButton, QTableWidget,
    QTableWidgetItem, QVBoxLayout, QWidget,
)

from openarmx_ptp_ui import diag_spec as ds

_STATE_COLOR = {"ok": "#0a0", "warn": "#d80", "down": "#c00", "idle": "#888"}


class PipeHealthTab(QWidget):
    def __init__(self, bridge, parent=None) -> None:
        super().__init__(parent)
        self._bridge = bridge
        self._spec = ds.TOPIC_SPECS
        self._build_ui()
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(1000)

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        hdr = QHBoxLayout()
        hdr.addWidget(QLabel(
            "<b>Pipe Health</b> — ptp 파이프라인 토픽 발행률 / latched / age"))
        hdr.addStretch()
        self._btn_refresh = QPushButton("Refresh")
        self._btn_refresh.clicked.connect(self._refresh)
        hdr.addWidget(self._btn_refresh)
        root.addLayout(hdr)

        self._table = QTableWidget(len(self._spec), 5)
        self._table.setHorizontalHeaderLabels(["", "Category", "Topic", "Value", "Detail"])
        self._table.verticalHeader().setVisible(False)
        self._table.verticalHeader().setDefaultSectionSize(26)
        hh = self._table.horizontalHeader()
        hh.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        hh.setSectionResizeMode(1, QHeaderView.ResizeToContents)
        hh.setSectionResizeMode(2, QHeaderView.Stretch)
        hh.setSectionResizeMode(3, QHeaderView.ResizeToContents)
        hh.setSectionResizeMode(4, QHeaderView.Stretch)
        for row, (cat, topic, _mt, _q, _kind, detail) in enumerate(self._spec):
            self._table.setItem(row, 0, QTableWidgetItem("●"))
            self._table.setItem(row, 1, QTableWidgetItem(cat))
            self._table.setItem(row, 2, QTableWidgetItem(topic))
            self._table.setItem(row, 3, QTableWidgetItem(""))
            self._table.setItem(row, 4, QTableWidgetItem(detail))
        root.addWidget(self._table)

    def _refresh(self) -> None:
        rates = self._bridge.diag_consume_rates()
        for row, (_cat, topic, _mt, _q, kind, _detail) in enumerate(self._spec):
            r = rates.get(topic, {"hz": 0.0, "age": None})
            hz, age = r["hz"], r["age"]
            if kind == "rate":
                if hz >= ds.RATE_OK_MIN_HZ:
                    state, val = "ok", f"{hz:.1f} Hz"
                elif hz > 0:
                    state, val = "warn", f"{hz:.1f} Hz"
                else:
                    state, val = "down", "no pub"
            elif kind == "latched":
                state, val = ("ok", "latched") if age is not None else ("idle", "no data")
            else:  # event
                if age is not None and age < 3.0:
                    state, val = "ok", f"{hz:.1f} Hz"
                elif age is not None:
                    state, val = "idle", f"{age:.0f}s ago"
                else:
                    state, val = "idle", "—"
            self._table.item(row, 0).setForeground(QColor(_STATE_COLOR[state]))
            self._table.item(row, 3).setText(val)

    # Lazy rate counting tied to tab visibility (Qt delivers show/hide when the
    # QTabWidget page is selected / left).
    def showEvent(self, event) -> None:
        super().showEvent(event)
        try:
            self._bridge.diag_start()
        except Exception:
            pass

    def hideEvent(self, event) -> None:
        super().hideEvent(event)
        try:
            self._bridge.diag_stop()
        except Exception:
            pass

    def shutdown(self) -> None:
        self._timer.stop()
        try:
            self._bridge.diag_stop()
        except Exception:
            pass
