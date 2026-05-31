#!/usr/bin/env python3
"""Entry point for openarmx_scenario_ui box-style UI."""

import signal
import sys

from PyQt5.QtWidgets import QApplication

from openarmx_scenario_ui.scenario_box_window import ScenarioBoxWindow


def main() -> int:
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app = QApplication(sys.argv)
    app.setApplicationName("OpenArmX Scenario Box UI")
    win = ScenarioBoxWindow()
    win.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
