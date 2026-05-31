#!/usr/bin/env python3
"""Entry point for tr_works_scenario_ui — launches the PyQt5 main window."""

import argparse
import signal
import sys

from PyQt5.QtWidgets import QApplication

from tr_works_scenario_ui.main_window import ScenarioMainWindow


def main() -> int:
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    parser = argparse.ArgumentParser(description="TR-Works Scenario UI")
    parser.add_argument(
        "--no-auto", action="store_true",
        help="Disable auto-start of hardware + scenario player on launch",
    )
    args, qt_args = parser.parse_known_args()

    app = QApplication(qt_args)
    app.setApplicationName("TR-Works Scenario UI")
    win = ScenarioMainWindow(auto_start=not args.no_auto)
    win.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
