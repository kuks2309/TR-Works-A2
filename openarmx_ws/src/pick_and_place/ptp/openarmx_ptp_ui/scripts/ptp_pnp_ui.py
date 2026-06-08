#!/usr/bin/env python3
"""Entry point for openarmx_ptp_ui — launches the ptp pick-and-place PyQt5 window."""

import argparse
import signal
import sys

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication

from openarmx_ptp_ui.main_window import PtpPnpMainWindow


def main() -> int:
    parser = argparse.ArgumentParser(description="OpenArmX ptp Pick-and-Place UI")
    parser.add_argument(
        "--no-rviz", action="store_true",
        help="Do not auto-spawn the openarmx_scenario.rviz RViz instance "
             "(use when an external RViz already owns the display).",
    )
    args, qt_args = parser.parse_known_args()

    app = QApplication(qt_args)
    app.setApplicationName("OpenArmX ptp Pick-and-Place UI")
    win = PtpPnpMainWindow(with_rviz=not args.no_rviz)
    win.show()

    # SIGINT (Ctrl+C / ros2 launch shutdown) -> graceful close so closeEvent
    # runs and the launches this UI started are torn down (no orphan nodes).
    def _on_sigint(*_a):
        win.close()
        app.quit()
    signal.signal(signal.SIGINT, _on_sigint)
    # Let the Qt loop wake up periodically to service the Python signal.
    _sigint_timer = QTimer()
    _sigint_timer.timeout.connect(lambda: None)
    _sigint_timer.start(200)

    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
