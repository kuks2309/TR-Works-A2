#!/usr/bin/env python3
"""Entry point for openarmx_scenario_ui — launches the PyQt5 main window."""

import argparse
import signal
import sys

from PyQt5.QtWidgets import QApplication

from openarmx_scenario_ui.main_window import ScenarioMainWindow


def main() -> int:
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    parser = argparse.ArgumentParser(description="OpenArmX Scenario UI")
    parser.add_argument(
        "--no-auto", action="store_true",
        help="Disable auto-start of hardware + scenario player on launch",
    )
    parser.add_argument(
        "--no-rviz", action="store_true",
        help="Do not auto-spawn the openarmx_scenario.rviz RViz instance "
             "(use when an external RViz/launch already owns the display)",
    )
    parser.add_argument(
        "--follower", choices=["cyclo", "moveit"], default="moveit",
        help="Marker → arm follower path. 'moveit' (default) spawns "
             "only RViz + markers; the user runs SIL Bringup + MoveIt "
             "Demo separately and uses the UI Cartesian tab toggle to "
             "trigger plan&execute per drag-release. 'cyclo' spawns "
             "openarmx_motion cyclo_sim + vr_controller_node (QP+CBF "
             "follower that auto-tracks the marker) — requires "
             "cyclo_motion_controller_ros to be built in the workspace.",
    )
    args, qt_args = parser.parse_known_args()

    app = QApplication(qt_args)
    app.setApplicationName("OpenArmX Scenario UI")
    win = ScenarioMainWindow(auto_start=not args.no_auto,
                             with_rviz=not args.no_rviz,
                             follower=args.follower)
    win.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
