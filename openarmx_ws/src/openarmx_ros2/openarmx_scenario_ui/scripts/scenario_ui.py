#!/usr/bin/env python3
"""Entry point for openarmx_scenario_ui — launches the PyQt5 main window."""

import argparse
import signal
import sys

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication

from openarmx_scenario_ui.main_window import ScenarioMainWindow


def main() -> int:
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

    # SIGINT(터미널 Ctrl+C / ros2 launch 종료) 시 즉시 종료(SIG_DFL) 대신 graceful
    # 종료 → closeEvent 가 실행되어 카메라/검출/백엔드 등 자식 프로세스를 정리한다.
    # (SIG_DFL 이면 closeEvent 미실행 → /camera/camera, d435_center_to_camera_link_tf 잔존)
    def _on_sigint(*_a):
        win.close()
        app.quit()
    signal.signal(signal.SIGINT, _on_sigint)
    # Qt 이벤트 루프가 Python 시그널을 처리할 틈을 주도록 주기적으로 깨운다.
    _sigint_timer = QTimer()
    _sigint_timer.timeout.connect(lambda: None)
    _sigint_timer.start(200)

    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
