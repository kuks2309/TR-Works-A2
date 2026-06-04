"""Launch the openarmx scenario UI."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="openarmx_scenario_ui",
            executable="scenario_ui.py",
            name="openarmx_scenario_ui",
            output="screen",
            # --no-auto: do NOT auto-start Hardware Bringup / Scenario Player on
            # launch. UI + RViz only; the user presses Start manually.
            arguments=["--no-auto"],
        ),
    ])
