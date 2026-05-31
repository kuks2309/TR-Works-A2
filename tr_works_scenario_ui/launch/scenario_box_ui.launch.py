"""Launch the box-style scenario UI."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="tr_works_scenario_ui",
            executable="scenario_box_ui.py",
            name="tr_works_scenario_box_ui",
            output="screen",
        ),
    ])
