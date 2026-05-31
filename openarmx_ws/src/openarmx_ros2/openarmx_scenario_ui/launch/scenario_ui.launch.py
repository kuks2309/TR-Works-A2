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
        ),
    ])
