"""Launch the openarmx ptp pick-and-place UI."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="openarmx_ptp_ui",
            executable="ptp_pnp_ui.py",
            name="openarmx_ptp_ui",
            output="screen",
        ),
    ])
