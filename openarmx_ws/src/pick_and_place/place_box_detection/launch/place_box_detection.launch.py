"""Launch the place-box detection pipeline node (and optionally the TOF driver).

The D435 camera + TF (openarmx_body_link0 <- camera) are assumed to be already
running (brought up elsewhere, e.g. the scenario player / d435_camera launch).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("place_box_detection")
    params = os.path.join(pkg, "config", "place_box_detection.yaml")

    use_tof = LaunchConfiguration("use_tof_driver")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_tof_driver", default_value="false",
            description="Also start the TOF USB-serial driver (/dev/ttyACM0)."),

        Node(
            package="place_box_detection",
            executable="place_box_detection_node",
            name="place_box_detection_node",
            output="screen",
            parameters=[params],
        ),

        Node(
            package="place_box_detection",
            executable="tof_serial_driver_node",
            name="tof_serial_driver",
            output="screen",
            parameters=[params],
            condition=IfCondition(use_tof),
        ),
    ])
