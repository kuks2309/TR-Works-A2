"""Launch the place-box detection pipeline node (and optionally the TOF driver).

The D435 camera + TF (openarmx_body_link0 <- camera) are assumed to be already
running (brought up elsewhere, e.g. the scenario player / d435_camera launch).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("place_box_detection")
    params = os.path.join(pkg, "config", "place_box_detection.yaml")

    use_tof = LaunchConfiguration("use_tof_driver")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_tof_driver", default_value="false",
            description="Also start the TOF driver (openarmx_tof_driver, /dev/ttyACM0)."),

        Node(
            package="place_box_detection",
            executable="place_box_detection_node",
            name="place_box_detection_node",
            output="screen",
            parameters=[params],
        ),

        # The TOF transport driver now lives in the hardware-layer package
        # openarmx_tof_driver; optionally bring it up here for convenience
        # (default off, since it is usually launched on its own).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("openarmx_tof_driver"),
                "launch", "tof_driver.launch.py")),
            condition=IfCondition(use_tof),
        ),
    ])
