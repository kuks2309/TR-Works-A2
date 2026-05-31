"""Live plane detection: launches two D435 cameras (center + upper),
the live_plane_detector node, and RViz.

The serial numbers below match HOWTO.md (TR-Works camera mapping).
Override with launch args if your hardware differs.

Usage:
    ros2 launch box_detection live.launch.py
    ros2 launch box_detection live.launch.py rviz:=false
    ros2 launch box_detection live.launch.py center_serial:=_xxx upper_serial:=_yyy
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("box_detection")
    rviz_config = os.path.join(pkg_share, "config", "box_analysis.rviz")
    rs_share = get_package_share_directory("realsense2_camera")
    rs_launch = os.path.join(rs_share, "launch", "rs_launch.py")

    args = [
        DeclareLaunchArgument("center_serial", default_value="_818312070932"),
        DeclareLaunchArgument("upper_serial",  default_value="_819612070814"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("refresh_rate", default_value="1.0",
                              description="plane detector update rate (Hz)"),
    ]

    common_cam_args = {
        "pointcloud.enable": "true",
        "align_depth.enable": "true",
        "enable_color": "true",
        "enable_depth": "true",
    }

    cam_center = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rs_launch),
        launch_arguments={
            "serial_no": LaunchConfiguration("center_serial"),
            "camera_name": "d435_center",
            "camera_namespace": "d435_center",
            **common_cam_args,
        }.items(),
    )
    cam_upper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rs_launch),
        launch_arguments={
            "serial_no": LaunchConfiguration("upper_serial"),
            "camera_name": "d435_center_upper",
            "camera_namespace": "d435_center_upper",
            **common_cam_args,
        }.items(),
    )

    detector = Node(
        package="box_detection",
        executable="live_plane_detector",
        name="live_plane_detector",
        output="screen",
        parameters=[{"refresh_rate": LaunchConfiguration("refresh_rate")}],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription([
        # See HOWTO.md §4.1 — FastRTPS profile env var can break communication.
        SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE", ""),
        *args,
        cam_center, cam_upper, detector, rviz,
    ])
