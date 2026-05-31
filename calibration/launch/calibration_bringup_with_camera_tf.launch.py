"""Calibration bringup + camera static TF.

Wraps calibration_bringup.launch.py and adds a static TF:
    openarmx_body_link0 -> d435_center_link

Translation (m): x=0.034018, y=0.036608, z=0.644715
Rotation  (deg): roll=-1.4041, pitch=31.0059, yaw=-2.1785
(calibrated via solve_extrinsic.py with board at (+0.59, 0, 0) m in base_link,
 board lies flat with z-axis pointing down → --roll 180)

Run:
    ros2 launch /home/openarmx/TR-Works/kkw/China/calibration/launch/calibration_bringup_with_camera_tf.launch.py

Override anything via launch args, e.g.:
    ... bimanual:=true arm_type:=v10
"""

import math
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


CALIBRATION_LAUNCH_DIR = os.path.dirname(os.path.realpath(__file__))
BRINGUP_LAUNCH_FILE = os.path.join(
    CALIBRATION_LAUNCH_DIR, "calibration_bringup.launch.py"
)


def deg2rad(deg: float) -> str:
    return str(deg * math.pi / 180.0)


def generate_launch_description():
    arm_type_arg = DeclareLaunchArgument("arm_type", default_value="v10")
    ee_type_arg = DeclareLaunchArgument("ee_type", default_value="openarmx_hand")
    bimanual_arg = DeclareLaunchArgument("bimanual", default_value="false")
    use_gui_arg = DeclareLaunchArgument("use_gui", default_value="false")
    enable_camera_arg = DeclareLaunchArgument("enable_camera", default_value="true")
    enable_charuco_arg = DeclareLaunchArgument("enable_charuco", default_value="true")
    enable_rviz_arg = DeclareLaunchArgument("enable_rviz", default_value="true")
    rviz_config_arg = DeclareLaunchArgument("rviz_config", default_value="")
    camera_name_arg = DeclareLaunchArgument("camera_name", default_value="d435_center")
    camera_namespace_arg = DeclareLaunchArgument(
        "camera_namespace", default_value="d435_center"
    )

    # openarmx_body_link0 -> d435_center_link (calibrated)
    camera_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="openarmx_body_link0_to_d435_center_link_tf",
        arguments=[
            "--x", "0.034018",
            "--y", "0.036608",
            "--z", "0.644715",
            "--roll",  deg2rad(-1.4041),
            "--pitch", deg2rad(31.0059),
            "--yaw",   deg2rad(-2.1785),
            "--frame-id", "openarmx_body_link0",
            "--child-frame-id", "d435_center_link",
        ],
        output="screen",
    )

    calibration_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(BRINGUP_LAUNCH_FILE),
        launch_arguments={
            "arm_type": LaunchConfiguration("arm_type"),
            "ee_type": LaunchConfiguration("ee_type"),
            "bimanual": LaunchConfiguration("bimanual"),
            "use_gui": LaunchConfiguration("use_gui"),
            "enable_camera": LaunchConfiguration("enable_camera"),
            "enable_charuco": LaunchConfiguration("enable_charuco"),
            "enable_rviz": LaunchConfiguration("enable_rviz"),
            "rviz_config": LaunchConfiguration("rviz_config"),
            "camera_name": LaunchConfiguration("camera_name"),
            "camera_namespace": LaunchConfiguration("camera_namespace"),
        }.items(),
    )

    return LaunchDescription([
        arm_type_arg,
        ee_type_arg,
        bimanual_arg,
        use_gui_arg,
        enable_camera_arg,
        enable_charuco_arg,
        enable_rviz_arg,
        rviz_config_arg,
        camera_name_arg,
        camera_namespace_arg,
        camera_static_tf,
        calibration_bringup,
    ])
