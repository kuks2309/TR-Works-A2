"""Calibration bringup with the calibrated D435 camera shown in RViz.

Wraps calibration_bringup.launch.py and points RViz at a dedicated config that
displays the robot, the camera body mesh, and TF axes.

The camera (frame d435_center_link + d435.dae mesh) is now baked into the robot
description itself — see openarmx_description/urdf/robot/v10.urdf.xacro, guarded
by `bimanual` since the calibration parent frame openarmx_body_link0 only exists
in the bimanual urdf. So robot_state_publisher publishes the calibrated transform
    openarmx_body_link0 -> d435_center_link
    t=(0.034018, 0.036608, 0.644715) m   rpy=(-1.4041, 31.0059, -2.1785) deg
and renders the mesh; no separate static_transform_publisher is needed here.

Because of that, this launch defaults to bimanual:=true — single-arm v10 has no
body_link0 and therefore no camera.

Run:
    ros2 launch /home/openarmx/TR-Works/kkw/China/calibration/launch/calibration_bringup_with_camera_tf.launch.py

Show only the models (no hardware needed):
    ... enable_camera:=false enable_charuco:=false
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


CALIBRATION_LAUNCH_DIR = os.path.dirname(os.path.realpath(__file__))
CALIBRATION_DIR = os.path.dirname(CALIBRATION_LAUNCH_DIR)
BRINGUP_LAUNCH_FILE = os.path.join(
    CALIBRATION_LAUNCH_DIR, "calibration_bringup.launch.py"
)
CAMERA_RVIZ_CONFIG = os.path.join(
    CALIBRATION_DIR, "rviz", "calibration_camera.rviz"
)


def generate_launch_description():
    arm_type_arg = DeclareLaunchArgument("arm_type", default_value="v10")
    ee_type_arg = DeclareLaunchArgument("ee_type", default_value="openarmx_hand")
    # body_link0 (camera parent frame) only exists in the bimanual urdf.
    bimanual_arg = DeclareLaunchArgument("bimanual", default_value="true")
    use_gui_arg = DeclareLaunchArgument("use_gui", default_value="false")
    enable_camera_arg = DeclareLaunchArgument("enable_camera", default_value="true")
    enable_charuco_arg = DeclareLaunchArgument("enable_charuco", default_value="true")
    enable_rviz_arg = DeclareLaunchArgument("enable_rviz", default_value="true")
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config", default_value=CAMERA_RVIZ_CONFIG
    )
    camera_name_arg = DeclareLaunchArgument("camera_name", default_value="d435_center")
    camera_namespace_arg = DeclareLaunchArgument(
        "camera_namespace", default_value="d435_center"
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
        calibration_bringup,
    ])
