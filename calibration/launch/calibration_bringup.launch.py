"""Calibration bringup launch.

Spawns in one shot:
  - openarmx robot_state_publisher (URDF -> /robot_description, /tf)
  - joint_state_publisher (publishes /joint_states; GUI optional)
  - RealSense D435 (color, depth, aligned depth, pointcloud)
  - live_detect.py (ChArUco detector publishing /charuco/image_debug + TF)
  - rviz2 with display config

Run (workspace must be sourced):
    ros2 launch /home/openarmx/TR-Works/kkw/China/calibration/launch/calibration_bringup.launch.py

Override anything via launch args, e.g.:
    ... arm_type:=v10 use_gui:=true enable_camera:=false
"""

import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


CALIBRATION_DIR = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
LIVE_DETECT_SCRIPT = os.path.join(CALIBRATION_DIR, "scripts", "live_detect.py")


def robot_state_publisher_spawner(context: LaunchContext, arm_type, ee_type, bimanual):
    arm_type_str = context.perform_substitution(arm_type)
    ee_type_str = context.perform_substitution(ee_type)
    bimanual_str = context.perform_substitution(bimanual)

    xacro_path = os.path.join(
        get_package_share_directory("openarmx_description"),
        "urdf", "robot", f"{arm_type_str}.urdf.xacro",
    )

    robot_description = xacro.process_file(
        xacro_path,
        mappings={
            "arm_type": arm_type_str,
            "ee_type": ee_type_str,
            "bimanual": bimanual_str,
        },
    ).toprettyxml(indent="  ")

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        )
    ]


def rviz_spawner(context: LaunchContext, bimanual, rviz_config, enable_rviz):
    enable_rviz_str = context.perform_substitution(enable_rviz)
    if enable_rviz_str.lower() != "true":
        return []

    rviz_config_str = context.perform_substitution(rviz_config)
    if not rviz_config_str:
        bimanual_str = context.perform_substitution(bimanual)
        rviz_file = "bimanual.rviz" if bimanual_str.lower() == "true" else "arm_only.rviz"
        rviz_config_str = os.path.join(
            get_package_share_directory("openarmx_description"),
            "rviz", rviz_file,
        )

    return [
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["--display-config", rviz_config_str],
            output="screen",
        )
    ]


def generate_launch_description():
    arm_type_arg = DeclareLaunchArgument(
        "arm_type", default_value="v10",
        description="Arm type matching urdf/robot/<arm_type>.urdf.xacro (e.g., v10)",
    )
    ee_type_arg = DeclareLaunchArgument(
        "ee_type", default_value="openarmx_hand",
        description="End-effector type (e.g., openarmx_hand or none)",
    )
    bimanual_arg = DeclareLaunchArgument(
        "bimanual", default_value="false",
        description="Use bimanual configuration",
    )
    use_gui_arg = DeclareLaunchArgument(
        "use_gui", default_value="false",
        description="Use joint_state_publisher_gui instead of headless joint_state_publisher",
    )
    enable_camera_arg = DeclareLaunchArgument(
        "enable_camera", default_value="true",
        description="Launch RealSense D435 driver",
    )
    enable_charuco_arg = DeclareLaunchArgument(
        "enable_charuco", default_value="true",
        description="Run live_detect.py ChArUco detector",
    )
    enable_rviz_arg = DeclareLaunchArgument(
        "enable_rviz", default_value="true",
        description="Launch rviz2",
    )
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config", default_value="",
        description="Path to RViz display config (default: bimanual.rviz if bimanual:=true else arm_only.rviz)",
    )
    camera_name_arg = DeclareLaunchArgument(
        "camera_name", default_value="d435_center",
        description="RealSense camera node name",
    )
    camera_namespace_arg = DeclareLaunchArgument(
        "camera_namespace", default_value="d435_center",
        description="RealSense camera namespace",
    )

    arm_type = LaunchConfiguration("arm_type")
    ee_type = LaunchConfiguration("ee_type")
    bimanual = LaunchConfiguration("bimanual")
    use_gui = LaunchConfiguration("use_gui")
    enable_camera = LaunchConfiguration("enable_camera")
    enable_charuco = LaunchConfiguration("enable_charuco")
    enable_rviz = LaunchConfiguration("enable_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    camera_name = LaunchConfiguration("camera_name")
    camera_namespace = LaunchConfiguration("camera_namespace")

    robot_state_publisher_loader = OpaqueFunction(
        function=robot_state_publisher_spawner,
        args=[arm_type, ee_type, bimanual],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        condition=UnlessCondition(use_gui),
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=IfCondition(use_gui),
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("realsense2_camera"),
                "launch", "rs_launch.py",
            )
        ),
        launch_arguments={
            "enable_color": "true",
            "enable_depth": "true",
            "align_depth.enable": "true",
            "pointcloud.enable": "true",
            "camera_name": camera_name,
            "camera_namespace": camera_namespace,
        }.items(),
        condition=IfCondition(enable_camera),
    )

    charuco_proc = ExecuteProcess(
        cmd=["python3", LIVE_DETECT_SCRIPT],
        output="screen",
        condition=IfCondition(enable_charuco),
    )

    rviz_loader = OpaqueFunction(
        function=rviz_spawner,
        args=[bimanual, rviz_config, enable_rviz],
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
        robot_state_publisher_loader,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        realsense_launch,
        charuco_proc,
        rviz_loader,
    ])
