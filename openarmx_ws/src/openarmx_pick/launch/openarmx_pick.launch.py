#!/usr/bin/env python3
"""Full Stage-B pick pipeline: vision -> grasp synthesis -> QP MoveL solver.

Brings up:
  1. the cyclo_control QP+CBF MoveL solver bound to the OpenArmX left arm
     (via ``openarmx_movel.launch.py``), and
  2. ``grasp_pose_node`` which turns the existing ``/box_plane`` box-top cloud
     into a top-down grasp PoseStamped (and, if ``auto_send:=true``, a pre-grasp
     MoveL command).

It does NOT launch the camera / YOLO / box_plane stack -- run that separately
(``run_yolov8_ros.sh`` in 3d_detect_ws). This launch only consumes its output.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("openarmx_pick")

    args = [
        DeclareLaunchArgument("auto_send", default_value="false",
                              description="grasp_pose_node also emits a pre-grasp MoveL."),
        DeclareLaunchArgument("pregrasp_height", default_value="0.10"),
        DeclareLaunchArgument("cloud_topic", default_value="/box_plane/cloud"),
        DeclareLaunchArgument("start_solver", default_value="true",
                              description="Also launch the MoveL solver."),
    ]

    solver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, "launch", "openarmx_movel.launch.py"])),
        condition=IfCondition(LaunchConfiguration("start_solver")),
    )

    grasp = Node(
        package="openarmx_pick",
        executable="grasp_pose_node",
        name="grasp_pose_node",
        output="screen",
        parameters=[{
            "cloud_topic": LaunchConfiguration("cloud_topic"),
            "info_topic": "/box_plane/info",
            "base_frame": "openarmx_body_link0",
            "movel_topic": "/openarmx/movel",
            "auto_send": LaunchConfiguration("auto_send"),
            "pregrasp_height": LaunchConfiguration("pregrasp_height"),
            "grasp_depth": 0.005,
            "cloud_stride": 4,
        }],
    )

    return LaunchDescription([*args, solver, grasp])
