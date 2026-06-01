#!/usr/bin/env python3
"""Headless mock-hardware sim for OpenArmX -- MoveIt-free, RViz-free.

Brings up exactly the hardware-abstraction layer that the cyclo_control
QP+CBF stack needs:
  * robot_state_publisher  (URDF from xacro v10 bimanual)
  * ros2_control_node      (mock_components -- no real CAN/motor needed)
  * joint_state_broadcaster
  * {left,right}_joint_trajectory_controller  (vr_controller writes here)
  * {left,right}_gripper_controller

NO MoveIt move_group, NO Pilz, NO RViz. Those belong to other launches
(demo_sim for MoveIt+Pilz, openarmx_drag_follow_full for cyclo+RViz).
Keep this sim layer pure so cyclo's vr_controller can drive the JTCs
without competing with MoveIt's MoveGroup planner publishing to the same
trajectory topics.

Usage (typically composed into a higher-level launch via
IncludeLaunchDescription):
    ros2 launch openarmx_motion openarmx_cyclo_sim.launch.py
"""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _spawn_robot_nodes(
    context: LaunchContext,
    description_package, description_file, arm_type,
    can_fd, control_mode, right_can_interface, left_can_interface,
    controllers_file,
):
    """OpaqueFunction: expand xacro and spawn RSP + ros2_control_node."""
    dp = context.perform_substitution(description_package)
    df = context.perform_substitution(description_file)
    xacro_path = os.path.join(
        get_package_share_directory(dp), "urdf", "robot", df)

    robot_description = xacro.process_file(
        xacro_path,
        mappings={
            "arm_type": context.perform_substitution(arm_type),
            "bimanual": "true",
            "use_fake_hardware": "true",   # MOCK -- this is the whole point
            "ros2_control": "true",
            "can_fd": context.perform_substitution(can_fd),
            "control_mode": context.perform_substitution(control_mode),
            "left_can_interface": context.perform_substitution(left_can_interface),
            "right_can_interface": context.perform_substitution(right_can_interface),
        },
    ).toprettyxml(indent="  ")

    robot_description_param = {"robot_description": robot_description}
    controllers_file_str = context.perform_substitution(controllers_file)

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[robot_description_param],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="both",
            parameters=[robot_description_param, controllers_file_str],
        ),
    ]


def generate_launch_description():
    description_pkg = "openarmx_description"
    bringup_pkg = FindPackageShare("openarmx_bringup")

    args = [
        DeclareLaunchArgument("description_package", default_value=description_pkg),
        DeclareLaunchArgument("description_file", default_value="v10.urdf.xacro"),
        DeclareLaunchArgument("arm_type", default_value="v10"),
        DeclareLaunchArgument("can_fd", default_value="false"),
        DeclareLaunchArgument("control_mode", default_value="mit",
                              choices=["mit", "csp"]),
        DeclareLaunchArgument("right_can_interface", default_value="can0"),
        DeclareLaunchArgument("left_can_interface", default_value="can1"),
        DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution(
                [bringup_pkg, "config", "v10_controllers",
                 "openarmx_v10_bimanual_controllers.yaml"]),
            description=("ros2_control YAML. Reused from openarmx_bringup so "
                         "JTC names match demo_sim conventions "
                         "(left/right_joint_trajectory_controller).")),
    ]

    spawner_args = [
        LaunchConfiguration("description_package"),
        LaunchConfiguration("description_file"),
        LaunchConfiguration("arm_type"),
        LaunchConfiguration("can_fd"),
        LaunchConfiguration("control_mode"),
        LaunchConfiguration("right_can_interface"),
        LaunchConfiguration("left_can_interface"),
        LaunchConfiguration("controllers_file"),
    ]

    robot_nodes = OpaqueFunction(function=_spawn_robot_nodes, args=spawner_args)

    # JointStateBroadcaster + arm JTC + gripper -- delayed so
    # ros2_control_node finishes loading the hardware interface first.
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )
    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_joint_trajectory_controller",
                   "right_joint_trajectory_controller",
                   "-c", "/controller_manager"],
    )
    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_gripper_controller", "right_gripper_controller",
                   "-c", "/controller_manager"],
    )

    return LaunchDescription([
        *args,
        robot_nodes,
        TimerAction(period=2.0, actions=[jsb_spawner]),
        TimerAction(period=3.0, actions=[arm_spawner]),
        TimerAction(period=3.5, actions=[gripper_spawner]),
    ])
