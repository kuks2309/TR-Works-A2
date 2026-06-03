#!/usr/bin/env python3
"""Hardware-only bringup (L0) — controller_manager + hardware interface +
robot_state_publisher, WITHOUT spawning any controllers.

Purpose: the scenario/teaching stack wants the control layers as separate
start/stop units (L0 hardware   vs  L1 controllers/JTC group). The vendor
`openarmx_bringup/openarmx.bimanual.launch.py` always spawns the controllers
together with the hardware and is a fork (modifying it cascades), so this is a
NEW, self-contained launch that brings up ONLY the hardware layer. The L1
controllers are spawned separately (e.g. via
`ros2 run controller_manager spawner <names> -c /controller_manager`, or the
Launch Manager 'Controllers (JTC group)' target).

Reuses the EXACT xacro mapping + controllers YAML that the vendor bringup uses
(openarmx_description/urdf/robot/v10.urdf.xacro,
 openarmx_bringup/config/v10_controllers/openarmx_v10_bimanual_controllers.yaml)
so the resulting controller_manager is identical to the vendor one — only the
controller spawners are omitted.

  SIL (Software In the Loop):  use_fake_hardware:=true   (default)
  HIL (Hardware In the Loop):  use_fake_hardware:=false control_mode:=mit \
                               right_can_interface:=can0 left_can_interface:=can1
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _spawn_hardware(context, *_args, **_kwargs):
    use_fake = LaunchConfiguration("use_fake_hardware").perform(context)
    control_mode = LaunchConfiguration("control_mode").perform(context)
    right_can = LaunchConfiguration("right_can_interface").perform(context)
    left_can = LaunchConfiguration("left_can_interface").perform(context)
    can_fd = LaunchConfiguration("can_fd").perform(context)

    xacro_path = os.path.join(
        get_package_share_directory("openarmx_description"),
        "urdf", "robot", "v10.urdf.xacro")
    robot_description = xacro.process_file(
        xacro_path,
        mappings={
            "arm_type": "v10",
            "bimanual": "true",
            "use_fake_hardware": use_fake,
            "ros2_control": "true",
            "can_fd": can_fd,
            "right_can_interface": right_can,
            "left_can_interface": left_can,
            "control_mode": control_mode,
            "node_namespace": "",
        },
    ).toprettyxml(indent="  ")
    rd = {"robot_description": robot_description}

    controllers_yaml = os.path.join(
        get_package_share_directory("openarmx_bringup"),
        "config", "v10_controllers", "openarmx_v10_bimanual_controllers.yaml")

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[rd],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="both",
            parameters=[rd, controllers_yaml],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_fake_hardware", default_value="true",
            description="true = SIL(fake hardware), false = HIL(real CAN motors)."),
        DeclareLaunchArgument("control_mode", default_value="mit"),
        DeclareLaunchArgument("right_can_interface", default_value="can0"),
        DeclareLaunchArgument("left_can_interface", default_value="can1"),
        DeclareLaunchArgument("can_fd", default_value="false"),
        OpaqueFunction(function=_spawn_hardware),
    ])
