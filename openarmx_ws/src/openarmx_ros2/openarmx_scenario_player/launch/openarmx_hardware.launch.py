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
import subprocess

import xacro
from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _ensure_can_up(interfaces):
    """HIL 에서 ros2_control_node 가 뜨기 전에 follower CAN 버스를 보장한다.

    CAN 이 DOWN 이면 v10_simple_hardware 의 소켓 초기화가 std::runtime_error 를
    던져 ros2_control_node 가 SIGABRT → respawn 루프 → /joint_states 미발행 →
    TF 깨짐(RViz 로봇 망가짐)으로 이어진다. openarmx_hardware.launch.py 자체는
    원래 CAN 을 올리지 않았으므로(이름만 파라미터 전달), 여기서 멱등 기동한다.

    실제 기동 로직은 scripts/up_follower_can.sh 한 곳에만 둔다(단일 근원).
    이미 UP 인 인터페이스는 스크립트가 건너뛰므로 반복 호출에 안전하다.
    """
    script = os.path.join(
        get_package_prefix("openarmx_scenario_player"),
        "lib", "openarmx_scenario_player", "up_follower_can.sh")
    if not os.path.exists(script):
        print(f"[openarmx_hardware] CAN 기동 스크립트 없음: {script} "
              "(colcon build 필요?) — 자동 기동 건너뜀")
        return
    try:
        subprocess.run([script, *interfaces], check=False, timeout=30)
    except Exception as exc:  # noqa: BLE001 - launch 는 계속 진행
        print(f"[openarmx_hardware] CAN 자동 기동 실패: {exc}")


def _spawn_hardware(context, *_args, **_kwargs):
    use_fake = LaunchConfiguration("use_fake_hardware").perform(context)
    control_mode = LaunchConfiguration("control_mode").perform(context)
    right_can = LaunchConfiguration("right_can_interface").perform(context)
    left_can = LaunchConfiguration("left_can_interface").perform(context)
    can_fd = LaunchConfiguration("can_fd").perform(context)

    # HIL(실기) 일 때만 CAN 자동 기동. SIL(fake) 은 CAN 불필요.
    if use_fake.lower() == "false":
        _ensure_can_up([right_can, left_can])

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
