#!/usr/bin/env python3
"""Bimanual gravity-compensation feedforward — toggleable from the Launch Manager.

Spawns the forward_effort_controller(s) (which the spawner activates then exits,
leaving the controllers loaded) and runs ``gravity_comp_node`` which computes
per-arm g(q) from the robot model and publishes it to
``/<side>_forward_effort_controller/commands``. The hardware write() adds that as
the MIT-mode feedforward torque → removes the no-integral droop.

NOTE: the spawner intentionally does NOT use ``--unload-on-kill``. That flag keeps
the spawner alive to unload the controllers when killed, but it means ANY spawner
death (CM-not-ready timeout, partial kill) silently unloads the effort controllers
while the node keeps publishing → comp dies with no symptom (2026-06-07 incident).
Clean stop on checkbox-off is instead guaranteed by the OFF handler setting
``enable_compensation=false`` (node publishes zeros → controllers output 0) before
stopping; the controllers persist (harmless, output 0 when off).

Prerequisite: L1 controllers (controller_manager + joint_trajectory_controllers)
must already be running.

Args:
  g_scale       (0.95) feedforward scale. HIL A/B 측정(2026-06-07): g=1.0 은 일부
                       자세에서 과보상(오버슈트), 전역 평균오차 최소는 g≈0.95.
  enable_left   (true)
  enable_right  (true) right arm; set false if a right motor is faulty.
"""
import os
import subprocess
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _setup(context, *args, **kwargs):
    g_scale = LaunchConfiguration("g_scale").perform(context)
    en_left = LaunchConfiguration("enable_left").perform(context).lower() == "true"
    en_right = LaunchConfiguration("enable_right").perform(context).lower() == "true"

    # Expand the SSOT xacro to a temp URDF for the KDL dynamics. The arm_base z
    # offset does NOT affect gravity torques (pure root translation), so this is
    # robust regardless of the model's base height.
    desc_share = FindPackageShare("openarmx_description").perform(context)
    xacro_path = os.path.join(desc_share, "urdf", "robot", "v10.urdf.xacro")
    urdf_path = os.path.join(tempfile.gettempdir(), "v10_gravity_comp.urdf")
    try:
        urdf = subprocess.run(
            ["xacro", xacro_path, "bimanual:=true"],
            capture_output=True, text=True, check=True).stdout
        with open(urdf_path, "w") as f:
            f.write(urdf)
    except Exception as e:  # noqa: BLE001
        print(f"[gravity_comp.launch] xacro expand failed: {e}")
        return []

    controllers = []
    if en_left:
        controllers.append("left_forward_effort_controller")
    if en_right:
        controllers.append("right_forward_effort_controller")

    nodes = []
    if controllers:
        # --controller-manager-timeout 60: a real-robot HW bringup needs ~30-40s
        # for CAN motor init before controller_manager answers. A short timeout
        # makes the spawner give up and die before the controllers ever load.
        # NO --unload-on-kill (see module docstring): the spawner activates the
        # controllers then exits; they persist even if the launch/spawner dies, so
        # gravity comp can't be silently disabled by a spawner death.
        nodes.append(Node(
            package="controller_manager", executable="spawner",
            arguments=[*controllers, "-c", "/controller_manager",
                       "--controller-manager-timeout", "60"],
            output="screen"))
    nodes.append(Node(
        package="openarmx_gravity_comp", executable="gravity_comp_node",
        name="gravity_comp_node", output="screen",
        parameters=[{
            "urdf_path": urdf_path,
            "g_scale": float(g_scale),
            "enable_left": en_left,
            "enable_right": en_right,
            "verbose": False,
        }]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        # 0.95: HIL A/B 격자 측정(2026-06-07)에서 g=1.0 은 일부 자세에서 살짝
        # 과보상(위로 오버슈트)이라, 전역 평균 위치오차가 g=0.95 에서 최소였음
        # (중심점 10.4→3.8mm). g>1.0 은 일관되게 악화. 자세별 최적은 다르나 0.95 가
        # 평균 절충. (docs/issues_and_fixes/2026-06-07_gravity_comp_hil_accuracy.md)
        DeclareLaunchArgument("g_scale", default_value="0.95"),
        DeclareLaunchArgument("enable_left", default_value="true"),
        DeclareLaunchArgument("enable_right", default_value="true"),
        OpaqueFunction(function=_setup),
    ])
