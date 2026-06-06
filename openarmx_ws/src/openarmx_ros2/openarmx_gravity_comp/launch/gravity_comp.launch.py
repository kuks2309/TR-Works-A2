#!/usr/bin/env python3
"""Bimanual gravity-compensation feedforward — toggleable from the Launch Manager.

Spawns the forward_effort_controller(s) with ``--unload-on-kill`` (so a Stop /
checkbox-off cleanly UNLOADS them via SIGINT → no stale feedforward torque left
on the motors) and runs ``gravity_comp_node`` which computes per-arm g(q) from
the robot model and publishes it to ``/<side>_forward_effort_controller/commands``.
The hardware write() adds that as the MIT-mode feedforward torque → removes the
no-integral droop.

Prerequisite: L1 controllers (controller_manager + joint_trajectory_controllers)
must already be running.

Args:
  g_scale       (1.0)  feedforward scale (1.0 ≈ full gravity; 1.05 slight over).
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
        # for CAN motor init before controller_manager answers. If gravity comp is
        # enabled slightly early (e.g. checkbox toggled right after HW start), a
        # short timeout makes the spawner give up and DIE → effort controllers
        # never load → no feedforward torque reaches the motors. 60s lets the
        # spawner WAIT for CM and self-recover instead of failing.
        nodes.append(Node(
            package="controller_manager", executable="spawner",
            arguments=[*controllers, "-c", "/controller_manager",
                       "--controller-manager-timeout", "60", "--unload-on-kill"],
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
        DeclareLaunchArgument("g_scale", default_value="1.0"),
        DeclareLaunchArgument("enable_left", default_value="true"),
        DeclareLaunchArgument("enable_right", default_value="true"),
        OpaqueFunction(function=_setup),
    ])
