#!/usr/bin/env python3
"""Bimanual drag-and-follow: RViz 6-DoF markers x2 -> adapter x2 -> vr_controller.

Brings up the full chain so the operator can drag the left / right EE markers
in RViz and have both arms follow in real time via the cyclo_control
QP+CBF VR controller (bimanual 14-DOF integrated solver).

Components:
  1. openarmx_vr_bimanual.launch.py  -- the cyclo vr_controller_node
  2. eef_interactive_marker_node x 2 -- one per arm, publishes
     robotis_interfaces/MoveL while the operator drags the RViz marker
  3. marker_to_posestamped x 2       -- converts MoveL.pose -> PoseStamped
     for vr_controller's r/l_goal_pose subscriptions

Topic flow:
  RViz marker drag (left)
    --> /openarmx/left/marker_movel   (robotis_interfaces/MoveL)
        --> marker_to_posestamped (left adapter)
              --> /openarmx/left/goal_pose   (geometry_msgs/PoseStamped)
                    --> vr_controller_node (subscribes l_goal_pose)
                          --> /openarmx/left_arm/joint_trajectory
  ...and symmetrically for right.

The eef_interactive_marker_node default ``publish_while_dragging = true``
gives instantaneous follow -- no Plan/Execute clicks needed.

Requires a joint_states publisher upstream (demo_sim.launch.py from
openarmx_bimanual_moveit_config, or a real robot bringup). vr_controller
holds motion until the first joint_state arrives and the operator's marker
pose is within startup_ref_pos/ori thresholds of the current EE pose --
since the marker node starts at the live FK pose, this is automatic.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _marker_node(side, color_rgb):
    """One eef_interactive_marker_node per arm."""
    r, g, b = color_rgb
    return Node(
        package="cyclo_motion_controller_ros",
        executable="interactive_marker_node",
        name=f"openarmx_{side}_marker",
        output="screen",
        parameters=[{
            "base_frame": "openarmx_body_link0",
            "controlled_link": f"openarmx_{side}_link7",
            "goal_topic": f"/openarmx/{side}/marker_movel",
            "server_name": f"openarmx_{side}_marker",
            "marker_name": f"{side}_goal",
            "marker_description": f"OpenArmX {side} EE goal",
            "marker_scale": 0.15,
            "publish_while_dragging": True,
            "marker_color_r": float(r),
            "marker_color_g": float(g),
            "marker_color_b": float(b),
        }],
    )


def _adapter_node(side):
    """One marker_to_posestamped per arm (MoveL -> PoseStamped)."""
    return Node(
        package="openarmx_motion",
        executable="marker_to_posestamped",
        name=f"openarmx_{side}_marker_adapter",
        output="screen",
        parameters=[{
            "input_topic": f"/openarmx/{side}/marker_movel",
            "output_topic": f"/openarmx/{side}/goal_pose",
        }],
    )


def generate_launch_description():
    pkg = FindPackageShare("openarmx_motion")

    args = [
        DeclareLaunchArgument(
            "srdf_path", default_value="",
            description="Empty for stage-1; set when SRDF is added."),
        DeclareLaunchArgument("joint_states_topic", default_value="/joint_states"),
        DeclareLaunchArgument("control_frequency", default_value="100.0"),
    ]

    vr = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, "launch", "openarmx_vr_bimanual.launch.py"])),
        launch_arguments={
            "srdf_path": LaunchConfiguration("srdf_path"),
            "joint_states_topic": LaunchConfiguration("joint_states_topic"),
            "control_frequency": LaunchConfiguration("control_frequency"),
        }.items(),
    )

    # right = red, left = blue (matches typical mirror convention from operator pov)
    nodes = [
        _marker_node("right", (1.0, 0.2, 0.2)),
        _marker_node("left",  (0.2, 0.2, 1.0)),
        _adapter_node("right"),
        _adapter_node("left"),
    ]

    return LaunchDescription([*args, vr, *nodes])
