#!/usr/bin/env python3
"""Spawn two EE Leader Markers (left + right) and an optional RViz.

Robot-agnostic launch. To port to a new project, change the launch arguments
``base_frame``, ``left_controlled_link``, ``right_controlled_link`` to match
the target robot's URDF.

Topic outputs (PoseStamped, no custom messages):
  /<left_goal_topic>   default /ee_leader/left/goal_pose
  /<right_goal_topic>  default /ee_leader/right/goal_pose
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _marker(side, color_rgb, base_frame, controlled_link, goal_topic):
    r, g, b = color_rgb
    return Node(
        package="ee_leader_marker",
        executable="eef_interactive_marker_node",
        name=f"ee_leader_{side}_marker",
        output="screen",
        parameters=[{
            "base_frame": base_frame,
            "controlled_link": controlled_link,
            "goal_topic": goal_topic,
            "server_name": f"ee_leader_{side}_marker",
            "marker_name": f"{side}_goal",
            "marker_description": f"{side.capitalize()} EE Leader Marker",
            "marker_scale": 0.15,
            "publish_while_dragging": True,
            "marker_color_r": float(r),
            "marker_color_g": float(g),
            "marker_color_b": float(b),
        }],
    )


def generate_launch_description():
    pkg = FindPackageShare("ee_leader_marker")

    args = [
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("left_controlled_link", default_value="left_end_effector"),
        DeclareLaunchArgument("right_controlled_link", default_value="right_end_effector"),
        DeclareLaunchArgument("left_goal_topic", default_value="/ee_leader/left/goal_pose"),
        DeclareLaunchArgument("right_goal_topic", default_value="/ee_leader/right/goal_pose"),
        DeclareLaunchArgument(
            "start_rviz", default_value="true",
            description="Bring up RViz with InteractiveMarkers displays preconfigured."),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution(
                [pkg, "config", "ee_leader_marker_bimanual.rviz"])),
    ]

    nodes = [
        _marker("right", (1.0, 0.2, 0.2),
                LaunchConfiguration("base_frame"),
                LaunchConfiguration("right_controlled_link"),
                LaunchConfiguration("right_goal_topic")),
        _marker("left", (0.2, 0.2, 1.0),
                LaunchConfiguration("base_frame"),
                LaunchConfiguration("left_controlled_link"),
                LaunchConfiguration("left_goal_topic")),
    ]

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="ee_leader_marker_rviz",
        output="log",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        condition=IfCondition(LaunchConfiguration("start_rviz")),
    )

    return LaunchDescription([*args, *nodes, rviz])
