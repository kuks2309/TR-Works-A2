"""Scenario authoring workflow launch — RViz + EE Leader Markers only.

NOTE: This launch does NOT start scenario_player_node. It is intended for the
scenario AUTHORING stage (drag the EE Leader Marker in RViz, capture pose to
JSON via the openarmx_scenario_ui Cartesian Control tab). Once scenarios are
registered, start scenario_player_node separately (Launch Manager > "Scenario
Player node (backend only)").

scenario_ui.py auto-spawns this launch on startup so the user sees RViz +
both 6-DoF markers immediately after opening the UI.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ee_pkg = FindPackageShare("ee_leader_marker")
    player_pkg = FindPackageShare("openarmx_scenario_player")

    args = [
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution(
                [player_pkg, "config", "openarmx_scenario.rviz"]),
            description="RViz config — openarmx_scenario.rviz (Grid + RobotModel + 2 markers)",
        ),
    ]

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="openarmx_scenario_rviz",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
    )

    ee_leader = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [ee_pkg, "launch", "ee_leader_marker_bimanual.launch.py"])),
        launch_arguments={
            "base_frame": "openarmx_body_link0",
            "left_controlled_link": "openarmx_left_link7",
            "right_controlled_link": "openarmx_right_link7",
            "left_goal_topic": "/openarmx/left/ee_leader/goal_pose",
            "right_goal_topic": "/openarmx/right/ee_leader/goal_pose",
            "start_rviz": "false",   # this wrapper owns the RViz
        }.items(),
    )

    return LaunchDescription([*args, rviz, ee_leader])
