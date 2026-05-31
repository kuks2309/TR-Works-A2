"""Launch scenario_player + EE Leader Marker (bimanual) for OpenArmX.

Wraps the stub scenario_player_node and the robot-agnostic ee_leader_marker
package, pre-configured for OpenArmX bimanual (base = openarmx_body_link0,
EE tips = openarmx_{right,left}_link7).

The two systems are decoupled: scenario_player runs its scenario step loop;
EE Leader Marker publishes /openarmx/{right,left}/ee_leader/goal_pose
(geometry_msgs/PoseStamped) for any downstream consumer (vr_controller,
custom recorder, etc.). RViz auto-loaded with both InteractiveMarkers
displays via ee_leader_marker_bimanual.launch.py.

Use OPENARMX_SCENARIOS_DIR to override the scenario search path, exactly
like scenario_player.launch.py:

    OPENARMX_SCENARIOS_DIR=$HOME/openarmx_ws/scenarios \\
      ros2 launch openarmx_scenario_player scenario_player_with_ee_leader.launch.py
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_path = os.environ.get(
        "OPENARMX_SCENARIOS_DIR",
        os.path.expanduser("~/openarmx_ws/scenarios"),
    )

    ee_pkg = FindPackageShare("ee_leader_marker")

    args = [
        DeclareLaunchArgument(
            "scenario_search_path",
            default_value=default_path,
            description="Directory holding <scenario_name>/scenario.json subtrees",
        ),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution(
                [ee_pkg, "config", "ee_leader_marker_bimanual.rviz"]),
        ),
    ]

    scenario = Node(
        package="openarmx_scenario_player",
        executable="scenario_player_node.py",
        name="scenario_player",
        output="screen",
        parameters=[{
            "scenario_search_path": LaunchConfiguration("scenario_search_path"),
        }],
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
            "start_rviz": LaunchConfiguration("start_rviz"),
            "rviz_config": LaunchConfiguration("rviz_config"),
        }.items(),
    )

    return LaunchDescription([*args, scenario, ee_leader])
