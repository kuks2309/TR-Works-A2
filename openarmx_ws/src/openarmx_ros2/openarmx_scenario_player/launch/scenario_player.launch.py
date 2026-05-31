"""Launch the openarmx_scenario_player stub backend.

Use OPENARMX_SCENARIOS_DIR to override the search path:

    OPENARMX_SCENARIOS_DIR=$HOME/openarmx_ws/scenarios \\
      ros2 launch openarmx_scenario_player scenario_player.launch.py
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_path = os.environ.get(
        "OPENARMX_SCENARIOS_DIR",
        os.path.expanduser("~/openarmx_ws/scenarios"),
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "scenario_search_path",
            default_value=default_path,
            description="Directory holding <scenario_name>/scenario.json subtrees",
        ),
        Node(
            package="openarmx_scenario_player",
            executable="scenario_player_node.py",
            name="scenario_player",
            output="screen",
            parameters=[{
                "scenario_search_path": LaunchConfiguration("scenario_search_path"),
            }],
        ),
    ])
