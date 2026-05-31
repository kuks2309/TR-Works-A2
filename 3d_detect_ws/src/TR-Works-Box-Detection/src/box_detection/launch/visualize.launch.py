"""Launch RViz, the plane visualizer node, and (optionally) ros2 bag play.

Usage:
    ros2 launch box_detection visualize.launch.py \
        bag:=short_box_wide_a \
        bag_root:=/home/amap/TR-Works/TR-Works_ros2_ws/rosbag/bags/20260506

Args:
    bag        : bag directory name (under bag_root). Required to publish
                 plane markers. The bag is also auto-played if play:=true.
    bag_root   : parent directory containing the bag dirs. Defaults to the
                 rosbag set in this workspace.
    play       : true (default) to also start `ros2 bag play --clock --loop`.
    rviz       : true (default) to also start RViz with config/box_analysis.rviz.
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


DEFAULT_BAG_ROOT = "/home/amap/TR-Works/TR-Works_ros2_ws/rosbag/bags/20260506"


def generate_launch_description():
    pkg_share = get_package_share_directory("box_detection")
    rviz_config = os.path.join(pkg_share, "config", "box_analysis.rviz")

    bag_arg = DeclareLaunchArgument(
        "bag", default_value="short_box_wide_a",
        description="bag directory name under bag_root")
    bag_root_arg = DeclareLaunchArgument(
        "bag_root", default_value=DEFAULT_BAG_ROOT,
        description="parent directory containing bag dirs")
    play_arg = DeclareLaunchArgument(
        "play", default_value="true", description="auto-play the bag in loop")
    rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="true", description="auto-launch RViz")

    bag = LaunchConfiguration("bag")
    bag_root = LaunchConfiguration("bag_root")

    visualizer = Node(
        package="box_detection",
        executable="visualize_planes",
        name="plane_visualizer",
        output="screen",
        arguments=["--bag", bag, "--root", bag_root],
    )

    bag_play = ExecuteProcess(
        cmd=["ros2", "bag", "play",
             PathJoinSubstitution([bag_root, bag]),
             "--clock", "--loop"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("play")),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription([
        bag_arg, bag_root_arg, play_arg, rviz_arg,
        visualizer, bag_play, rviz,
    ])
