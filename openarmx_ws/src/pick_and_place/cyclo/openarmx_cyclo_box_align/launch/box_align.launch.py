"""Launch the box_align action server.

This launch starts ONLY the box_align_node. The prerequisites must already be
running (start them separately):
  * box_perception_node — publishes /detected_boxes (geometry_msgs/PoseArray, base)
  * the cyclo MoveL controllers (/openarmx/{left,right}/movel)

Then trigger:
  ros2 action send_goal /openarmx/align_to_boxes \\
    openarmx_cyclo_box_align_msgs/action/AlignToBoxes "{z: 0.4}"
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("move_time", default_value="6.0"),
    ]
    node = Node(
        package="openarmx_cyclo_box_align",
        executable="box_align_node",
        name="box_align_node",
        output="screen",
        parameters=[{
            "move_time": LaunchConfiguration("move_time"),
        }],
    )
    return LaunchDescription([*args, node])
