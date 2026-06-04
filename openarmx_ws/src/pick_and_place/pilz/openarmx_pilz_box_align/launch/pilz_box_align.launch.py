"""Launch the pilz_box_align action server (node only).

Prerequisites (start separately):
  * box_perception_node — publishes /detected_boxes (geometry_msgs/PoseArray, base)
  * MoveIt move_group with the Pilz pipeline (/plan_kinematic_path service) and
    the arm JTCs (/{left,right}_joint_trajectory_controller/joint_trajectory)

Trigger:
  ros2 action send_goal /openarmx/pilz_align_to_boxes \\
    openarmx_pilz_box_align_msgs/action/AlignToBoxes "{z: 0.4}"
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("default_vel_scale", default_value="0.3"),
        DeclareLaunchArgument("plan_time", default_value="5.0"),
    ]
    node = Node(
        package="openarmx_pilz_box_align",
        executable="pilz_box_align_node",
        name="pilz_box_align_node",
        output="screen",
        parameters=[{
            "default_vel_scale": LaunchConfiguration("default_vel_scale"),
            "plan_time": LaunchConfiguration("plan_time"),
        }],
    )
    return LaunchDescription([*args, node])
