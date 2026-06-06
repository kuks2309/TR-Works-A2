#!/usr/bin/env python3
"""Launch the ptp_box_align action server (direct Pinocchio IK -> single JTC endpoint).

The node loads the FULL robot model from the latched ``/robot_description`` topic (the
same model robot_state_publisher uses) and builds each arm's reduced single-arm model
at runtime via Pinocchio buildReducedModel. There is NO separate solver URDF — the arm
geometry (incl. arm_base z) lives in ONE place (the xacro), so the IK solver can never
drift from the real robot / TF.

Prerequisites (start separately):
  * robot_state_publisher — publishes /robot_description (so this node has a model)
  * box_perception_node — publishes /detected_boxes (geometry_msgs/PoseArray, base)
  * the arm joint_trajectory_controllers
    (/{left,right}_joint_trajectory_controller/follow_joint_trajectory)
  * the gripper controllers (/{left,right}_gripper_controller/gripper_cmd)

Then trigger:
  ros2 action send_goal /openarmx/ptp_align_to_boxes \\
    openarmx_ptp_box_align_msgs/action/AlignToBoxes "{z: 0.80}"
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        # JTC interpolation duration per move (s). Lower = faster (was 6.0).
        DeclareLaunchArgument("move_time", default_value="2.5"),
    ]

    node = Node(
        package="openarmx_ptp_box_align",
        executable="ptp_box_align_node",
        name="ptp_box_align_node",
        output="screen",
        parameters=[{
            "move_time": LaunchConfiguration("move_time"),
        }],
    )

    return LaunchDescription([*args, node])
