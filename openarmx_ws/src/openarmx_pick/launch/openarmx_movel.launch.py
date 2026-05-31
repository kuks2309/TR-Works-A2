#!/usr/bin/env python3
"""Launch the cyclo_control QP+CBF MoveL solver bound to the OpenArmX left arm.

This reuses the robot-agnostic ``omx_movel_controller_node`` executable from
``cyclo_motion_controller_ros`` (it takes the URDF + controlled link as
parameters) and points it at the reduced single-arm OpenArmX URDF produced by
``scripts/gen_solver_urdf.py``.

Frames: the solver URDF root is ``openarmx_body_link0`` (world == body_link0 is
identity), which is exactly the frame the vision pipeline emits grasp poses in,
so MoveL goal poses can be fed straight through with no extra TF.

I/O:
  sub  <joint_states_topic>            sensor_msgs/JointState   (current feedback)
  sub  <movel_topic>                   robotis_interfaces/MoveL (goal pose)
  pub  <joint_command_topic>           trajectory_msgs/JointTrajectory (command)
  pub  <ee_pose_topic>                 geometry_msgs/PoseStamped (current EE)

Stage 1 uses the no-collision URDF (srdf empty) so the solver runs on
joint-limit + singularity CBF only. Swap in the collision URDF + an SRDF later.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("openarmx_pick")

    args = [
        DeclareLaunchArgument(
            "urdf_path",
            default_value=PathJoinSubstitution(
                [pkg, "urdf", "openarmx_left_solver.urdf"]),
            description="Reduced single-arm solver URDF (root = openarmx_body_link0)."),
        DeclareLaunchArgument(
            "srdf_path", default_value="",
            description="Empty for stage-1 (no collision pairs)."),
        DeclareLaunchArgument("base_frame", default_value="openarmx_body_link0"),
        DeclareLaunchArgument("controlled_link", default_value="openarmx_left_hand_tcp"),
        DeclareLaunchArgument("joint_states_topic", default_value="/joint_states"),
        DeclareLaunchArgument(
            "joint_command_topic",
            default_value="/openarmx/left_arm/joint_trajectory",
            description="Map to the OpenArmX left-arm forward controller input."),
        DeclareLaunchArgument("movel_topic", default_value="/openarmx/movel"),
        DeclareLaunchArgument("ee_pose_topic", default_value="/openarmx/left_ee_pose"),
        DeclareLaunchArgument("control_frequency", default_value="100.0"),
    ]

    movel = Node(
        package="cyclo_motion_controller_ros",
        executable="omx_movel_controller_node",
        name="openarmx_left_movel_controller",
        output="screen",
        parameters=[{
            "urdf_path": LaunchConfiguration("urdf_path"),
            "srdf_path": LaunchConfiguration("srdf_path"),
            "base_frame": LaunchConfiguration("base_frame"),
            "controlled_link": LaunchConfiguration("controlled_link"),
            "joint_states_topic": LaunchConfiguration("joint_states_topic"),
            "joint_command_topic": LaunchConfiguration("joint_command_topic"),
            "control_frequency": LaunchConfiguration("control_frequency"),
            # task-space gains / QP weights (cyclo defaults, tuned conservative)
            "time_step": 0.01,
            "trajectory_time": 0.05,
            "kp_position": 4.0,
            "kp_orientation": 2.5,
            "weight_task_position": 10.0,
            "weight_task_orientation": 1.0,
            "weight_damping": 0.05,
            "slack_penalty": 1000.0,
            "cbf_alpha": 5.0,
            "joint_state_timeout": 0.5,
        }],
        remappings=[
            ("~/movel", LaunchConfiguration("movel_topic")),
            ("~/current_pose", LaunchConfiguration("ee_pose_topic")),
        ],
    )

    return LaunchDescription([*args, movel])
