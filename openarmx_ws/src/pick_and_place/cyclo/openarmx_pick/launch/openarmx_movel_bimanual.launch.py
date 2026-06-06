#!/usr/bin/env python3
"""Launch cyclo_control QP+CBF MoveL solvers for BOTH OpenArmX arms.

Spawns two independent ``omx_movel_controller_node`` instances (one per arm),
each bound to the reduced single-arm solver URDF for that side. The two solvers
are fully independent ROS nodes -- they share ``/joint_states`` only and
publish to disjoint joint_command / movel / ee_pose topics so they can run
concurrently without coupling.

Per-arm topic layout:
  left:  /openarmx/left/movel,  /openarmx/left/ee_pose,  /openarmx/left_arm/joint_trajectory
  right: /openarmx/right/movel, /openarmx/right/ee_pose, /openarmx/right_arm/joint_trajectory

Stage 1 uses the --no-collision solver URDFs (SRDF empty) -- only joint-limit
and singularity CBF active. Add SRDFs and switch to collision URDFs later to
enable inter-arm self-collision CBF.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _arm_node(side, pkg):
    """One omx_movel_controller_node parameterised for the given arm side."""
    urdf = PathJoinSubstitution([pkg, "urdf", f"openarmx_{side}_solver.urdf"])
    return Node(
        package="cyclo_motion_controller_ros",
        executable="omx_movel_controller_node",
        name=f"openarmx_{side}_movel_controller",
        output="screen",
        parameters=[{
            "urdf_path": urdf,
            "srdf_path": LaunchConfiguration("srdf_path"),
            "base_frame": "openarmx_body_link0",
            "controlled_link": f"openarmx_{side}_hand_tcp",
            "joint_states_topic": LaunchConfiguration("joint_states_topic"),
            # Publish to the ros2_control-standard JTC topic — controller
            # spawner namespaces the topic by the controller's name in
            # controllers.yaml (e.g. right_joint_trajectory_controller).
            # The previous /openarmx/{side}_arm/joint_trajectory had no
            # subscriber and silently dropped every trajectory.
            "joint_command_topic": f"/{side}_joint_trajectory_controller/joint_trajectory",
            # [B — China 2026-06-06] output_mode=forward 면 cyclo 매-틱 위치(7개)를 아래 토픽
            # ({side}_arm_position_controller/commands, JTC 대신 forward passthrough)에 직결.
            "output_mode": LaunchConfiguration("output_mode"),
            "forward_command_topic": f"/{side}_arm_position_controller/commands",
            "batch_trajectory": LaunchConfiguration("batch_trajectory"),
            "control_frequency": LaunchConfiguration("control_frequency"),
            # 게인: 단일팔 openarmx_movel.launch.py 의 안정화 튜닝값과 일치시킴.
            # upstream 기본값(kp 50, damping 0.001, traj_time 0)은 이 로봇/JTC+mock
            # 조합에서 저-manipulability 방향 qdot 폭주(EE fly)를 유발했음(2026-06-06
            # rosbag 측정: 5mm 명령에 qdot 6.4 rad/s·EE 100mm 이탈). 고게인↓·댐핑↑로 안정화.
            "time_step": 0.01,
            "trajectory_time": 0.05,
            "kp_position": 4.0,
            "kp_orientation": 2.5,
            "weight_task_position": 10.0,
            "weight_task_orientation": 1.0,
            "weight_damping": 0.05,
            "slack_penalty": 1000.0,
            "cbf_alpha": 5.0,
            "collision_buffer": 0.01,
            "collision_safe_distance": 0.005,
            "joint_state_timeout": 0.5,
        }],
        remappings=[
            ("~/movel", f"/openarmx/{side}/movel"),
            ("~/current_pose", f"/openarmx/{side}/ee_pose"),
        ],
    )


def generate_launch_description():
    pkg = FindPackageShare("openarmx_pick")

    args = [
        DeclareLaunchArgument(
            "srdf_path", default_value="",
            description="Empty for stage-1 (no collision pairs). Set when SRDFs are added."),
        DeclareLaunchArgument("joint_states_topic", default_value="/joint_states"),
        DeclareLaunchArgument("control_frequency", default_value="100.0"),
        # [B — China] jtc=JointTrajectory→JTC(기존) | forward=Float64MultiArray→
        # {side}_arm_position_controller(JTC 우회, cyclo 직접제어). forward 면 batch=false 권장.
        DeclareLaunchArgument("output_mode", default_value="jtc"),
        DeclareLaunchArgument("batch_trajectory", default_value="true"),
    ]

    return LaunchDescription([*args, _arm_node("left", pkg), _arm_node("right", pkg)])
