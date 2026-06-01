#!/usr/bin/env python3
"""Launch cyclo_control vr_controller_node (bimanual integrated QP+CBF) for OpenArmX.

Single solver node that owns BOTH arms (14 DOF) in one QP. Subscribes to
two PoseStamped goal topics (one per arm EE) and publishes two joint
trajectories (one per arm). Self-collision CBF between arms is wired in
when an SRDF is provided -- Stage 1 (this launch) keeps srdf_path empty
so only joint-limit + singularity CBF are active.

Topic layout (after this launch):
  sub  /openarmx/right/goal_pose    geometry_msgs/PoseStamped  (right EE target)
  sub  /openarmx/left/goal_pose     geometry_msgs/PoseStamped  (left  EE target)
  sub  /openarmx/right/elbow_pose   geometry_msgs/PoseStamped  (right elbow target, ignored if weight 0)
  sub  /openarmx/left/elbow_pose    geometry_msgs/PoseStamped  (left  elbow target,    "    "      )
  sub  /joint_states                sensor_msgs/JointState     (feedback)
  sub  /reactivate                  std_msgs/Bool              (recover from startup reject)
  pub  /openarmx/right_arm/joint_trajectory  trajectory_msgs/JointTrajectory
  pub  /openarmx/left_arm/joint_trajectory   trajectory_msgs/JointTrajectory
  pub  /openarmx/right/ee_pose      geometry_msgs/PoseStamped  (current right EE FK)
  pub  /openarmx/left/ee_pose       geometry_msgs/PoseStamped  (current left  EE FK)

AI-Worker -> OpenArmX adaptation:
  - r/l_gripper_name -> openarmx_{right,left}_link7         (EE tip link)
  - r/l_elbow_name   -> openarmx_{right,left}_link4         (elbow link)
  - r/l_gripper_joint-> openarmx_{right,left}_finger_joint1 (finger joints, frozen in our URDF)
  - lift_vel_bound = 0.0 -- OpenArmX has no lift; cyclo auto-skips if joint
    name with "lift_joint" substring is absent in URDF
  - weight_elbow_position = 0.0 -- elbow task disabled (no elbow input source yet)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("openarmx_motion")

    args = [
        DeclareLaunchArgument(
            "urdf_path",
            default_value=PathJoinSubstitution(
                [pkg, "urdf", "openarmx_bimanual_solver.urdf"]),
            description="Bimanual 14-DOF solver URDF (root = openarmx_body_link0)."),
        DeclareLaunchArgument(
            "srdf_path", default_value="",
            description="Empty for stage-1 (no collision pairs, no inter-arm CBF)."),
        DeclareLaunchArgument("joint_states_topic", default_value="/joint_states"),
        DeclareLaunchArgument("control_frequency", default_value="100.0"),
    ]

    vr = Node(
        package="cyclo_motion_controller_ros",
        executable="vr_controller_node",
        name="openarmx_vr_bimanual_controller",
        output="screen",
        parameters=[{
            # solver model
            "urdf_path": LaunchConfiguration("urdf_path"),
            "srdf_path": LaunchConfiguration("srdf_path"),
            "control_frequency": LaunchConfiguration("control_frequency"),
            "time_step": 0.01,
            "joint_state_timeout": 0.5,
            # OpenArmX link / joint names (override AI-Worker defaults)
            "r_gripper_name": "openarmx_right_link7",
            "l_gripper_name": "openarmx_left_link7",
            "r_elbow_name":   "openarmx_right_link4",
            "l_elbow_name":   "openarmx_left_link4",
            "right_gripper_joint": "openarmx_right_finger_joint1",
            "left_gripper_joint":  "openarmx_left_finger_joint1",
            # gains
            "kp_position": 50.0,
            "kp_orientation": 50.0,
            # QP task weights
            "weight_position": 10.0,
            "weight_orientation": 1.0,
            "weight_elbow_position": 0.0,    # disable elbow task (no elbow input)
            "weight_damping": 0.1,
            # CBF
            "slack_penalty": 1000.0,
            "cbf_alpha": 5.0,
            "collision_buffer": 0.05,
            "collision_safe_distance": 0.02,
            # lift (OpenArmX has none -> 0 vel bound locks it if model contains it)
            "lift_vel_bound": 0.0,
            # input topics (PoseStamped) -- aligned with ee_leader_marker output
            # so drag-and-follow works without an extra adapter node
            "r_goal_pose_topic": "/openarmx/right/ee_leader/goal_pose",
            "l_goal_pose_topic": "/openarmx/left/ee_leader/goal_pose",
            "r_elbow_pose_topic": "/openarmx/right/elbow_pose",
            "l_elbow_pose_topic": "/openarmx/left/elbow_pose",
            "joint_states_topic": LaunchConfiguration("joint_states_topic"),
            "reactivate_topic": "/openarmx/reactivate",
            # output joint trajectory topics -- map to ros2_control JTC inputs
            # (demo_sim / production both expose <controller_name>/joint_trajectory)
            "right_traj_topic": "/right_joint_trajectory_controller/joint_trajectory",
            "left_traj_topic":  "/left_joint_trajectory_controller/joint_trajectory",
            "right_raw_traj_topic": "/openarmx/right_arm/raw_joint_trajectory",
            "left_raw_traj_topic":  "/openarmx/left_arm/raw_joint_trajectory",
            # AI-Worker only -- harmless if joint absent in URDF
            "lift_topic": "/openarmx/lift/joint_trajectory",
            "r_gripper_pose_topic": "/openarmx/right/ee_pose",
            "l_gripper_pose_topic": "/openarmx/left/ee_pose",
            # startup gating (let the user start the marker near current EE pose)
            "startup_ref_pos_threshold": 0.15,
            "startup_ref_ori_threshold_deg": 45.0,
        }],
    )

    return LaunchDescription([*args, vr])
