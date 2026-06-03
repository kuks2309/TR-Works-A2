"""Two cyclo OMX MoveL controllers (left + right) wired for OpenArmX.

Spawns:
  /openarmx_left_cyclo_movel   — left arm controller, subscribes to
                                  /openarmx/left/cyclo/movel
                                  (openarmx_scenario_player_msgs/msg/MoveL)
                                  publishes
                                  /left_joint_trajectory_controller/joint_trajectory
  /openarmx_right_cyclo_movel  — mirror for the right arm.

UI Cartesian Jog tab Backend dropdown sends MoveL goals to these absolute
topics. The cyclo controller resolves IK via QP+CBF and streams a single-point
JointTrajectory to the active JointTrajectoryController.

URDF/SRDF are pulled from the installed openarmx_description and
openarmx_bimanual_moveit_config packages — cyclo's HAL design accepts any
URDF/SRDF, so we inject openarmx's directly without forking models.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def _arm_node(arm: str, urdf_path: str, srdf_path: str) -> Node:
    return Node(
        package="cyclo_motion_controller_ros",
        executable="omx_movel_controller_node",
        name=f"openarmx_{arm}_cyclo_movel",
        output="screen",
        parameters=[{
            "urdf_path": urdf_path,
            "srdf_path": srdf_path,
            "base_frame": f"openarmx_{arm}_link0",
            "controlled_link": f"openarmx_{arm}_link7",
            "joint_states_topic": "/joint_states",
            "joint_command_topic":
                f"/{arm}_joint_trajectory_controller/joint_trajectory",
            "movel_topic": f"/openarmx/{arm}/cyclo/movel",
            "control_frequency": 100.0,
            "time_step": 0.01,
            "kp_position": 4.0,
            "kp_orientation": 2.5,
        }],
    )


def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory("openarmx_description"),
        "urdf", "robot", "openarmx_robot.urdf",
    )
    srdf_path = os.path.join(
        get_package_share_directory("openarmx_bimanual_moveit_config"),
        "config", "openarmx_bimanual.srdf",
    )
    return LaunchDescription([
        _arm_node("right", urdf_path, srdf_path),
        _arm_node("left",  urdf_path, srdf_path),
    ])
