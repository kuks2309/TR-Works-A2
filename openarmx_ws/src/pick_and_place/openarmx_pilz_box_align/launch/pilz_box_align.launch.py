"""Launch the pilz_box_align action server (node only).

Prerequisites (start separately):
  * D435 camera (realsense2_camera) on /camera/camera/...
  * YOLOv8 DetectBox action server (/yolov8_node/detect)
  * calibrated base->camera TF
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
        DeclareLaunchArgument("n_frames", default_value="5"),
        DeclareLaunchArgument("cluster_radius", default_value="0.08"),
        DeclareLaunchArgument("min_hits", default_value="2"),
        DeclareLaunchArgument("default_confidence", default_value="0.02"),
        DeclareLaunchArgument("default_vel_scale", default_value="0.3"),
        DeclareLaunchArgument("plan_time", default_value="5.0"),
    ]
    node = Node(
        package="openarmx_pilz_box_align",
        executable="pilz_box_align_node",
        name="pilz_box_align_node",
        output="screen",
        parameters=[{
            "n_frames": LaunchConfiguration("n_frames"),
            "cluster_radius": LaunchConfiguration("cluster_radius"),
            "min_hits": LaunchConfiguration("min_hits"),
            "default_confidence": LaunchConfiguration("default_confidence"),
            "default_vel_scale": LaunchConfiguration("default_vel_scale"),
            "plan_time": LaunchConfiguration("plan_time"),
        }],
    )
    return LaunchDescription([*args, node])
