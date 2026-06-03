"""Launch the box_align action server.

This launch starts ONLY the box_align_node. The prerequisites must already be
running (start them separately):
  * D435 camera (realsense2_camera) on /camera/camera/...
  * YOLOv8 DetectBox action server (/yolov8_node/detect) -- see 3d_detect_ws
  * the calibrated base->camera TF (e.g. via the scenario stack's d435 bridge)
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
        DeclareLaunchArgument("n_frames", default_value="5"),
        DeclareLaunchArgument("cluster_radius", default_value="0.06"),
        DeclareLaunchArgument("move_time", default_value="6.0"),
        DeclareLaunchArgument("default_confidence", default_value="0.01"),
    ]
    node = Node(
        package="openarmx_cyclo_box_align",
        executable="box_align_node",
        name="box_align_node",
        output="screen",
        parameters=[{
            "n_frames": LaunchConfiguration("n_frames"),
            "cluster_radius": LaunchConfiguration("cluster_radius"),
            "move_time": LaunchConfiguration("move_time"),
            "default_confidence": LaunchConfiguration("default_confidence"),
        }],
    )
    return LaunchDescription([*args, node])
