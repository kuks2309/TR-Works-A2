"""Launch only the YOLOv8 node (assume realsense2_camera is already running)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("yolov8_detection")
    config_path = PathJoinSubstitution([pkg_share, "config", "yolov8.yaml"])

    args = [
        DeclareLaunchArgument("model", default_value="yolov8n.pt"),
        DeclareLaunchArgument("device", default_value="cpu"),
        DeclareLaunchArgument("use_depth", default_value="false"),
        DeclareLaunchArgument("image_topic",
                              default_value="/camera/camera/color/image_raw"),
        DeclareLaunchArgument("depth_topic",
                              default_value="/camera/camera/aligned_depth_to_color/image_raw"),
        DeclareLaunchArgument("camera_info_topic",
                              default_value="/camera/camera/color/camera_info"),
    ]

    yolov8_node = Node(
        package="yolov8_detection",
        executable="yolov8_node",
        name="yolov8_node",
        output="screen",
        parameters=[
            config_path,
            {
                "model": LaunchConfiguration("model"),
                "device": LaunchConfiguration("device"),
                "use_depth": LaunchConfiguration("use_depth"),
                "image_topic": LaunchConfiguration("image_topic"),
                "depth_topic": LaunchConfiguration("depth_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
            },
        ],
    )

    return LaunchDescription(args + [yolov8_node])
