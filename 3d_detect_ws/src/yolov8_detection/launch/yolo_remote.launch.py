"""Launch the remote YOLOv8 bridge node (inference offloaded to pi_yolo_server).

Assumes realsense2_camera is already running on this PC. The node forwards one
color frame per DetectBox goal to `server_url`, receives 2D detections, and
publishes the canonical detection JSON on ~/detections (box_plane compatible).

Drop-in for the pick_and_place stack: launch with node_name:=yolov8_node so the
action/topics land at /yolov8_node/detect and /yolov8_node/detections — exactly
what box_align_node / box_plane_node already consume (no changes to them):
    ros2 launch yolov8_detection yolo_remote.launch.py node_name:=yolov8_node
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  TextSubstitution)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("yolov8_detection")
    config_path = PathJoinSubstitution([pkg_share, "config", "yolo_remote.yaml"])

    args = [
        DeclareLaunchArgument(
            "node_name", default_value="yolo_remote_node",
            description="Node name. Set to 'yolov8_node' to drop in for the pick_and_place "
                        "stack (action -> /yolov8_node/detect, topic -> /yolov8_node/detections)."),
        DeclareLaunchArgument("server_url", default_value="http://10.42.0.2:8080/detect"),
        DeclareLaunchArgument("use_depth", default_value="false"),
        DeclareLaunchArgument("image_topic",
                              default_value="/camera/camera/color/image_raw"),
        DeclareLaunchArgument("depth_topic",
                              default_value="/camera/camera/aligned_depth_to_color/image_raw"),
        DeclareLaunchArgument("camera_info_topic",
                              default_value="/camera/camera/color/camera_info"),
        DeclareLaunchArgument("base_frame", default_value="openarmx_body_link0"),
        DeclareLaunchArgument("camera_frame",
                              default_value="camera_color_optical_frame"),
    ]

    yolo_remote_node = Node(
        package="yolov8_detection",
        executable="yolo_remote_node",
        name=LaunchConfiguration("node_name"),
        output="screen",
        parameters=[
            config_path,
            {
                "server_url": LaunchConfiguration("server_url"),
                "use_depth": LaunchConfiguration("use_depth"),
                "image_topic": LaunchConfiguration("image_topic"),
                "depth_topic": LaunchConfiguration("depth_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
            },
        ],
    )

    # 인지(Perception): 2D 검출 -> 3D box 좌표 토픽 + RViz 마커 (TF 브로드캐스트 X).
    # 브리지와 함께 기동 → 검출 결과와 box 위치가 같이 나온다(모션 불필요).
    box_perception_node = Node(
        package="yolov8_detection",
        executable="box_perception_node",
        name="box_perception_node",
        output="screen",
        parameters=[{
            "detections_topic": [TextSubstitution(text="/"),
                                 LaunchConfiguration("node_name"),
                                 TextSubstitution(text="/detections")],
            "depth_topic": LaunchConfiguration("depth_topic"),
            "camera_info_topic": LaunchConfiguration("camera_info_topic"),
            "base_frame": LaunchConfiguration("base_frame"),
            "camera_frame": LaunchConfiguration("camera_frame"),
        }],
    )

    return LaunchDescription(args + [yolo_remote_node, box_perception_node])
