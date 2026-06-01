"""Launch RealSense D435 + YOLOv8 inference node together.

Usage:
    ros2 launch yolov8_detection yolov8_d435.launch.py
    ros2 launch yolov8_detection yolov8_d435.launch.py use_depth:=true device:=cuda:0

The launch reads parameters from config/yolov8.yaml and lets the most common
ones be overridden from the command line.
"""

import math
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# Calibrated openarmx_body_link0 -> d435 camera body TF
# (2026-06-01 ChArUco recalib, 60deg mount; same value baked into
#  openarmx_description/urdf/robot/v10.urdf.xacro)
_CAM_TF = dict(
    x=0.065430, y=0.000987, z=0.641921,
    roll_deg=0.6846, pitch_deg=59.7350, yaw_deg=0.4038,
)


def _load_urdf(path: str) -> str:
    if not os.path.isfile(path):
        return ""
    with open(path, "r") as f:
        return f.read()


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("yolov8_detection")
    config_path = PathJoinSubstitution([pkg_share, "config", "yolov8.yaml"])

    args = [
        DeclareLaunchArgument("launch_realsense", default_value="true",
                              description="Launch realsense2_camera alongside the YOLO node."),
        DeclareLaunchArgument("rviz", default_value="false",
                              description="Open RViz2 with a preset showing /yolov8_node/image_annotated."),
        DeclareLaunchArgument("model", default_value="yolov8n.pt"),
        DeclareLaunchArgument("device", default_value="cpu"),
        DeclareLaunchArgument("use_depth", default_value="false"),
        DeclareLaunchArgument("confidence", default_value="0.35"),
        DeclareLaunchArgument("image_size", default_value="640"),
        DeclareLaunchArgument("image_topic",
                              default_value="/camera/camera/color/image_raw"),
        DeclareLaunchArgument("depth_topic",
                              default_value="/camera/camera/aligned_depth_to_color/image_raw"),
        DeclareLaunchArgument("camera_info_topic",
                              default_value="/camera/camera/color/camera_info"),
        DeclareLaunchArgument("prompts", default_value="",
                              description="Comma-separated YOLO-World prompts, e.g. 'box,cardboard box,carton'."),
        DeclareLaunchArgument("fit_box_plane", default_value="false",
                              description="Run the box-top RANSAC plane fitter. Implies use_depth:=true."),
        DeclareLaunchArgument("box_keywords",
                              default_value="box,cardboard box,carton,package",
                              description="Comma-separated detection class names treated as boxes."),
        DeclareLaunchArgument(
            "show_robot", default_value="false",
            description="Publish openarmx URDF via robot_state_publisher and add the body_link0->camera static TF."),
        DeclareLaunchArgument(
            "urdf_path",
            default_value="/home/openarmx/TR-Works/kkw/China/openarmx_ws/src/openarmx_description/urdf/robot/openarmx_robot.urdf",
            description="Path to the openarmx URDF used by robot_state_publisher when show_robot:=true."),
        DeclareLaunchArgument(
            "camera_link_frame", default_value="camera_link",
            description="Frame published by realsense2_camera_node that the static TF should target."),
    ]
    rviz_config = PathJoinSubstitution([pkg_share, "config", "yolov8.rviz"])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    )

    from launch.substitutions import PythonExpression
    need_aligned = PythonExpression([
        "'true' if ('",
        LaunchConfiguration("use_depth"),
        "' == 'true' or '",
        LaunchConfiguration("fit_box_plane"),
        "' == 'true') else 'false'",
    ])

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("realsense2_camera"),
            "/launch/rs_launch.py",
        ]),
        launch_arguments={
            "enable_color": "true",
            "enable_depth": "true",
            "align_depth.enable": need_aligned,
            # When the plane fitter is on, publish the RGB-textured cloud
            # so RViz can overlay the plane disk on the actual scene.
            "pointcloud.enable": need_aligned,
            "pointcloud.stream_filter": "2",  # 2 = color stream (textured cloud)
        }.items(),
        condition=None,
    )

    box_plane_node = Node(
        package="yolov8_detection",
        executable="box_plane_node",
        name="box_plane_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("fit_box_plane")),
        parameters=[{
            "detections_topic": "/yolov8_node/detections",
            "depth_topic": LaunchConfiguration("depth_topic"),
            "camera_info_topic": LaunchConfiguration("camera_info_topic"),
            "box_keywords": LaunchConfiguration("box_keywords"),
        }],
    )

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
                "confidence": LaunchConfiguration("confidence"),
                "image_size": LaunchConfiguration("image_size"),
                "image_topic": LaunchConfiguration("image_topic"),
                "depth_topic": LaunchConfiguration("depth_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                "prompts": LaunchConfiguration("prompts"),
            },
        ],
    )

    # ---- Robot model (openarmx) bringup ----
    # We resolve the URDF at launch-load time so we can pass it as a parameter
    # value instead of relying on xacro / Command. Mesh resolution still needs
    # the openarmx_description package on AMENT_PREFIX_PATH (source openarmx_ws).
    urdf_default = "/home/openarmx/TR-Works/kkw/China/openarmx_ws/src/openarmx_description/urdf/robot/openarmx_robot.urdf"
    urdf_xml = _load_urdf(urdf_default)

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        condition=IfCondition(LaunchConfiguration("show_robot")),
        output="screen",
        parameters=[{
            "robot_description": urdf_xml,
            "publish_frequency": 30.0,
        }],
    )
    joint_state_pub = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        condition=IfCondition(LaunchConfiguration("show_robot")),
        output="screen",
        parameters=[{"rate": 30}],
    )
    camera_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="openarmx_body_link0_to_camera_link_tf",
        condition=IfCondition(LaunchConfiguration("show_robot")),
        arguments=[
            "--x", str(_CAM_TF["x"]),
            "--y", str(_CAM_TF["y"]),
            "--z", str(_CAM_TF["z"]),
            "--roll",  str(math.radians(_CAM_TF["roll_deg"])),
            "--pitch", str(math.radians(_CAM_TF["pitch_deg"])),
            "--yaw",   str(math.radians(_CAM_TF["yaw_deg"])),
            "--frame-id", "openarmx_body_link0",
            "--child-frame-id", LaunchConfiguration("camera_link_frame"),
        ],
        output="screen",
    )

    return LaunchDescription(args + [
        realsense_launch,
        yolov8_node,
        box_plane_node,
        robot_state_pub,
        joint_state_pub,
        camera_static_tf,
        rviz_node,
    ])
