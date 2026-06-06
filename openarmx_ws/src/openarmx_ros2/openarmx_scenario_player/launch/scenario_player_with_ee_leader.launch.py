"""Launch scenario_player + EE Leader Marker (bimanual) for OpenArmX.

Wraps the stub scenario_player_node and the robot-agnostic ee_leader_marker
package, pre-configured for OpenArmX bimanual (base = openarmx_body_link0,
EE tips = openarmx_{right,left}_link7).

The two systems are decoupled: scenario_player runs its scenario step loop;
EE Leader Marker publishes /openarmx/{right,left}/ee_leader/goal_pose
(geometry_msgs/PoseStamped) for any downstream consumer (vr_controller,
custom recorder, etc.). RViz auto-loaded with both InteractiveMarkers
displays via ee_leader_marker_bimanual.launch.py.

Use OPENARMX_SCENARIOS_DIR to override the scenario search path, exactly
like scenario_player.launch.py:

    OPENARMX_SCENARIOS_DIR=$HOME/openarmx_ws/scenarios \\
      ros2 launch openarmx_scenario_player scenario_player_with_ee_leader.launch.py
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# RViz config lives in the SRC tree, NOT install/share. Point RViz directly at
# the source file (instead of FindPackageShare -> install/share/...) so that
# "Save Config" (Ctrl+S) in RViz writes back to source and the layout survives
# the next `colcon build` (symlink-install regenerates the install copy and
# would otherwise wipe any saved RViz changes).
_SRC_RVIZ_CONFIG = (
    "/home/openarmx/TR-Works/kkw/China/openarmx_ws/src/openarmx_ros2/"
    "openarmx_scenario_player/config/openarmx_scenario.rviz"
)


def generate_launch_description():
    default_path = os.environ.get(
        "OPENARMX_SCENARIOS_DIR",
        os.path.expanduser("~/openarmx_ws/scenarios"),
    )

    ee_pkg = FindPackageShare("ee_leader_marker")
    player_pkg = FindPackageShare("openarmx_scenario_player")
    motion_pkg = FindPackageShare("openarmx_motion")
    pick_pkg = FindPackageShare("openarmx_pick")

    args = [
        DeclareLaunchArgument(
            "scenario_search_path",
            default_value=default_path,
            description="Directory holding <scenario_name>/scenario.json subtrees",
        ),
        DeclareLaunchArgument(
            # Renamed from "start_rviz" → "spawn_workflow_rviz" to avoid scope
            # collision with the IncludeLaunchDescription below (ee_leader_marker
            # bimanual.launch.py also accepts a "start_rviz" arg; if both share
            # the same name, ros2 launch leaks the sub-launch's "false" back to
            # this scope and the outer IfCondition silently skips spawning).
            "spawn_workflow_rviz", default_value="true",
            description=("True by default -- spawns RViz with the openarmx "
                         "Cartesian/EE-Leader workflow config "
                         "(openarmx_scenario_player/config/openarmx_scenario.rviz). "
                         "Set false when reusing demo_sim's RViz.")),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=_SRC_RVIZ_CONFIG,
            description=("RViz config path -- the SRC-tree openarmx_scenario.rviz "
                         "so RViz 'Save Config' persists across colcon build."),
        ),
        DeclareLaunchArgument(
            "start_sim", default_value="true",
            description=("True spawns openarmx_cyclo_sim (MoveIt-free mock "
                         "hardware + JTCs + RSP). Set false when demo_sim or "
                         "real hardware is already providing /joint_states and "
                         "the JTCs.")),
        DeclareLaunchArgument(
            # [cyclo 자동기동 제거 2026-06-07] 사용자 요청: cyclo MoveL 은 자동기동하지 않고
            # opt-in 으로만 켠다(start_movel_controllers:=true 또는 Launch Manager 의 cyclo
            # 행 Start). 기본값 true→false.
            "start_movel_controllers", default_value="false",
            description=("True spawns the cyclo single-arm MoveL controllers "
                         "(openarmx_pick/openarmx_movel_bimanual.launch.py -- "
                         "two omx_movel_controller_node instances, one per "
                         "arm). UI's Cartesian Control tab picks which side "
                         "to command via the /openarmx/{left,right}/movel "
                         "topics. NOTE: vr_controller (bimanual integrated "
                         "QP) is intentionally NOT spawned here.")),
        DeclareLaunchArgument(
            "start_player", default_value="true",
            description=("True spawns scenario_player_node. Set false during "
                         "the scenario AUTHORING stage (markers + movel "
                         "controllers + RViz only) before any scenarios are "
                         "registered.")),
    ]

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="openarmx_scenario_rviz",
        output="log",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        condition=IfCondition(LaunchConfiguration("spawn_workflow_rviz")),
    )

    # Bridge the URDF-baked camera link (d435_center_link, published by RSP from
    # the calibrated extrinsic) to the live realsense2_camera frame tree
    # (camera_link -> *_optical_frame). Identity, because d435_center_link IS the
    # calibrated camera_link pose. Without this edge the external /camera/camera
    # RGBD cloud can't resolve into openarmx_body_link0 and won't render in RViz.
    camera_tf_bridge = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="d435_center_to_camera_link_tf",
        arguments=["--x", "0", "--y", "0", "--z", "0",
                   "--roll", "0", "--pitch", "0", "--yaw", "0",
                   "--frame-id", "d435_center_link", "--child-frame-id", "camera_link"],
        output="log",
    )

    scenario = Node(
        package="openarmx_scenario_player",
        executable="scenario_player_node.py",
        name="scenario_player",
        output="screen",
        parameters=[{
            "scenario_search_path": LaunchConfiguration("scenario_search_path"),
        }],
        condition=IfCondition(LaunchConfiguration("start_player")),
    )

    ee_leader = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [ee_pkg, "launch", "ee_leader_marker_bimanual.launch.py"])),
        launch_arguments={
            "base_frame": "openarmx_body_link0",
            "left_controlled_link": "openarmx_left_link7",
            "right_controlled_link": "openarmx_right_link7",
            "left_goal_topic": "/openarmx/left/ee_leader/goal_pose",
            "right_goal_topic": "/openarmx/right/ee_leader/goal_pose",
            # This wrapper owns the RViz instance (openarmx_scenario.rviz),
            # never let ee_leader_marker_bimanual.launch.py spawn its own.
            "start_rviz": "false",
        }.items(),
    )

    # MoveIt-free mock hardware + JTCs + RSP. Required for cyclo vr_controller
    # to have /joint_states feedback and JTC sinks to publish into.
    cyclo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [motion_pkg, "launch", "openarmx_cyclo_sim.launch.py"])),
        condition=IfCondition(LaunchConfiguration("start_sim")),
    )

    # Cyclo single-arm MoveL controllers, one per arm (left + right). Each
    # subscribes to /openarmx/<side>/movel (robotis_interfaces/MoveL =
    # PoseStamped + Duration) and publishes JointTrajectory to the matching
    # JTC spawned by openarmx_cyclo_sim. UI's Cartesian Control tab picks
    # which side to command.
    #
    # NOTE: cyclo's vr_controller_node (bimanual 14-DOF integrated QP+CBF)
    # is intentionally NOT spawned -- the user is driving each arm
    # independently via its own omx_movel_controller_node.
    movel_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pick_pkg, "launch", "openarmx_movel_bimanual.launch.py"])),
        condition=IfCondition(LaunchConfiguration("start_movel_controllers")),
    )

    return LaunchDescription([
        *args, cyclo_sim, scenario, ee_leader, movel_controllers,
        camera_tf_bridge, rviz,
    ])
