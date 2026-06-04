"""D435 카메라 + base TF 브리지 — Pick&Place 카메라 전용 독립 launch.

카메라 구동(Pick and Place 탭 "카메라 Start") 시 사용하는 전용 launch.
realsense2_camera(컬러 + 깊이 + depth→color 정렬 + 포인트클라우드)와 함께
``d435_center_link → camera_link`` 정적 TF 브리지를 한 그룹으로 띄운다.

이 브리지가 없으면 외부 ``/camera/camera`` RGBD/포인트클라우드가
``openarmx_body_link0`` 으로 풀리지 않아 RViz 에 클라우드/박스 TF 가
렌더되지 않는다(= 검출 결과가 RViz 에 안 보임의 근본 원인).

기존 ``scenario_player_with_ee_leader.launch.py`` 의 브리지를 재사용하지 않고,
카메라 기동에 묶인 독립 launch 로 분리한다 → 카메라 Start 시에만 브리지가
함께 뜨고, 카메라 Stop(프로세스그룹 종료) 시 브리지도 함께 내려간다.

체인: openarmx_body_link0 →(정적 extrinsic, RSP)→ d435_center_link
      →(본 브리지, identity)→ camera_link →(realsense)→ *_optical_frame
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 센서 스트림 QoS(pointcloud/depth/color = SENSOR_DATA=BEST_EFFORT) 를 노드에
    # 주입. rs_launch.py 는 qos 를 launch 인자로 노출하지 않으므로(--show-args 0건),
    # config_file 메커니즘(parameters=[params, params_from_file])으로 전달한다.
    qos_config = PathJoinSubstitution(
        [FindPackageShare("openarmx_scenario_player"), "config", "d435_qos.yaml"])

    # D435 RealSense — Pick&Place 탭 D435_CMD 와 동일 인자(컬러/깊이/정렬/클라우드)
    # + QoS config(BEST_EFFORT).
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"])),
        launch_arguments={
            "enable_color": "true",
            "enable_depth": "true",
            "align_depth.enable": "true",
            "pointcloud.enable": "true",
            "pointcloud.stream_filter": "2",
            "config_file": qos_config,
        }.items(),
    )

    # d435_center_link(URDF/extrinsic, RSP 발행) → camera_link(realsense 라이브
    # 프레임 트리). identity — d435_center_link 가 곧 캘리브된 camera_link 위치.
    # 이 엣지가 있어야 /camera/camera 클라우드가 base 로 풀려 RViz 에 렌더된다.
    camera_tf_bridge = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="d435_center_to_camera_link_tf",
        arguments=["--x", "0", "--y", "0", "--z", "0",
                   "--roll", "0", "--pitch", "0", "--yaw", "0",
                   "--frame-id", "d435_center_link", "--child-frame-id", "camera_link"],
        output="log",
    )

    return LaunchDescription([realsense, camera_tf_bridge])
