"""ptp pick&place 정본 스택 launch.

검출 루프 + 컨테이너 게이트 + 좌/우 상주 픽 서버를 한 번에 띄운다.
구 experiments/ 절대경로 bash 그룹(_pick_srv_cmd)을 대체 — UI 는 이 launch 를 띄운다.

주의: box_detect_loop 은 yolov8_detection_msgs(별도 워크스페이스 3d_detect_ws)를 쓰므로
이 launch 를 띄우는 환경에서 3d_detect_ws/install/setup.bash 가 먼저 source 돼 있어야 한다.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 검출/이동 분리: box_detect_loop 가 /detected_boxes 를 갱신, 좌/우 resident 가 소비.
        # 반드시 resident 보다 먼저 떠야 첫 픽부터 최신 박스가 있다(launch 는 동시 기동이라
        # resident 의 박스-대기 로직이 흡수).
        Node(package='openarmx_ptp_pick', executable='box_detect_loop',
             name='box_detect_loop', output='screen'),
        Node(package='openarmx_ptp_pick', executable='container_pick_gate',
             name='container_pick_gate', output='screen',
             parameters=[{'gated': True}]),
        Node(package='openarmx_ptp_pick', executable='ptp_pick_resident',
             name='ptp_pick_left', output='screen', arguments=['--side=left']),
        Node(package='openarmx_ptp_pick', executable='ptp_pick_resident',
             name='ptp_pick_right', output='screen', arguments=['--side=right']),
    ])
