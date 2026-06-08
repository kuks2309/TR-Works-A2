#!/usr/bin/env python3
"""D435 ROS 콜드 스타트 측정 — 실제 "카메라 Start" 경험.

d435_camera.launch.py(realsense2_camera_node + 정적 TF 브리지)를 띄우는 순간부터
/camera/camera/color/image_raw 첫 메시지가 도착할 때까지의 벽시계 시간.
ros2 launch 프로세스 생성 + 노드 init + 파라미터 로드 + SDK init + 첫 발행 포함.

측정 후 launch 는 detach 되어 계속 실행(스택 카메라 복구).
"""
from __future__ import annotations

import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

LAUNCH = ["ros2", "launch", "openarmx_scenario_player", "d435_camera.launch.py"]
COLOR = "/camera/camera/color/image_raw"
DEPTH = "/camera/camera/aligned_depth_to_color/image_raw"


class Meter(Node):
    def __init__(self):
        super().__init__("d435_ros_startup_meter")
        self.t_color = None
        self.t_depth = None
        self.create_subscription(Image, COLOR, self._color, qos_profile_sensor_data)
        self.create_subscription(Image, DEPTH, self._depth, qos_profile_sensor_data)

    def _color(self, _m):
        if self.t_color is None:
            self.t_color = time.monotonic()

    def _depth(self, _m):
        if self.t_depth is None:
            self.t_depth = time.monotonic()


def main():
    rclpy.init()
    node = Meter()
    t0 = time.monotonic()
    proc = subprocess.Popen(LAUNCH, stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL, start_new_session=True)
    print(f"launched d435_camera.launch.py (pid {proc.pid}); waiting for first frames...")
    while rclpy.ok() and (node.t_color is None or node.t_depth is None):
        if time.monotonic() - t0 > 40:
            break
        rclpy.spin_once(node, timeout_sec=0.05)
    if node.t_color:
        print(f"launch -> first COLOR frame : {(node.t_color - t0) * 1e3:.0f} ms")
    else:
        print("no color frame within 40s")
    if node.t_depth:
        print(f"launch -> first DEPTH(aligned) frame : {(node.t_depth - t0) * 1e3:.0f} ms")
    node.destroy_node()
    rclpy.shutdown()
    print("(launch left running to restore the stack camera)")


if __name__ == "__main__":
    main()
