"""Launch the TOF USB-serial transport driver.

ESP32-C3(VL53L0X) ToF2CAN board over USB-CDC (/dev/ttyACM0) -> sensor_msgs/Range
on /tof/range. Hardware-layer sensor package; consumers (e.g. place_box_detection
wall gate) subscribe to /tof/range and do not care how it is produced.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("openarmx_tof_driver")
    params = os.path.join(pkg, "config", "tof_driver.yaml")

    return LaunchDescription([
        Node(
            package="openarmx_tof_driver",
            executable="tof_serial_driver_node",
            name="tof_serial_driver",
            output="screen",
            parameters=[params],
        ),
    ])
