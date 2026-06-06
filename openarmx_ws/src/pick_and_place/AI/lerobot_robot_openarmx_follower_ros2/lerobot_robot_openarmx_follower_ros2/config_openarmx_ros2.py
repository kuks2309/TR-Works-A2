# Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International
#
# Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd.
# https://www.openarmx.com
#
# This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike
# 4.0 International License (CC BY-NC-SA 4.0).
#
# To view a copy of this license, visit:
# http://creativecommons.org/licenses/by-nc-sa/4.0/
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.

from __future__ import annotations

from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig
from lerobot.cameras.configs import ColorMode, Cv2Rotation
from lerobot.robots import RobotConfig

from .ros2_camera import Ros2CameraConfig


@dataclass
class OpenArmXRos2InterfaceConfig:
    """ROS 2 wiring for OpenArmX.

    OpenArmX uses ros2_control forward_command_controller topics like:
    - /left_forward_position_controller/commands
    - /right_forward_position_controller/commands

    and publishes joint states on /joint_states.
    """

    namespace: str = ""  # Optional ROS namespace

    joint_states_topic: str = "/joint_states"

    left_command_topic: str = "/left_forward_position_controller/commands"
    right_command_topic: str = "/right_forward_position_controller/commands"

    # Joint ordering for commands (must match controller 'joints:' order)
    left_joint_names: list[str] = field(
        default_factory=lambda: [
            "openarmx_left_joint1",
            "openarmx_left_joint2",
            "openarmx_left_joint3",
            "openarmx_left_joint4",
            "openarmx_left_joint5",
            "openarmx_left_joint6",
            "openarmx_left_joint7",
            "openarmx_left_finger_joint1",
        ]
    )
    right_joint_names: list[str] = field(
        default_factory=lambda: [
            "openarmx_right_joint1",
            "openarmx_right_joint2",
            "openarmx_right_joint3",
            "openarmx_right_joint4",
            "openarmx_right_joint5",
            "openarmx_right_joint6",
            "openarmx_right_joint7",
            "openarmx_right_finger_joint1",
        ]
    )


@RobotConfig.register_subclass("openarmx_follower_ros2")
@dataclass
class OpenArmXRos2Config(RobotConfig):
    """LeRobot RobotConfig for OpenArmX via ROS 2 topics."""

    # Safety: clip per-joint relative steps (radians). If None, no clipping.
    max_relative_target: float | dict[str, float] | None = None

    # Skip send_action (for direct teleop mode where teleop_node controls robot directly)
    # When True, lerobot only records data without sending commands to robot
    skip_send_action: bool = True

    # cameras - 단일 중앙(헤드) D435 카메라 구성 (China 워크스페이스 적응)
    #
    # 원본 openarmx_vla 는 손목 D405 2대 + 헤드 D435 1대(/cam_*/...)를 가정하나,
    # 본 프로젝트는 현재 중앙(d435_center_link) D435 1대만 존재한다.
    # 따라서 기존 d435_camera.launch.py 가 발행하는 realsense 기본 토픽
    #   /camera/camera/color/image_raw           (컬러)
    #   /camera/camera/aligned_depth_to_color/image_raw (깊이, 미사용)
    # 을 직접 구독한다. (cam_head 1개만 사용)
    #
    # 주의(README 핵심 제약): width/height/fps 는 실제 d435 스트림과 반드시 일치해야
    # 한다. 불일치 시 lerobot 데이터셋 shape 오류. 손목 D405 추가 시 항목만 더하고
    # 그 데이터로 재학습하면 된다.
    cameras: dict[str, CameraConfig] = field(
        default_factory=lambda: {
            "cam_head": Ros2CameraConfig(
                image_topic="/camera/camera/color/image_raw",
                depth_topic="/camera/camera/aligned_depth_to_color/image_raw",
                fps=30,
                width=640,
                height=480,
                color_mode=ColorMode.RGB,
                use_depth=False,  # depth 는 get_observation 에서 미기록 → 구독 끔
                rotation=Cv2Rotation.NO_ROTATION,
                qos_reliability="best_effort",
                queue_size=1,
            ),
        }
    )

    ros2: OpenArmXRos2InterfaceConfig = field(default_factory=OpenArmXRos2InterfaceConfig)
