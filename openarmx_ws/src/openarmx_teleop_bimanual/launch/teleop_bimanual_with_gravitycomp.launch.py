#!/usr/bin/env python3
# -*- coding: utf-8 -*-

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


from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 右臂：can0, g=0.9, gdir=0,-9.81,0
        Node(
            package='openarmx_teleop_bimanual',
            executable='teleop_bimanual_with_gravitycomp_single',
            name='teleop_bimanual_with_gravitycomp_right',
            output='screen',
            parameters=[{
                'arm_side': 'right_arm',
                'leader_can': 'can0',
                'leader_urdf_path': '/tmp/v10_bimanual.urdf',
                'follower_prefix': 'right',
                'control_rate_hz': 300,
                'g_scale': 0.9,
                'kd_damp': 0.0,
                'kp_hold': 0.0,
                'vel_hold_thresh': 0.02,
                'hold_settle_ms': 300,
                'gdir': [0.0, -9.81, 0.0],
                'verbose': False,
            }]
        ),

        # 左臂：can1, g=0.8, gdir=0,9.81,0
        Node(
            package='openarmx_teleop_bimanual',
            executable='teleop_bimanual_with_gravitycomp_single',
            name='teleop_bimanual_with_gravitycomp_left',
            output='screen',
            parameters=[{
                'arm_side': 'left_arm',
                'leader_can': 'can1',
                'leader_urdf_path': '/tmp/v10_bimanual.urdf',
                'follower_prefix': 'left',
                'control_rate_hz': 300,
                'g_scale': 0.8,
                'kd_damp': 0.0,
                'kp_hold': 0.0,
                'vel_hold_thresh': 0.02,
                'hold_settle_ms': 300,
                'gdir': [0.0, 9.81, 0.0],
                'verbose': False,
            }]
        ),
    ])
