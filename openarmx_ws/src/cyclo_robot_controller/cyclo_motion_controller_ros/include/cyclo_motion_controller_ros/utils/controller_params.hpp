// Copyright 2026 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Yeonguk Kim

#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>

namespace cyclo_motion_controller_ros
{
namespace controller_params
{

/// 6 controllers 공통 파라미터 구조체.
/// MoveL 전용 (kp_position, kp_orientation, weight_task_position, weight_task_orientation),
/// MoveJ 전용 (kp_joint, weight_joint_tracking) 은 각 controller에서 별도 declare.
struct CommonControllerParams
{
  double control_frequency;
  double time_step;
  double trajectory_time;
  double weight_damping;
  double slack_penalty;
  double cbf_alpha;
  double collision_buffer;
  double collision_safe_distance;
  double joint_state_timeout;

  std::string urdf_path;
  std::string srdf_path;
  std::string base_frame;
  std::string controlled_link;
  std::string joint_states_topic;
  std::string joint_command_topic;
  std::string ee_pose_topic;
  std::string controller_error_topic;
};

/// 공통 파라미터 declare + 값 반환.
/// default_joint_command_topic: 각 controller마다 다름 (예: "/omx/joint_trajectory").
/// default_controlled_link: 각 controller마다 다름 (예: "end_effector_link").
inline CommonControllerParams declareCommonControllerParams(
  rclcpp::Node * node,
  const std::string & default_joint_command_topic,
  const std::string & default_controlled_link)
{
  CommonControllerParams p;
  p.control_frequency = node->declare_parameter("control_frequency", 100.0);
  p.time_step = node->declare_parameter("time_step", 0.01);
  p.trajectory_time = node->declare_parameter("trajectory_time", 0.05);
  p.weight_damping = node->declare_parameter("weight_damping", 0.05);
  p.slack_penalty = node->declare_parameter("slack_penalty", 1000.0);
  p.cbf_alpha = node->declare_parameter("cbf_alpha", 5.0);
  p.collision_buffer = node->declare_parameter("collision_buffer", 0.05);
  p.collision_safe_distance = node->declare_parameter("collision_safe_distance", 0.02);
  p.joint_state_timeout = node->declare_parameter("joint_state_timeout", 0.5);

  p.urdf_path = node->declare_parameter("urdf_path", std::string(""));
  p.srdf_path = node->declare_parameter("srdf_path", std::string(""));
  p.base_frame = node->declare_parameter("base_frame", std::string("link0"));
  p.controlled_link = node->declare_parameter("controlled_link", default_controlled_link);
  p.joint_states_topic = node->declare_parameter(
    "joint_states_topic", std::string("/joint_states"));
  p.joint_command_topic = node->declare_parameter(
    "joint_command_topic", default_joint_command_topic);
  p.ee_pose_topic = node->declare_parameter("ee_pose_topic", std::string("~/current_pose"));
  p.controller_error_topic = node->declare_parameter(
    "controller_error_topic", std::string("~/controller_error"));
  return p;
}

}  // namespace controller_params
}  // namespace cyclo_motion_controller_ros
