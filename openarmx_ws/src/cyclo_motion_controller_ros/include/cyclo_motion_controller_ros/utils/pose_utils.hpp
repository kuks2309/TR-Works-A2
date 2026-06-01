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

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "common/type_define.hpp"

namespace cyclo_motion_controller_ros
{
namespace pose_utils
{

/// Affine3d → PoseStamped 변환 후 publish. publisher가 nullptr이면 no-op.
inline void publishPoseStamped(
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub,
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  const Eigen::Affine3d & pose)
{
  if (!pub) {
    return;
  }

  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = stamp;
  pose_msg.header.frame_id = frame_id;
  pose_msg.pose.position.x = pose.translation().x();
  pose_msg.pose.position.y = pose.translation().y();
  pose_msg.pose.position.z = pose.translation().z();

  const Eigen::Quaterniond quat(pose.linear());
  pose_msg.pose.orientation.w = quat.w();
  pose_msg.pose.orientation.x = quat.x();
  pose_msg.pose.orientation.y = quat.y();
  pose_msg.pose.orientation.z = quat.z();
  pub->publish(pose_msg);
}

/// std_msgs::String publish. publisher가 nullptr이면 no-op.
inline void publishStringMsg(
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub,
  const std::string & data)
{
  if (!pub) {
    return;
  }
  std_msgs::msg::String msg;
  msg.data = data;
  pub->publish(msg);
}

/// geometry_msgs::PoseStamped → Eigen::Affine3d 변환.
inline Eigen::Affine3d poseMsgToEigen(const geometry_msgs::msg::PoseStamped & pose_msg)
{
  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  pose.translation() << pose_msg.pose.position.x,
    pose_msg.pose.position.y,
    pose_msg.pose.position.z;

  const Eigen::Quaterniond quat(
    pose_msg.pose.orientation.w,
    pose_msg.pose.orientation.x,
    pose_msg.pose.orientation.y,
    pose_msg.pose.orientation.z);
  pose.linear() = quat.normalized().toRotationMatrix();
  return pose;
}

/// 현재 pose에서 목표 pose로의 desired Cartesian velocity (6D) 계산.
/// kp_position, kp_orientation 게인을 인자로 받음.
inline cyclo_motion_controller::common::Vector6d computeDesiredVelocity(
  const Eigen::Affine3d & current_pose,
  const Eigen::Affine3d & goal_pose,
  double kp_position,
  double kp_orientation,
  const Eigen::Vector3d & feedforward_linear = Eigen::Vector3d::Zero(),
  const Eigen::Vector3d & feedforward_angular = Eigen::Vector3d::Zero())
{
  cyclo_motion_controller::common::Vector6d desired_vel =
    cyclo_motion_controller::common::Vector6d::Zero();

  const Eigen::Vector3d position_error = goal_pose.translation() - current_pose.translation();
  const Eigen::Matrix3d rotation_error = goal_pose.linear() * current_pose.linear().transpose();
  const Eigen::AngleAxisd angle_axis_error(rotation_error);
  const Eigen::Vector3d orientation_error =
    angle_axis_error.axis() * angle_axis_error.angle();

  desired_vel.head<3>() = feedforward_linear + kp_position * position_error;
  desired_vel.tail<3>() = feedforward_angular + kp_orientation * orientation_error;
  return desired_vel;
}

}  // namespace pose_utils
}  // namespace cyclo_motion_controller_ros
