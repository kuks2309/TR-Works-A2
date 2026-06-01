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
#include <vector>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace cyclo_motion_controller_ros
{
namespace trajectory_utils
{

/// VectorXd → JointTrajectory 변환 (simple 1-point, positions+velocities=0).
/// omx_movel / omy_movel 공통 publishTrajectory 패턴에 사용.
inline trajectory_msgs::msg::JointTrajectory makeJointTrajectoryMsg(
  const std::vector<std::string> & joint_names,
  double duration_sec,
  const Eigen::VectorXd & positions)
{
  trajectory_msgs::msg::JointTrajectory traj_msg;
  traj_msg.header.frame_id = "";
  traj_msg.joint_names = joint_names;

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.time_from_start = rclcpp::Duration::from_seconds(duration_sec);
  for (int idx = 0; idx < positions.size(); ++idx) {
    point.positions.push_back(positions[idx]);
    point.velocities.push_back(0.0);
  }

  traj_msg.points.push_back(point);
  return traj_msg;
}

}  // namespace trajectory_utils
}  // namespace cyclo_motion_controller_ros
