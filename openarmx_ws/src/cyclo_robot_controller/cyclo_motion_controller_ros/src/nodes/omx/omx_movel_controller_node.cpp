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

#include "cyclo_motion_controller_ros/nodes/omx/omx_movel_controller_node.hpp"

#include <cmath>

#include "common/type_define.hpp"
#include "cyclo_motion_controller_ros/utils/controller_params.hpp"
#include "cyclo_motion_controller_ros/utils/pose_utils.hpp"
#include "cyclo_motion_controller_ros/utils/trajectory_utils.hpp"

namespace cyclo_motion_controller_ros
{
OmxMoveLControllerNode::OmxMoveLControllerNode()
: Node("omx_movel_controller"),
  joint_state_received_(false),
  commanded_state_initialized_(false),
  movel_target_initialized_(false),
  movel_trajectory_active_(false),
  motion_start_time_(this->now()),
  last_joint_state_time_(this->now()),
  active_motion_duration_(0.0),
  movel_start_pose_(Eigen::Affine3d::Identity()),
  movel_goal_pose_(Eigen::Affine3d::Identity())
{
  RCLCPP_INFO(this->get_logger(), "========================================");
  RCLCPP_INFO(this->get_logger(), "OMX MoveL Controller - Starting up...");
  RCLCPP_INFO(this->get_logger(), "Node name: %s", this->get_name());
  RCLCPP_INFO(this->get_logger(), "========================================");

  const auto p = controller_params::declareCommonControllerParams(
    this, std::string("/omx/joint_trajectory"), std::string("end_effector_link"));

  control_frequency_ = p.control_frequency;
  time_step_ = p.time_step;
  trajectory_time_ = p.trajectory_time;
  weight_damping_ = p.weight_damping;
  slack_penalty_ = p.slack_penalty;
  cbf_alpha_ = p.cbf_alpha;
  collision_buffer_ = p.collision_buffer;
  collision_safe_distance_ = p.collision_safe_distance;
  joint_state_timeout_ = p.joint_state_timeout;
  urdf_path_ = p.urdf_path;
  srdf_path_ = p.srdf_path;
  base_frame_ = p.base_frame;
  controlled_link_ = p.controlled_link;
  joint_states_topic_ = p.joint_states_topic;
  joint_command_topic_ = p.joint_command_topic;
  ee_pose_topic_ = p.ee_pose_topic;
  controller_error_topic_ = p.controller_error_topic;

  // MoveL-specific parameters
  kp_position_ = this->declare_parameter("kp_position", 4.0);
  kp_orientation_ = this->declare_parameter("kp_orientation", 2.5);
  weight_task_position_ = this->declare_parameter("weight_task_position", 10.0);
  weight_task_orientation_ = this->declare_parameter("weight_task_orientation", 1.0);
  movel_topic_ = this->declare_parameter("movel_topic", std::string("~/movel"));
  // [A — China 2026-06-06] true=MoveL 전체 궤적을 다중점 1개로 발행(JTC 정상 사용),
  // false=원본 ROBOTIS 방식(매 틱 단발 궤적 100Hz 스트리밍). HIL 자유낙하 회피 위해 기본 true.
  batch_trajectory_ = this->declare_parameter("batch_trajectory", true);
  // [B — China 2026-06-06] cyclo 는 매 틱(100Hz) 위치를 계산하는 위치 제어 컨트롤러다.
  // 이 매-틱 위치 명령은 트래젝토리 실행기(JTC)가 아니라 하드웨어 position 에 직접 가야 한다.
  // output_mode: "jtc"(JointTrajectory→JTC, 기존) | "forward"(Float64MultiArray→position
  // passthrough 컨트롤러, 매-틱 위치 직접 전달). forward 면 batch 무시하고 매-틱 스트리밍.
  output_mode_ = this->declare_parameter("output_mode", std::string("jtc"));
  forward_command_topic_ =
    this->declare_parameter("forward_command_topic", std::string("~/forward_command"));

  if (urdf_path_.empty()) {
    RCLCPP_FATAL(this->get_logger(), "URDF path not provided.");
    rclcpp::shutdown();
    return;
  }

  joint_command_pub_ =
    this->create_publisher<trajectory_msgs::msg::JointTrajectory>(joint_command_topic_, 10);
  forward_command_pub_ =
    this->create_publisher<std_msgs::msg::Float64MultiArray>(forward_command_topic_, 10);
  ee_pose_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseStamped>(ee_pose_topic_, 10);
  controller_error_pub_ =
    this->create_publisher<std_msgs::msg::String>(controller_error_topic_, 10);

  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            joint_states_topic_, 10,
            std::bind(&OmxMoveLControllerNode::jointStateCallback, this, std::placeholders::_1));
  movel_sub_ = this->create_subscription<openarmx_scenario_player_msgs::msg::MoveL>(
            movel_topic_, 10,
            std::bind(&OmxMoveLControllerNode::moveLCallback, this, std::placeholders::_1));

  try {
    RCLCPP_INFO(this->get_logger(), "URDF path: %s", urdf_path_.c_str());
    if (srdf_path_.empty()) {
      RCLCPP_INFO(this->get_logger(), "SRDF path not provided. Continuing without SRDF.");
    } else {
      RCLCPP_INFO(this->get_logger(), "SRDF path: %s", srdf_path_.c_str());
    }
    kinematics_solver_ =
      std::make_shared<cyclo_motion_controller::kinematics::KinematicsSolver>(urdf_path_,
        srdf_path_);
    qp_controller_ =
      std::make_shared<cyclo_motion_controller::controllers::OpenManipulatorMoveLController>(
      kinematics_solver_, controlled_link_, time_step_);
    qp_controller_->setControllerParams(
                slack_penalty_, cbf_alpha_, collision_buffer_, collision_safe_distance_);

    q_.setZero(kinematics_solver_->getDof());
    qdot_.setZero(kinematics_solver_->getDof());
    q_commanded_.setZero(kinematics_solver_->getDof());

    initializeJointConfig();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(this->get_logger(), "Failed to initialize OMX MoveL Controller: %s", e.what());
    rclcpp::shutdown();
    return;
  }

  const int timer_period_ms =
    std::max(1, static_cast<int>(std::round(1000.0 / std::max(1.0, control_frequency_))));
  control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(timer_period_ms),
            std::bind(&OmxMoveLControllerNode::controlLoopCallback, this));

  if (!control_timer_) {
    RCLCPP_FATAL(this->get_logger(), "Failed to create control loop timer!");
    rclcpp::shutdown();
    return;
  }

  RCLCPP_INFO(this->get_logger(), "OMX MoveL Controller initialized successfully!");
}

OmxMoveLControllerNode::~OmxMoveLControllerNode()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down OMX MoveL Controller");
}

void OmxMoveLControllerNode::initializeJointConfig()
{
  model_joint_names_ = kinematics_solver_->getJointNames();
  model_joint_index_map_.clear();
  for (size_t i = 0; i < model_joint_names_.size(); ++i) {
    model_joint_index_map_[model_joint_names_[i]] = static_cast<int>(i);
  }

  std::string joint_list;
  for (const auto & joint_name : model_joint_names_) {
    joint_list += joint_name + " ";
  }
  RCLCPP_INFO(this->get_logger(), "Model joints: %s", joint_list.c_str());
}

void OmxMoveLControllerNode::extractJointStates(const sensor_msgs::msg::JointState::SharedPtr & msg)
{
  const int dof = kinematics_solver_->getDof();
  q_.setZero(dof);
  qdot_.setZero(dof);

  const int max_index = std::min<int>(dof, static_cast<int>(model_joint_names_.size()));
  for (int i = 0; i < max_index; ++i) {
    const auto & joint_name = model_joint_names_[i];
    const auto it = joint_index_map_.find(joint_name);
    if (it == joint_index_map_.end()) {
      continue;
    }
    const int msg_idx = it->second;
    if (msg_idx < static_cast<int>(msg->position.size())) {
      q_[i] = msg->position[msg_idx];
    }
    if (msg_idx < static_cast<int>(msg->velocity.size())) {
      qdot_[i] = msg->velocity[msg_idx];
    }
  }
}

void OmxMoveLControllerNode::publishCurrentPose(const Eigen::Affine3d & pose) const
{
  pose_utils::publishPoseStamped(ee_pose_pub_, base_frame_, this->now(), pose);
}

void OmxMoveLControllerNode::publishTrajectory(const Eigen::VectorXd & q_command) const
{
  if (output_mode_ == "forward") {
    // [B] 매-틱 위치를 position passthrough 컨트롤러로 직접 전달(재계획·궤적해석 없음).
    std_msgs::msg::Float64MultiArray msg;
    msg.data.assign(q_command.data(), q_command.data() + q_command.size());
    forward_command_pub_->publish(msg);
  } else {
    joint_command_pub_->publish(
      trajectory_utils::makeJointTrajectoryMsg(model_joint_names_, trajectory_time_, q_command));
  }
}

void OmxMoveLControllerNode::publishControllerError(const std::string & error) const
{
  pose_utils::publishStringMsg(controller_error_pub_, error);
}

void OmxMoveLControllerNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (joint_index_map_.empty()) {
    for (size_t i = 0; i < msg->name.size(); ++i) {
      joint_index_map_[msg->name[i]] = static_cast<int>(i);
    }
  }

  extractJointStates(msg);
  last_joint_state_time_ = this->now();
  joint_state_received_ = true;

  const bool was_uninitialized = !commanded_state_initialized_;
  const bool recovering_from_timeout = joint_state_timeout_active_;
  joint_state_timeout_active_ = false;

  if (was_uninitialized || recovering_from_timeout) {
    syncCommandStateToFeedback();
    commanded_state_initialized_ = true;
    movel_target_initialized_ = true;
  }
}

void OmxMoveLControllerNode::moveLCallback(const openarmx_scenario_player_msgs::msg::MoveL::SharedPtr msg)
{
  if (!msg || !joint_state_received_ || jointStateTimedOut()) {
    RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Ignoring moveL command until joint states are available.");
    return;
  }

  const double requested_duration = commandDurationSeconds(msg->time_from_start);

  syncCommandStateToFeedback();
  kinematics_solver_->updateState(q_commanded_, qdot_);
  movel_start_pose_ = kinematics_solver_->getPose(controlled_link_);
  movel_goal_pose_ = pose_utils::poseMsgToEigen(msg->pose);
  active_motion_duration_ = requested_duration;
  motion_start_time_ = this->now();
  movel_target_initialized_ = true;
  movel_trajectory_active_ = requested_duration > -1.0;

  // [A — China] batch 모드(output_mode=jtc 일 때만): 전체 궤적을 endpoint 1점으로 미리
  // 발행하고 스트리밍을 끈다. forward 모드(B)는 매-틱 스트리밍이 목적이므로 batch 건너뜀.
  if (batch_trajectory_ && output_mode_ == "jtc" && movel_trajectory_active_) {
    precomputeAndPublishTrajectory();
  }
}

cyclo_motion_controller::common::Vector6d OmxMoveLControllerNode::computeDesiredVelocity(
  const Eigen::Affine3d & current_pose,
  const Eigen::Affine3d & goal_pose,
  const Eigen::Vector3d & feedforward_linear,
  const Eigen::Vector3d & feedforward_angular) const
{
  return pose_utils::computeDesiredVelocity(
    current_pose, goal_pose, kp_position_, kp_orientation_,
    feedforward_linear, feedforward_angular);
}

void OmxMoveLControllerNode::precomputeAndPublishTrajectory()
{
  namespace math_utils = cyclo_motion_controller::common::math_utils;
  const int dof = kinematics_solver_->getDof();
  const double dt = std::max(1e-6, time_step_);
  const double duration = active_motion_duration_;
  const int steps = std::max(1, static_cast<int>(std::ceil(duration / dt)));

  // 시작 = 동기화된 명령상태(=측정값). control loop가 open-loop(q_feedback=q_commanded_)라
  // 전체 궤적을 결정적으로 미리 적분할 수 있다(매-틱 스트리밍과 동일한 q 시퀀스).
  Eigen::VectorXd q = q_commanded_;

  // 가중치 (controlLoopCallback과 동일)
  cyclo_motion_controller::common::Vector6d task_weight =
    cyclo_motion_controller::common::Vector6d::Zero();
  task_weight.head<3>().setConstant(weight_task_position_);
  task_weight.tail<3>().setConstant(weight_task_orientation_);
  const Eigen::VectorXd damping_weight = Eigen::VectorXd::Ones(dof) * weight_damping_;

  // KinematicsSolver에 역기구학(IK)이 없으므로, control loop와 동일한 cubic+QP를
  // open-loop로 목표까지 적분해 endpoint(목표 관절각)만 구한다(경로점은 버린다).
  for (int k = 1; k <= steps; ++k) {
    const double elapsed = std::min(static_cast<double>(k) * dt, duration);

    kinematics_solver_->updateState(q, qdot_);
    const Eigen::Affine3d current_pose = kinematics_solver_->getPose(controlled_link_);

    const Eigen::Vector3d linear_ref = math_utils::cubicDotVector<3>(
      elapsed, 0.0, duration,
      movel_start_pose_.translation(), movel_goal_pose_.translation(),
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    const Eigen::Vector3d position_ref = math_utils::cubicVector<3>(
      elapsed, 0.0, duration,
      movel_start_pose_.translation(), movel_goal_pose_.translation(),
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    const Eigen::Matrix3d rotation_ref = math_utils::rotationCubic(
      elapsed, 0.0, duration,
      movel_start_pose_.linear(), movel_goal_pose_.linear());
    const Eigen::Vector3d angular_ref = math_utils::rotationCubicDot(
      elapsed, 0.0, duration,
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      movel_start_pose_.linear(), movel_goal_pose_.linear());

    Eigen::Affine3d pose_ref = Eigen::Affine3d::Identity();
    pose_ref.translation() = position_ref;
    pose_ref.linear() = rotation_ref;

    const cyclo_motion_controller::common::Vector6d desired_task_vel =
      computeDesiredVelocity(current_pose, pose_ref, linear_ref, angular_ref);

    qp_controller_->setDesiredTaskVel(desired_task_vel);
    qp_controller_->setWeights(task_weight, damping_weight);

    Eigen::VectorXd optimal_velocities;
    if (!qp_controller_->getOptJointVel(optimal_velocities)) {
      publishControllerError("OMX MoveL Controller: QP solve failed (endpoint precompute)");
      RCLCPP_WARN(
                this->get_logger(),
                "Endpoint precompute QP failed at step %d/%d; aborting.", k, steps);
      movel_trajectory_active_ = false;
      return;
    }

    q = q + optimal_velocities * dt;
  }

  // endpoint 1점만 발행 — JTC가 현재위치→endpoint를 duration에 걸쳐 보간(스트리밍 없음).
  joint_command_pub_->publish(
    trajectory_utils::makeJointTrajectoryMsg(model_joint_names_, duration, q));

  // 최종 명령상태로 갱신하고 스트리밍 비활성화 → control loop는 else 분기에서 return(양보).
  q_commanded_ = q;
  movel_trajectory_active_ = false;

  RCLCPP_INFO(
            this->get_logger(),
            "MoveL endpoint published as single point (duration %.2fs, no 100Hz streaming).",
            duration);
}

void OmxMoveLControllerNode::controlLoopCallback()
{
  if (!joint_state_received_ || !commanded_state_initialized_ || !movel_target_initialized_) {
    RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Control loop waiting for joint states...");
    return;
  }

  if (jointStateTimedOut()) {
    if (!joint_state_timeout_active_) {
      joint_state_timeout_active_ = true;
      movel_trajectory_active_ = false;
      RCLCPP_WARN(
        this->get_logger(),
        "Joint states timed out. Holding commands until fresh feedback is received.");
    }
    return;
  }

  try {
    // [open-loop 복원 — 2026-06-06] 제어루프는 q_commanded_(자체 적분 명령상태)를 피드백으로
    // 쓴다(ROBOTIS 원본 방식). 큐빅 궤적을 q_commanded_가 적분해 목표에 도달 후 멈추므로,
    // 하드웨어 지연이 있어도 명령이 과도하게 램프하지 않는다. (closed-loop(q_feedback=q_)는
    // 지연된 실제값 기준이라 오차가 안 닫혀 over-swing/방향오류 발생 — 하드웨어 검증.
    // 앞서 의심한 "개루프 발산"의 진짜 원인은 UI dual-publish로 인한 joint_index_map 손상이었고
    // 그건 별도 수정됨.) MoveL 콜백의 sync가 매 명령 시작 시 q_commanded_=q_(실측)로 재기준화.
    const Eigen::VectorXd q_feedback = q_commanded_;
    kinematics_solver_->updateState(q_feedback, qdot_);
    const Eigen::Affine3d current_pose = kinematics_solver_->getPose(controlled_link_);
    publishCurrentPose(current_pose);

    const double elapsed = (this->now() - motion_start_time_).seconds();
    cyclo_motion_controller::common::Vector6d desired_task_vel =
      cyclo_motion_controller::common::Vector6d::Zero();

    if (movel_trajectory_active_ && elapsed < active_motion_duration_) {
      const Eigen::Vector3d linear_ref =
        cyclo_motion_controller::common::math_utils::cubicDotVector<3>(
                        elapsed,
                        0.0,
                        active_motion_duration_,
                        movel_start_pose_.translation(),
                        movel_goal_pose_.translation(),
                        Eigen::Vector3d::Zero(),
                        Eigen::Vector3d::Zero());
      const Eigen::Vector3d position_ref =
        cyclo_motion_controller::common::math_utils::cubicVector<3>(
                        elapsed,
                        0.0,
                        active_motion_duration_,
                        movel_start_pose_.translation(),
                        movel_goal_pose_.translation(),
                        Eigen::Vector3d::Zero(),
                        Eigen::Vector3d::Zero());
      const Eigen::Matrix3d rotation_ref =
        cyclo_motion_controller::common::math_utils::rotationCubic(
                        elapsed,
                        0.0,
                        active_motion_duration_,
                        movel_start_pose_.linear(),
                        movel_goal_pose_.linear());
      const Eigen::Vector3d angular_ref =
        cyclo_motion_controller::common::math_utils::rotationCubicDot(
                        elapsed,
                        0.0,
                        active_motion_duration_,
                        Eigen::Vector3d::Zero(),
                        Eigen::Vector3d::Zero(),
                        movel_start_pose_.linear(),
                        movel_goal_pose_.linear());

      Eigen::Affine3d pose_ref = Eigen::Affine3d::Identity();
      pose_ref.translation() = position_ref;
      pose_ref.linear() = rotation_ref;

      desired_task_vel =
        computeDesiredVelocity(current_pose, pose_ref, linear_ref, angular_ref);
    } else {
      if (movel_trajectory_active_) {
        movel_trajectory_active_ = false;
      }
      // [China 적응 — 유지 필수] 큐빅 종료 후 cyclo는 발행을 멈추고 양보한다(quiescent).
      // ROBOTIS 원본은 여기서 movel_goal_pose_로 계속 목표추종(kp*error)하지만, 그 거동을
      // China에 가져오면 cyclo가 idle에도 JTC로 마지막 목표를 계속 당겨 init/joint 등
      // 다른 컨트롤러 명령을 덮어쓴다(검증: 2026-06-06 하드웨어에서 'init 위치로 안감').
      // China는 cyclo·pilz·joint_control이 동일 JTC를 공유하는 멀티-컨트롤러 구조라
      // idle 시 양보(return)가 필수다. 원본 거동으로 되돌리지 말 것.
      return;
    }

    cyclo_motion_controller::common::Vector6d task_weight =
      cyclo_motion_controller::common::Vector6d::Zero();
    task_weight.head<3>().setConstant(weight_task_position_);
    task_weight.tail<3>().setConstant(weight_task_orientation_);
    const Eigen::VectorXd damping_weight =
      Eigen::VectorXd::Ones(kinematics_solver_->getDof()) * weight_damping_;

    qp_controller_->setDesiredTaskVel(desired_task_vel);
    qp_controller_->setWeights(task_weight, damping_weight);

    Eigen::VectorXd optimal_velocities;
    if (!qp_controller_->getOptJointVel(optimal_velocities)) {
      publishControllerError("OMX MoveL Controller: QP solve failed");
      RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "OMX MoveL Controller QP solver failed");
      return;
    }

    q_commanded_ = q_feedback + optimal_velocities * time_step_;
    publishTrajectory(q_commanded_);
  } catch (const std::exception & e) {
    publishControllerError("OMX MoveL Controller loop error: " + std::string(e.what()));
    RCLCPP_ERROR(this->get_logger(), "OMX MoveL Controller loop error: %s", e.what());
  }
}

bool OmxMoveLControllerNode::jointStateTimedOut() const
{
  return joint_state_received_ &&
         (this->now() - last_joint_state_time_).seconds() > joint_state_timeout_;
}

void OmxMoveLControllerNode::syncCommandStateToFeedback()
{
  q_commanded_ = q_;
  kinematics_solver_->updateState(q_commanded_, qdot_);
  movel_start_pose_ = kinematics_solver_->getPose(controlled_link_);
  movel_goal_pose_ = movel_start_pose_;
  movel_trajectory_active_ = false;
}
}  // namespace cyclo_motion_controller_ros

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cyclo_motion_controller_ros::OmxMoveLControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
