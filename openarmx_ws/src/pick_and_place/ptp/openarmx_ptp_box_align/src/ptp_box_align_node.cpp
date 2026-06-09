// Copyright 2026 openarmx
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Bimanual box-align action server using a DIRECT point-to-point (PTP) move:
// a single Pinocchio damped-least-squares inverse kinematics (IK) solve for the
// link7 goal pose, published as one JointTrajectory endpoint to the arm's
// joint_trajectory_controller (JTC). No QP, no Control-Barrier-Function safety,
// no MoveIt -- the lightest of the three backends (cyclo / pilz / ptp).
//
// Perception is a SEPARATE node: this server only consumes the latest
// /detected_boxes (geometry_msgs/PoseArray, base frame). The IK target frame is
// openarmx_<side>_<controlled_link_suffix>, default "hand_tcp" so the goal z is
// the TOOL-CENTER-POINT (TCP) height (like cyclo); set the suffix to "link7" to
// constrain link7 directly (like pilz). Pinocchio solves IK for any frame, so no
// manual link7->hand_tcp offset is needed either way.
//
// The reduced single-arm solver URDFs (openarmx_<side>_solver.urdf, reused from
// openarmx_pick) are loaded into Pinocchio: every joint except that arm's 7
// revolute joints is fixed, so model.nq == 7 per side.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <cerrno>
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/spatial/explog.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <openarmx_ptp_box_align_msgs/action/align_to_boxes.hpp>

namespace
{
constexpr char kBase[] = "openarmx_body_link0";

// Roll/pitch/yaw in DEGREES -> rotation matrix (ZYX intrinsic, matching the
// rpy_to_quat used by the cyclo/pilz Python backends).
Eigen::Matrix3d rpyDegToRot(double roll, double pitch, double yaw)
{
  const double r = roll * M_PI / 180.0;
  const double p = pitch * M_PI / 180.0;
  const double y = yaw * M_PI / 180.0;
  return (Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX()))
    .toRotationMatrix();
}
}  // namespace

// One arm's Pinocchio model + the joint metadata needed to emit a JointTrajectory.
struct ArmModel
{
  pinocchio::Model model;
  pinocchio::Data data;
  pinocchio::FrameIndex ctrl_frame {0};
  std::vector<std::string> joint_names;   // revolute joint names, JTC publish order
  std::vector<int> q_index;               // q vector index per joint_names entry
  bool ok {false};
};

class PtpBoxAlignNode : public rclcpp::Node
{
public:
  using AlignToBoxes = openarmx_ptp_box_align_msgs::action::AlignToBoxes;
  using GoalHandle = rclcpp_action::ServerGoalHandle<AlignToBoxes>;
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GripperCommand = control_msgs::action::GripperCommand;

  PtpBoxAlignNode()
  : rclcpp::Node("ptp_box_align_node")
  {
    move_time_ = this->declare_parameter<double>("move_time", 6.0);
    max_box_age_ = this->declare_parameter<double>("max_box_age", 60.0);
    gripper_open_pos_ = this->declare_parameter<double>("gripper_open_pos", 0.044);
    gripper_effort_ = this->declare_parameter<double>("gripper_effort", 14.0);
    // IK tuning (Pinocchio damped-least-squares / CLIK).
    ik_eps_ = this->declare_parameter<double>("ik_eps", 1e-4);
    ik_max_iter_ = this->declare_parameter<int>("ik_max_iter", 1000);
    ik_dt_ = this->declare_parameter<double>("ik_dt", 0.1);
    ik_damp_ = this->declare_parameter<double>("ik_damp", 1e-6);
    // Random IK restarts: a single neutral-seed CLIK gets stuck for higher/extended
    // hovers; restarts recover in-limit solutions (one-shot pre-move solve, so the
    // extra cost is fine).
    ik_restarts_ = this->declare_parameter<int>("ik_restarts", 20);
    const std::string boxes_topic =
      this->declare_parameter<std::string>("detected_boxes_topic", "/detected_boxes");
    // Controlled frame suffix: the IK target is openarmx_<side>_<suffix>. Default
    // "hand_tcp" so the goal z is the TOOL-CENTER-POINT (TCP) height (like cyclo);
    // set to "link7" to constrain link7 directly (like pilz).
    ctrl_suffix_ = this->declare_parameter<std::string>("controlled_link_suffix", "hand_tcp");

    // Approach-move controller selection (verified pick-sequence design, docs/issues_and_fixes/
    // 2026-06-07_pick_seq_v2_timing.md): the INIT->box approach is a FORWARD-POSITION ramp on
    // {side}_arm_position_controller, NOT a JTC trajectory. JTC is reserved for the precise
    // descend (hover->pick), which this hover-only node does not perform. Set
    // use_forward_position:=false to fall back to the legacy single-endpoint JTC move.
    use_forward_position_ = this->declare_parameter<bool>("use_forward_position", true);
    ramp_rate_dps_ = this->declare_parameter<double>("ramp_rate_dps", 90.0);
    ramp_hz_ = this->declare_parameter<double>("ramp_hz", 50.0);
    ramp_settle_ = this->declare_parameter<double>("ramp_settle", 0.1);
    restore_jtc_after_ = this->declare_parameter<bool>("restore_jtc_after", true);

    // Load the FULL robot model from /robot_description (latched) — the SAME model
    // robot_state_publisher uses, so the IK solver is guaranteed consistent with TF /
    // the real robot. Per-arm 7-DOF models are built at runtime via buildReducedModel
    // (lock the other arm + fingers), NOT from a separate hand-maintained solver URDF.
    // → the arm-base z / any geometry lives in ONE place (the xacro), no model drift.
    rclcpp::QoS rd_qos(rclcpp::KeepLast(1));
    rd_qos.reliable().transient_local();
    robot_desc_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/robot_description", rd_qos,
      [this](std_msgs::msg::String::SharedPtr msg) { onRobotDescription(msg->data); });

    // latched(transient_local): /detected_boxes is published once per detection,
    // so a late-starting backend still receives the most recent set (QoS must
    // match the publisher).
    rclcpp::QoS boxes_qos(rclcpp::KeepLast(1));
    boxes_qos.reliable().transient_local();
    boxes_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      boxes_topic, boxes_qos,
      [this](geometry_msgs::msg::PoseArray::SharedPtr msg) {
        boxes_ = msg->poses;
        boxes_stamp_ = this->now();
      });

    for (const std::string side : {"left", "right"}) {
      jtc_client_[side] = rclcpp_action::create_client<FollowJointTrajectory>(
        this, "/" + side + "_joint_trajectory_controller/follow_joint_trajectory");
      jtc_pub_[side] = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/" + side + "_joint_trajectory_controller/joint_trajectory", 10);
      grip_client_[side] = rclcpp_action::create_client<GripperCommand>(
        this, "/" + side + "_gripper_controller/gripper_cmd");
      fpos_pub_[side] = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/" + side + "_arm_position_controller/commands", 10);
    }

    switch_client_ = this->create_client<controller_manager_msgs::srv::SwitchController>(
      "/controller_manager/switch_controller");
    joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::QoS(rclcpp::KeepLast(10)),
      [this](sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(js_mutex_);
        const size_t n = std::min(msg->name.size(), msg->position.size());
        for (size_t i = 0; i < n; ++i) {
          latest_joint_pos_[msg->name[i]] = msg->position[i];
        }
      });

    action_server_ = rclcpp_action::create_server<AlignToBoxes>(
      this, "/openarmx/ptp_align_to_boxes",
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const AlignToBoxes::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<GoalHandle>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandle> gh) {
        std::thread{[this, gh]() { execute(gh); }}.detach();
      });

    RCLCPP_INFO(
      this->get_logger(),
      "ptp_box_align_node ready: action /openarmx/ptp_align_to_boxes "
      "(consumes /detected_boxes; Pinocchio IK -> single JTC endpoint)");
  }

private:
  // ----------------------------------------------------------- model loading
  // /robot_description callback: build the full Pinocchio model once, then derive each
  // arm's reduced (single-arm 7-DOF) model. Same source as robot_state_publisher → no
  // separate solver URDF, no model drift.
  void onRobotDescription(const std::string & urdf_xml)
  {
    if (full_model_ready_) {
      return;  // build once
    }
    try {
      pinocchio::urdf::buildModelFromXML(urdf_xml, full_model_);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to parse /robot_description: %s", e.what());
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Full robot model from /robot_description: nq=%d, %d joints.",
                full_model_.nq, static_cast<int>(full_model_.njoints));
    buildArmFromFull("left");
    buildArmFromFull("right");
    full_model_ready_ = true;
  }

  // Reduced single-arm model = full model with EVERY movable joint locked except this
  // arm's openarmx_<side>_jointN (the other arm + fingers locked at the neutral config).
  void buildArmFromFull(const std::string & side)
  {
    ArmModel arm;
    try {
      const std::string keep_prefix = "openarmx_" + side + "_joint";  // matches joint1..7
      std::vector<pinocchio::JointIndex> lock;
      for (pinocchio::JointIndex j = 1; j < full_model_.joints.size(); ++j) {
        const bool is_arm_joint = (full_model_.names[j].rfind(keep_prefix, 0) == 0);
        if (full_model_.joints[j].nq() >= 1 && !is_arm_joint) {
          lock.push_back(j);
        }
      }
      pinocchio::buildReducedModel(
        full_model_, lock, pinocchio::neutral(full_model_), arm.model);
      arm.data = pinocchio::Data(arm.model);
      const std::string frame = "openarmx_" + side + "_" + ctrl_suffix_;
      if (!arm.model.existFrame(frame)) {
        RCLCPP_ERROR(this->get_logger(), "%s arm: frame '%s' not in reduced model.",
                     side.c_str(), frame.c_str());
        arms_[side] = arm;
        return;
      }
      arm.ctrl_frame = arm.model.getFrameId(frame);
      for (pinocchio::JointIndex j = 1; j < arm.model.joints.size(); ++j) {
        if (arm.model.joints[j].nq() == 1) {
          arm.joint_names.push_back(arm.model.names[j]);
          arm.q_index.push_back(arm.model.joints[j].idx_q());
        }
      }
      arm.ok = !arm.joint_names.empty();
      RCLCPP_INFO(this->get_logger(),
                  "%s arm reduced model: nq=%d, %zu movable joints, target='%s'.",
                  side.c_str(), arm.model.nq, arm.joint_names.size(), frame.c_str());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "%s arm reduced-model build failed: %s",
                   side.c_str(), e.what());
    }
    arms_[side] = arm;
  }

  // ------------------------------------------------------------------ IK core
  // Pinocchio damped-least-squares (CLIK) IK for the link7 frame, seeded from the
  // model's neutral configuration. Returns true on convergence; q holds the
  // solution, residual holds the final 6D error norm (metres+rad mixed).
  // One limit-aware DLS (CLIK) descent from seed q0. Clamps to joint limits EVERY
  // step (not only at the end): without per-step clamping the descent converges to
  // an out-of-limit q whose final one-shot clamp lands the TCP far from target
  // (measured 65 mm). Returns convergence; res = final 6D error norm.
  bool ikDescent(ArmModel & arm, const pinocchio::SE3 & oMdes,
                 Eigen::VectorXd & q, double & res)
  {
    Eigen::MatrixXd J(6, arm.model.nv);
    J.setZero();
    Eigen::Matrix<double, 6, 1> err;
    Eigen::VectorXd v(arm.model.nv);
    for (int i = 0; i < ik_max_iter_; ++i) {
      pinocchio::forwardKinematics(arm.model, arm.data, q);
      pinocchio::updateFramePlacement(arm.model, arm.data, arm.ctrl_frame);
      const pinocchio::SE3 iMd = arm.data.oMf[arm.ctrl_frame].actInv(oMdes);
      err = pinocchio::log6(iMd).toVector();
      res = err.norm();
      if (res < ik_eps_) {
        return true;
      }
      pinocchio::computeFrameJacobian(arm.model, arm.data, q, arm.ctrl_frame, J);
      pinocchio::Data::Matrix6 Jlog;
      pinocchio::Jlog6(iMd.inverse(), Jlog);
      J = -Jlog * J;
      Eigen::Matrix<double, 6, 6> JJt = J * J.transpose();
      JJt.diagonal().array() += ik_damp_;
      v.noalias() = -J.transpose() * JJt.ldlt().solve(err);
      q = pinocchio::integrate(arm.model, q, v * ik_dt_);
      clampToLimits(arm, q);
    }
    return false;
  }

  // Robust IK: try the neutral seed first (fast common case), then up to
  // ik_restarts_ random restarts. A single neutral-seed CLIK gets stuck at limits
  // for higher/extended hovers (e.g. fails above TCP z≈0.80 while an in-limit
  // solution exists to ≈0.88); random restarts recover those. Returns the first
  // converged solution, else the best-effort (lowest-residual) q.
  bool solveIK(ArmModel & arm, const pinocchio::SE3 & oMdes,
               Eigen::VectorXd & q, double & residual)
  {
    Eigen::VectorXd best_q = pinocchio::neutral(arm.model);
    double best_res = std::numeric_limits<double>::max();
    for (int attempt = 0; attempt <= ik_restarts_; ++attempt) {
      Eigen::VectorXd qa = (attempt == 0) ? pinocchio::neutral(arm.model)
                                          : pinocchio::randomConfiguration(arm.model);
      double res = std::numeric_limits<double>::max();
      const bool converged = ikDescent(arm, oMdes, qa, res);
      if (res < best_res) {
        best_res = res;
        best_q = qa;
      }
      if (converged) {
        q = qa;
        residual = res;
        return true;
      }
    }
    q = best_q;
    residual = best_res;
    return false;
  }

  static void clampToLimits(const ArmModel & arm, Eigen::VectorXd & q)
  {
    for (size_t k = 0; k < arm.q_index.size(); ++k) {
      const int qi = arm.q_index[k];
      q[qi] = std::min(std::max(q[qi], arm.model.lowerPositionLimit[qi]),
                       arm.model.upperPositionLimit[qi]);
    }
  }

  // Position of the controlled frame in the base frame at configuration q.
  Eigen::Vector3d framePosition(ArmModel & arm, const Eigen::VectorXd & q)
  {
    pinocchio::forwardKinematics(arm.model, arm.data, q);
    pinocchio::updateFramePlacement(arm.model, arm.data, arm.ctrl_frame);
    return arm.data.oMf[arm.ctrl_frame].translation();
  }

  // --------------------------------------------------------- perception input
  std::vector<std::array<double, 3>> currentBoxes()
  {
    std::vector<std::array<double, 3>> boxes;
    if (boxes_.empty()) {
      return boxes;
    }
    const double age = (this->now() - boxes_stamp_).seconds();
    if (age > max_box_age_) {
      RCLCPP_WARN(this->get_logger(),
                  "/detected_boxes is stale (%.1fs) -- run detection again.", age);
      return boxes;
    }
    for (const auto & p : boxes_) {
      boxes.push_back({p.position.x, p.position.y, p.position.z});
    }
    // +Y (left) first.
    std::sort(boxes.begin(), boxes.end(),
              [](const auto & a, const auto & b) { return a[1] > b[1]; });
    return boxes;
  }

  // -------------------------------------------------------- forward position
  // Latest measured positions for this arm's 7 joints in the controller order
  // (openarmx_<side>_joint1..7). false if any joint has no /joint_states sample yet.
  bool currentArmPositions(const std::string & side, std::vector<double> & out)
  {
    out.assign(7, 0.0);
    std::lock_guard<std::mutex> lk(js_mutex_);
    for (int i = 1; i <= 7; ++i) {
      const std::string jn = "openarmx_" + side + "_joint" + std::to_string(i);
      auto it = latest_joint_pos_.find(jn);
      if (it == latest_joint_pos_.end()) {
        return false;
      }
      out[static_cast<size_t>(i - 1)] = it->second;
    }
    return true;
  }

  // q_goal mapped to this arm's 7 joints in controller order (openarmx_<side>_joint1..7).
  std::vector<double> goalArmPositions(const std::string & side, const ArmModel & arm,
                                       const Eigen::VectorXd & q)
  {
    std::map<std::string, double> by_name;
    for (size_t k = 0; k < arm.joint_names.size(); ++k) {
      by_name[arm.joint_names[k]] = q[arm.q_index[k]];
    }
    std::vector<double> out(7, 0.0);
    for (int i = 1; i <= 7; ++i) {
      out[static_cast<size_t>(i - 1)] = by_name["openarmx_" + side + "_joint" + std::to_string(i)];
    }
    return out;
  }

  // Activate/deactivate controllers via /controller_manager/switch_controller (STRICT: the
  // arm_position_controller and JTC share the position interface, so the switch is all-or-nothing).
  bool switchControllers(const std::vector<std::string> & activate,
                         const std::vector<std::string> & deactivate)
  {
    if (!switch_client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "switch_controller service unavailable.");
      return false;
    }
    auto req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    req->activate_controllers = activate;
    req->deactivate_controllers = deactivate;
    req->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
    req->activate_asap = true;
    req->timeout = rclcpp::Duration::from_seconds(5.0);
    auto fut = switch_client_->async_send_request(req);
    if (fut.wait_for(std::chrono::seconds(8)) != std::future_status::ready) {
      RCLCPP_ERROR(this->get_logger(), "switch_controller timeout.");
      return false;
    }
    return fut.get()->ok;
  }

  // Forward-position approach: switch this arm onto its arm_position_controller and ramp from
  // the CURRENT measured config to q_goal under a joint-rate limit (mirrors the verified
  // experiments/ptp_pick_seq_v2 forward_ramp). Restores JTC afterwards (seeded at the current
  // pose) so the subsequent JTC descend and other JTC consumers keep working.
  bool sendForwardRamp(const std::string & side, const ArmModel & arm, const Eigen::VectorXd & q)
  {
    const std::string fpos = side + "_arm_position_controller";
    const std::string jtc = side + "_joint_trajectory_controller";
    if (!switchControllers({fpos}, {jtc})) {
      RCLCPP_ERROR(this->get_logger(), "%s: switch to forward position failed.", side.c_str());
      return false;
    }
    std::vector<double> cmd;
    if (!currentArmPositions(side, cmd)) {
      RCLCPP_ERROR(this->get_logger(),
                   "%s: no /joint_states yet; cannot seed forward ramp.", side.c_str());
      if (restore_jtc_after_) {
        switchControllers({jtc}, {fpos});
      }
      return false;
    }
    const std::vector<double> target = goalArmPositions(side, arm, q);
    const double max_step = (ramp_rate_dps_ * M_PI / 180.0) / std::max(1.0, ramp_hz_);
    const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / std::max(1.0, ramp_hz_)));
    int steps = 0;
    while (rclcpp::ok()) {
      double max_err = 0.0;
      for (size_t k = 0; k < 7; ++k) {
        double d = target[k] - cmd[k];
        d = std::min(std::max(d, -max_step), max_step);
        cmd[k] += d;
        max_err = std::max(max_err, std::abs(target[k] - cmd[k]));
      }
      std_msgs::msg::Float64MultiArray m;
      m.data = cmd;
      fpos_pub_[side]->publish(m);
      if (max_err < 1e-3) {
        break;
      }
      std::this_thread::sleep_for(period_ns);
      if (++steps > 600) {
        RCLCPP_WARN(this->get_logger(), "%s: forward ramp step cap (600) hit.", side.c_str());
        break;
      }
    }
    // settle: hold the target briefly so the motors converge before any next phase.
    const int settle_n = static_cast<int>(ramp_hz_ * ramp_settle_);
    for (int i = 0; i < settle_n && rclcpp::ok(); ++i) {
      std_msgs::msg::Float64MultiArray m;
      m.data = target;
      fpos_pub_[side]->publish(m);
      std::this_thread::sleep_for(period_ns);
    }
    RCLCPP_INFO(this->get_logger(), "%s: forward-position approach done (%d steps).",
                side.c_str(), steps);
    if (restore_jtc_after_ && switchControllers({jtc}, {fpos})) {
      // Seed the reactivated JTC reference at the current pose (open_loop_control=true) so it
      // holds here instead of snapping to a stale last-command reference.
      trajectory_msgs::msg::JointTrajectory warm;
      warm.joint_names = arm.joint_names;
      trajectory_msgs::msg::JointTrajectoryPoint pt;
      {
        std::lock_guard<std::mutex> lk(js_mutex_);
        for (const auto & jn : arm.joint_names) {
          auto it = latest_joint_pos_.find(jn);
          pt.positions.push_back(it != latest_joint_pos_.end() ? it->second : 0.0);
        }
      }
      pt.time_from_start = rclcpp::Duration::from_seconds(0.2);
      warm.points.push_back(pt);
      jtc_pub_[side]->publish(warm);
    }
    return true;
  }

  // ------------------------------------------------------------------ trajectory
  // Send a single-endpoint trajectory (current -> q_goal over move_time) to the
  // arm JTC via FollowJointTrajectory; falls back to the topic if the action is
  // unavailable. Returns the goal handle future (or nullptr on topic fallback).
  bool sendTrajectory(const std::string & side, const ArmModel & arm,
                      const Eigen::VectorXd & q, bool & used_action)
  {
    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = arm.joint_names;
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions.reserve(arm.joint_names.size());
    for (int qi : arm.q_index) {
      pt.positions.push_back(q[qi]);
    }
    pt.time_from_start = rclcpp::Duration::from_seconds(move_time_);
    traj.points.push_back(pt);

    auto client = jtc_client_[side];
    if (!client->wait_for_action_server(std::chrono::seconds(5))) {
      jtc_pub_[side]->publish(traj);
      RCLCPP_WARN(this->get_logger(),
                  "%s JTC follow_joint_trajectory unavailable; fell back to topic "
                  "(no completion signal).", side.c_str());
      used_action = false;
      return true;
    }
    FollowJointTrajectory::Goal goal;
    goal.trajectory = traj;
    auto send_future = client->async_send_goal(goal);
    if (send_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      used_action = false;
      return false;
    }
    auto gh = send_future.get();
    if (!gh) {
      used_action = false;
      return false;
    }
    auto result_future = client->async_get_result(gh);
    const auto deadline = std::chrono::duration<double>(move_time_ + 5.0);
    used_action = true;
    if (result_future.wait_for(std::chrono::duration_cast<std::chrono::nanoseconds>(deadline)) !=
        std::future_status::ready)
    {
      RCLCPP_WARN(this->get_logger(), "%s: trajectory result timeout.", side.c_str());
      return false;
    }
    return result_future.get().code == rclcpp_action::ResultCode::SUCCEEDED;
  }

  std::string openGripper(const std::string & side)
  {
    auto client = grip_client_[side];
    if (!client->wait_for_action_server(std::chrono::seconds(3))) {
      return side + " gripper action unavailable";
    }
    GripperCommand::Goal goal;
    goal.command.position = gripper_open_pos_;
    goal.command.max_effort = gripper_effort_;
    auto send_future = client->async_send_goal(goal);
    if (send_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      return side + " gripper goal send timeout";
    }
    auto gh = send_future.get();
    if (!gh) {
      return side + " gripper goal rejected";
    }
    auto result_future = client->async_get_result(gh);
    if (result_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
      return side + " gripper open timeout";
    }
    return "";
  }

  // -------------------------------------------------------------- execute
  void feedback(const std::shared_ptr<GoalHandle> & gh, const std::string & phase, float prog)
  {
    auto fb = std::make_shared<AlignToBoxes::Feedback>();
    fb->phase = phase;
    fb->progress = prog;
    gh->publish_feedback(fb);
  }

  void execute(const std::shared_ptr<GoalHandle> gh)
  {
    // [경로 상호배타·EX] 정본 pick&place 는 resident 경로(SH 락)다. 이 hover 디버그 노드는
    // resident 가 모션 중이면(SH 보유) 같은 controller_manager 를 동시에 만지지 않도록 EX 락
    // 획득에 실패하면 즉시 abort. 함수 종료 시 RAII 로 해제. (/tmp 미기록 등 open 실패 시엔
    // 디버그 도구를 막지 않도록 락 없이 진행.)
    const int path_fd = ::open("/tmp/openarmx_motion_path.lock", O_RDWR | O_CREAT, 0666);
    if (path_fd < 0) {   // fail-SAFE: 락을 열 수 없으면(권한/디스크 등) 모션 거부(조용한 진행 금지)
      auto busy = std::make_shared<AlignToBoxes::Result>();
      busy->success = false;
      busy->message = "motion-path lock open failed — refuse to move (fail-safe)";
      RCLCPP_ERROR(this->get_logger(),
                   "AlignToBoxes refused: cannot open motion-path lock (errno=%d).", errno);
      gh->abort(busy);
      return;
    }
    if (::flock(path_fd, LOCK_EX | LOCK_NB) != 0) {   // resident SH 보유 중 → busy
      ::close(path_fd);
      auto busy = std::make_shared<AlignToBoxes::Result>();
      busy->success = false;
      busy->message = "busy: resident pick path active (motion-path lock held)";
      RCLCPP_WARN(this->get_logger(),
                  "AlignToBoxes refused: resident pick path holds the motion lock.");
      gh->abort(busy);
      return;
    }
    struct PathLockGuard {
      int fd;
      ~PathLockGuard() { if (fd >= 0) { ::flock(fd, LOCK_UN); ::close(fd); } }
    } path_guard{path_fd};

    const auto goal = gh->get_goal();
    auto result = std::make_shared<AlignToBoxes::Result>();
    const Eigen::Matrix3d R = rpyDegToRot(goal->roll_deg, goal->pitch_deg, goal->yaw_deg);

    // [4] 협조적 취소: Cancel 요청 시 단계 경계에서 깨끗이 종료(현재 단계까지만 진행).
    //     램프 도중 즉시정지는 sendForwardRamp 내부 폴링이 필요 — 별도 트랙.
    auto check_cancel = [&]() -> bool {
      if (gh->is_canceling()) {
        result->success = false;
        result->message = "canceled at step boundary";
        gh->canceled(result);
        return true;
      }
      return false;
    };

    feedback(gh, "reading_boxes", 0.1f);
    auto boxes = currentBoxes();
    {
      std::ostringstream js;
      js << "[";
      for (size_t i = 0; i < boxes.size(); ++i) {
        js << (i ? "," : "") << "{\"base\":[" << boxes[i][0] << "," << boxes[i][1]
           << "," << boxes[i][2] << "]}";
      }
      js << "]";
      result->detections_json = js.str();
    }
    if (boxes.empty()) {
      result->success = false;
      result->message = "no detected boxes (run perception first -> /detected_boxes)";
      gh->abort(result);
      return;
    }

    if (check_cancel()) return;
    feedback(gh, "assigning", 0.4f);
    std::string want = goal->arms.empty() ? "both" : goal->arms;
    std::transform(want.begin(), want.end(), want.begin(), ::tolower);
    std::map<std::string, std::array<double, 3>> assigned;
    std::vector<std::array<double, 3>> left_boxes, right_boxes;
    for (const auto & b : boxes) {
      (b[1] >= 0 ? left_boxes : right_boxes).push_back(b);
    }
    std::sort(right_boxes.begin(), right_boxes.end(),
              [](const auto & a, const auto & b) { return a[1] < b[1]; });
    if ((want == "both" || want == "left") && !left_boxes.empty()) {
      assigned["left"] = left_boxes.front();
    }
    if ((want == "both" || want == "right") && !right_boxes.empty()) {
      assigned["right"] = right_boxes.front();
    }
    if ((want == "both" || want == "left") && !assigned.count("left")) {
      assigned["left"] = boxes.front();
    }
    if ((want == "both" || want == "right") && !assigned.count("right")) {
      assigned["right"] = boxes.back();
    }

    // 핸드를 먼저 연다 — 박스로 접근하기 전에 그리퍼를 열어둔다(사용자 요청).
    feedback(gh, "opening_gripper", 0.5f);
    for (auto & [side, box] : assigned) {
      (void)box;
      if (!arms_[side].ok) {
        continue;
      }
      const std::string gerr = openGripper(side);
      if (!gerr.empty()) {
        RCLCPP_WARN(this->get_logger(), "%s gripper open: %s", side.c_str(), gerr.c_str());
      }
    }

    feedback(gh, "solving", 0.6f);
    std::ostringstream report;
    report << "{";
    bool first = true;
    bool any_moved = false;
    for (auto & [side, box] : assigned) {
      ArmModel & arm = arms_[side];
      report << (first ? "" : ",") << "\"" << side << "\":";
      first = false;
      if (!arm.ok) {
        report << "{\"error\":\"arm model not loaded\"}";
        RCLCPP_WARN(this->get_logger(), "%s: arm model not loaded; skipped.", side.c_str());
        continue;
      }
      pinocchio::SE3 oMdes(R, Eigen::Vector3d(box[0], box[1], goal->z));
      Eigen::VectorXd q;
      double residual = 0.0;
      const bool converged = solveIK(arm, oMdes, q, residual);
      const Eigen::Vector3d tcp = framePosition(arm, q);
      const double err_mm =
        std::sqrt(std::pow(tcp[0] - box[0], 2) + std::pow(tcp[1] - box[1], 2) +
                  std::pow(tcp[2] - goal->z, 2)) * 1000.0;
      report << "{\"box_base\":[" << box[0] << "," << box[1] << "," << box[2] << "]"
             << ",\"tcp_target\":[" << box[0] << "," << box[1] << "," << goal->z << "]"
             << ",\"ik_converged\":" << (converged ? "true" : "false")
             << ",\"ik_residual\":" << residual
             << ",\"err_mm\":" << err_mm;
      if (!converged) {
        RCLCPP_WARN(this->get_logger(),
                    "%s: IK did not converge (residual %.4g); sending best-effort.",
                    side.c_str(), residual);
      }
      if (check_cancel()) return;
      feedback(gh, "moving", 0.8f);
      bool moved;
      if (use_forward_position_) {
        // Approach is a forward-position ramp; JTC is reserved for the descend (see params).
        moved = sendForwardRamp(side, arm, q);
      } else {
        bool used_action = false;
        moved = sendTrajectory(side, arm, q, used_action);
      }
      report << ",\"moved\":" << (moved ? "true" : "false") << "}";
      any_moved = any_moved || moved;
    }
    report << "}";

    feedback(gh, "done", 1.0f);
    result->success = any_moved;
    result->assignments_json = report.str();
    std::ostringstream msg;
    msg << boxes.size() << " box(es) from perception; ptp-moved " << assigned.size() << " arm(s)";
    result->message = msg.str();
    gh->succeed(result);
  }

  // ----------------------------------------------------------------- members
  double move_time_;
  double max_box_age_;
  double gripper_open_pos_;
  double gripper_effort_;
  double ik_eps_;
  int ik_max_iter_;
  double ik_dt_;
  double ik_damp_;
  int ik_restarts_;
  std::string ctrl_suffix_;
  bool use_forward_position_;
  double ramp_rate_dps_;
  double ramp_hz_;
  double ramp_settle_;
  bool restore_jtc_after_;

  pinocchio::Model full_model_;
  bool full_model_ready_ {false};
  std::map<std::string, ArmModel> arms_;
  std::vector<geometry_msgs::msg::Pose> boxes_;
  rclcpp::Time boxes_stamp_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_desc_sub_;

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr boxes_sub_;
  std::map<std::string, rclcpp_action::Client<FollowJointTrajectory>::SharedPtr> jtc_client_;
  std::map<std::string, rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr> jtc_pub_;
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr> fpos_pub_;
  std::map<std::string, rclcpp_action::Client<GripperCommand>::SharedPtr> grip_client_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  std::mutex js_mutex_;
  std::map<std::string, double> latest_joint_pos_;
  rclcpp_action::Server<AlignToBoxes>::SharedPtr action_server_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PtpBoxAlignNode>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
