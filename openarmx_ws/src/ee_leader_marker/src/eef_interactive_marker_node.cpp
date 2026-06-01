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
// Original Author: Yeonguk Kim (cyclo_motion_controller_ros)
//
// PoseStamped-only port for the ``ee_leader_marker`` package -- removes the
// robotis_interfaces dependency so this single-file node can be dropped into
// any ROS 2 (Humble+) workspace with only standard message packages. The
// dragged 6-DoF pose is published as a plain ``geometry_msgs/PoseStamped``
// (frame_id = base_frame, stamp = now).

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace ee_leader_marker
{
class InteractiveMarkerNode : public rclcpp::Node
{
public:
  InteractiveMarkerNode()
  : Node("ee_leader_marker_node"),
    initialized_(false),
    dragging_(false)
  {
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
    controlled_link_ =
      this->declare_parameter<std::string>("controlled_link", "end_effector_link");
    goal_topic_ =
      this->declare_parameter<std::string>("goal_topic", "/goal_pose");
    server_name_ =
      this->declare_parameter<std::string>("server_name", "ee_leader_marker");
    marker_name_ =
      this->declare_parameter<std::string>("marker_name", "goal_marker");
    marker_description_ =
      this->declare_parameter<std::string>("marker_description", "EE Leader Marker");
    marker_scale_ = this->declare_parameter<double>("marker_scale", 0.2);
    publish_while_dragging_ =
      this->declare_parameter<bool>("publish_while_dragging", true);
    marker_color_r_ = this->declare_parameter<double>("marker_color_r", 0.2);
    marker_color_g_ = this->declare_parameter<double>("marker_color_g", 0.8);
    marker_color_b_ = this->declare_parameter<double>("marker_color_b", 0.2);
    // While idle (no user drag), keep the marker glued to the current
    // controlled_link TF so the marker tracks any motion of the robot
    // (joint slider, jog, etc). Disable to freeze the marker at the last
    // user-set pose regardless of robot motion.
    auto_follow_link_ =
      this->declare_parameter<bool>("auto_follow_link", true);
    follow_rate_hz_ =
      this->declare_parameter<double>("follow_rate_hz", 10.0);

    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic_, 10);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
                server_name_,
                this->get_node_base_interface(),
                this->get_node_clock_interface(),
                this->get_node_logging_interface(),
                this->get_node_topics_interface(),
                this->get_node_services_interface(),
                rclcpp::QoS(100),
                rclcpp::QoS(10));

    const int period_ms =
      std::max(20, static_cast<int>(1000.0 / std::max(1e-3, follow_rate_hz_)));
    update_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(period_ms),
                std::bind(&InteractiveMarkerNode::onTick, this));

    RCLCPP_INFO(this->get_logger(), "EE Leader Marker node started");
    RCLCPP_INFO(this->get_logger(), "  - Base frame: %s", base_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "  - Controlled link: %s", controlled_link_.c_str());
    RCLCPP_INFO(this->get_logger(), "  - Goal topic: %s (PoseStamped)", goal_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  - Marker name: %s", marker_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "  - Marker server: %s", server_name_.c_str());
  }

private:
  void create6DofMarker(const geometry_msgs::msg::Pose & pose, const std::string & frame_id)
  {
    visualization_msgs::msg::InteractiveMarker marker;
    marker.header.frame_id = frame_id;
    marker.name = marker_name_;
    marker.description = marker_description_;
    marker.scale = marker_scale_;
    marker.pose = pose;

    visualization_msgs::msg::Marker box_marker;
    box_marker.type = visualization_msgs::msg::Marker::CUBE;
    box_marker.scale.x = marker.scale * 0.2;
    box_marker.scale.y = marker.scale * 0.2;
    box_marker.scale.z = marker.scale * 0.2;
    box_marker.color.r = marker_color_r_;
    box_marker.color.g = marker_color_g_;
    box_marker.color.b = marker_color_b_;
    box_marker.color.a = 0.8;

    visualization_msgs::msg::InteractiveMarkerControl box_control;
    box_control.always_visible = true;
    box_control.markers.push_back(box_marker);
    marker.controls.push_back(box_control);

    addAxisControls(marker);

    server_->insert(
                marker,
                std::bind(&InteractiveMarkerNode::markerFeedback, this, std::placeholders::_1));
  }

  void addAxisControls(visualization_msgs::msg::InteractiveMarker & marker)
  {
    visualization_msgs::msg::InteractiveMarkerControl control;

    control.orientation.w = 1.0;
    control.orientation.x = 1.0;
    control.orientation.y = 0.0;
    control.orientation.z = 0.0;
    control.name = "rotate_x";
    control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    marker.controls.push_back(control);

    control.orientation.w = 1.0;
    control.orientation.x = 0.0;
    control.orientation.y = 1.0;
    control.orientation.z = 0.0;
    control.name = "rotate_y";
    control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    marker.controls.push_back(control);

    control.orientation.w = 1.0;
    control.orientation.x = 0.0;
    control.orientation.y = 0.0;
    control.orientation.z = 1.0;
    control.name = "rotate_z";
    control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    marker.controls.push_back(control);
  }

  void markerFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
  {
    if (feedback->marker_name != marker_name_) {
      return;
    }

    if (feedback->event_type ==
      visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_DOWN)
    {
      dragging_ = true;
      return;
    }

    if (feedback->event_type ==
      visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP)
    {
      dragging_ = false;
      publishGoal(
                    feedback->pose,
                    feedback->header.frame_id.empty() ? base_frame_ : feedback->header.frame_id);
      return;
    }

    if (feedback->event_type ==
      visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE &&
      dragging_ && publish_while_dragging_)
    {
      publishGoal(
                    feedback->pose,
                    feedback->header.frame_id.empty() ? base_frame_ : feedback->header.frame_id);
    }
  }

  void onTick()
  {
    if (!initialized_) {
      initializeMarkerIfReady();
      return;
    }
    if (!auto_follow_link_ || dragging_) {
      return;
    }
    // Idle follow: snap marker to current controlled_link TF so the marker
    // tracks every robot motion driven by other sources (joint slider, jog,
    // scenario playback). publishGoal is NOT called here — only user drag
    // produces a new goal_pose; follow updates are display-only.
    geometry_msgs::msg::PoseStamped current;
    if (!lookupPose(controlled_link_, current)) {
      return;
    }
    server_->setPose(marker_name_, current.pose, current.header);
    server_->applyChanges();
  }

  void initializeMarkerIfReady()
  {
    if (initialized_) {
      return;
    }

    if (!lookupPose(controlled_link_, initial_pose_)) {
      return;
    }

    create6DofMarker(initial_pose_.pose, initial_pose_.header.frame_id);
    server_->applyChanges();
    publishGoal(initial_pose_.pose, initial_pose_.header.frame_id);
    initialized_ = true;

    RCLCPP_INFO(this->get_logger(),
      "EE Leader Marker initialized from link transform. "
      "auto_follow_link=%s @ %.1f Hz",
      auto_follow_link_ ? "true" : "false", follow_rate_hz_);
  }

  bool lookupPose(const std::string & child_frame, geometry_msgs::msg::PoseStamped & pose_out)
  {
    try {
      const auto tf =
        tf_buffer_->lookupTransform(base_frame_, child_frame, tf2::TimePointZero);
      pose_out.header = tf.header;
      pose_out.pose.position.x = tf.transform.translation.x;
      pose_out.pose.position.y = tf.transform.translation.y;
      pose_out.pose.position.z = tf.transform.translation.z;
      pose_out.pose.orientation = tf.transform.rotation;
      return true;
    } catch (const std::exception &) {
      return false;
    }
  }

  void publishGoal(const geometry_msgs::msg::Pose & pose, const std::string & frame_id)
  {
    geometry_msgs::msg::PoseStamped goal_msg;
    goal_msg.header.stamp = this->get_clock()->now();
    goal_msg.header.frame_id = frame_id;
    goal_msg.pose = pose;
    goal_pub_->publish(goal_msg);
  }

  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::TimerBase::SharedPtr update_timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string base_frame_;
  std::string controlled_link_;
  std::string goal_topic_;
  std::string server_name_;
  std::string marker_name_;
  std::string marker_description_;
  double marker_scale_;
  bool publish_while_dragging_;
  double marker_color_r_;
  double marker_color_g_;
  double marker_color_b_;
  bool auto_follow_link_;
  double follow_rate_hz_;

  bool initialized_;
  bool dragging_;
  geometry_msgs::msg::PoseStamped initial_pose_;
};
}  // namespace ee_leader_marker

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ee_leader_marker::InteractiveMarkerNode>());
  rclcpp::shutdown();
  return 0;
}
