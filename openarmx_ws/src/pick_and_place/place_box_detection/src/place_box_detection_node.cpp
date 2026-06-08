// ROS2 node wiring the three-stage place-box pipeline:
//   /tof/range (sensor_msgs/Range)  --Stage1-->  candidate gate
//   /camera/.../points (cloud)      --Stage2-->  desk plane + upright wall (PCL)
//                                   --Stage3-->  color (5 yolov8 mini-box colors)
// Publishes:
//   /place_box/info        std_msgs/String           JSON summary
//   /place_box/markers     visualization_msgs/MarkerArray  desk/wall/normal/text
//   /place_box/wall_cloud  sensor_msgs/PointCloud2   wall inliers (body frame)
#include <chrono>
#include <memory>
#include <sstream>
#include <string>

#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include "place_box_detection/tof_gate.hpp"
#include "place_box_detection/wall_detector.hpp"
#include "place_box_detection/color_classifier.hpp"

using namespace std::chrono_literals;

namespace place_box_detection
{

class PlaceBoxDetectionNode : public rclcpp::Node
{
public:
  PlaceBoxDetectionNode() : rclcpp::Node("place_box_detection_node")
  {
    body_frame_ = declare_parameter<std::string>("body_frame", "openarmx_body_link0");
    cloud_topic_ = declare_parameter<std::string>(
      "cloud_topic", "/camera/camera/depth/color/points");
    tof_topic_ = declare_parameter<std::string>("tof_topic", "/tof/range");
    period_s_ = declare_parameter<double>("period_s", 0.5);
    require_tof_ = declare_parameter<bool>("require_tof", false);  // run cloud even w/o TOF

    // Stage params (overridable from YAML).
    tof_p_.min_range_m = declare_parameter<double>("tof.min_range_m", 0.30);
    tof_p_.max_range_m = declare_parameter<double>("tof.max_range_m", 1.20);
    tof_p_.stale_timeout_s = declare_parameter<double>("tof.stale_timeout_s", 0.5);
    tof_p_.mount_x_offset_m = declare_parameter<double>("tof.mount_x_offset_m", 0.0);

    desk_p_.z_min = declare_parameter<double>("desk.z_min", 0.55);
    desk_p_.z_max = declare_parameter<double>("desk.z_max", 0.85);
    desk_p_.ransac_thresh = declare_parameter<double>("desk.ransac_thresh", 0.006);
    desk_p_.axis_eps_deg = declare_parameter<double>("desk.axis_eps_deg", 12.0);
    desk_p_.min_inliers = declare_parameter<int>("desk.min_inliers", 2000);

    wall_p_.height_margin = declare_parameter<double>("wall.height_margin", 0.03);
    wall_p_.depth_gate_tol = declare_parameter<double>("wall.depth_gate_tol", 0.10);
    wall_p_.use_depth_gate = declare_parameter<bool>("wall.use_depth_gate", true);
    wall_p_.ransac_thresh = declare_parameter<double>("wall.ransac_thresh", 0.008);
    wall_p_.axis_eps_deg = declare_parameter<double>("wall.axis_eps_deg", 15.0);
    wall_p_.max_nz = declare_parameter<double>("wall.max_nz", 0.20);
    wall_p_.min_width = declare_parameter<double>("wall.min_width", 0.10);
    wall_p_.min_visible_height =
      declare_parameter<double>("wall.min_visible_height", 0.06);
    wall_p_.min_inliers = declare_parameter<int>("wall.min_inliers", 800);
    wall_p_.require_faces_robot =
      declare_parameter<bool>("wall.require_faces_robot", true);

    color_p_.min_saturation = declare_parameter<double>("color.min_saturation", 0.25);
    color_p_.min_value = declare_parameter<double>("color.min_value", 0.15);
    color_p_.min_confidence = declare_parameter<double>("color.min_confidence", 0.40);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    auto sensor_qos = rclcpp::SensorDataQoS();
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, sensor_qos,
      [this](sensor_msgs::msg::PointCloud2::SharedPtr m) { last_cloud_ = m; });
    tof_sub_ = create_subscription<sensor_msgs::msg::Range>(
      tof_topic_, sensor_qos,
      [this](sensor_msgs::msg::Range::SharedPtr m) {
        last_tof_ = m; last_tof_stamp_ = now();
      });

    pub_info_ = create_publisher<std_msgs::msg::String>("/place_box/info", 10);
    pub_markers_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("/place_box/markers", 5);
    pub_cloud_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("/place_box/wall_cloud", 5);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(period_s_),
      std::bind(&PlaceBoxDetectionNode::tick, this));

    RCLCPP_INFO(get_logger(),
                "place_box_detection up: cloud=%s tof=%s body=%s period=%.2fs",
                cloud_topic_.c_str(), tof_topic_.c_str(), body_frame_.c_str(),
                period_s_);
  }

private:
  void tick()
  {
    PlaceTarget target;

    // --- Stage 1: TOF gate ---
    double range = std::numeric_limits<double>::quiet_NaN();
    double age = 1e9;
    if (last_tof_) {
      range = last_tof_->range;
      age = (now() - last_tof_stamp_).seconds();
    }
    target.tof = detectWallCandidateFromTof(range, age, tof_p_);
    if (require_tof_ && !target.tof.present) {
      publish(target);
      return;
    }

    if (!last_cloud_) { return; }

    // Transform cloud -> body frame.
    Eigen::Affine3f body_T_cam;
    if (!lookupBodyTf(last_cloud_->header, body_T_cam)) {
      return;
    }
    CloudT cloud_cam;
    pcl::fromROSMsg(*last_cloud_, cloud_cam);
    CloudT::Ptr cloud_body = transformToBody(cloud_cam, body_T_cam);

    // --- Stage 2a: desk plane ---
    if (!detectDeskPlane(*cloud_body, desk_p_, target.desk)) {
      publish(target);
      return;
    }
    // --- Stage 2b: vertical wall ---
    if (!detectVerticalWall(*cloud_body, target.desk, target.tof, wall_p_,
                            target.wall)) {
      publish(target);
      return;
    }
    // --- Stage 3: color ---
    if (target.wall.inlier_cloud) {
      target.color = classifyColor(*target.wall.inlier_cloud, color_p_);
    }
    target.ok = true;
    publish(target);

    RCLCPP_INFO(get_logger(),
                "WALL @x=%.3fm w=%.2f h_vis=%.2f nz=%.3f color=%s(%.2f) "
                "desk_z=%.3f tof=%s",
                target.wall.front_distance, target.wall.width,
                target.wall.visible_height, target.wall.nz_abs,
                target.color.name.c_str(), target.color.confidence,
                target.desk.z, target.tof.present ? "yes" : "no");
  }

  bool lookupBodyTf(const std_msgs::msg::Header & hdr, Eigen::Affine3f & out)
  {
    try {
      auto ts = tf_buffer_->lookupTransform(body_frame_, hdr.frame_id,
                                            tf2::TimePointZero, 100ms);
      out = tf2::transformToEigen(ts).cast<float>();
      return true;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "TF %s<-%s unavailable: %s", body_frame_.c_str(),
                           hdr.frame_id.c_str(), e.what());
      return false;
    }
  }

  // ----------------------------- output ----------------------------------
  void colorRgba(ColorLabel c, float & r, float & g, float & b)
  {
    switch (c) {
      case ColorLabel::Blue:   r = 0.1f; g = 0.3f; b = 1.0f; break;
      case ColorLabel::Green:  r = 0.1f; g = 0.8f; b = 0.2f; break;
      case ColorLabel::Orange: r = 1.0f; g = 0.55f; b = 0.0f; break;
      case ColorLabel::Red:    r = 1.0f; g = 0.1f; b = 0.1f; break;
      case ColorLabel::Yellow: r = 1.0f; g = 0.9f; b = 0.0f; break;
      default:                 r = 0.6f; g = 0.6f; b = 0.6f; break;
    }
  }

  void publish(const PlaceTarget & t)
  {
    // JSON summary
    std::ostringstream js;
    js << "{";
    js << "\"ok\":" << (t.ok ? "true" : "false");
    js << ",\"tof\":{\"present\":" << (t.tof.present ? "true" : "false")
       << ",\"distance_m\":" << t.tof.distance_m
       << ",\"reason\":\"" << t.tof.reason << "\"}";
    js << ",\"desk\":{\"ok\":" << (t.desk.ok ? "true" : "false")
       << ",\"z\":" << t.desk.z << ",\"tilt_deg\":" << t.desk.tilt_deg
       << ",\"inliers\":" << t.desk.inliers << "}";
    js << ",\"wall\":{\"ok\":" << (t.wall.ok ? "true" : "false")
       << ",\"front_distance\":" << t.wall.front_distance
       << ",\"width\":" << t.wall.width
       << ",\"visible_height\":" << t.wall.visible_height
       << ",\"yaw_deg\":" << t.wall.yaw_deg
       << ",\"nz_abs\":" << t.wall.nz_abs
       << ",\"centroid\":[" << t.wall.centroid[0] << "," << t.wall.centroid[1]
       << "," << t.wall.centroid[2] << "]"
       << ",\"normal\":[" << t.wall.normal[0] << "," << t.wall.normal[1]
       << "," << t.wall.normal[2] << "]"
       << ",\"inliers\":" << t.wall.inliers << "}";
    js << ",\"color\":{\"name\":\"" << t.color.name << "\",\"class_id\":"
       << t.color.class_id << ",\"confidence\":" << t.color.confidence
       << ",\"hsv\":[" << t.color.h << "," << t.color.s << "," << t.color.v
       << "]}";
    js << "}";
    std_msgs::msg::String info;
    info.data = js.str();
    pub_info_->publish(info);

    // Markers + cloud (only meaningful when the wall was found).
    if (!t.wall.ok) return;
    const auto stamp = now();

    visualization_msgs::msg::MarkerArray arr;
    float cr, cg, cb;
    colorRgba(t.color.label, cr, cg, cb);

    // Wall plane as a thin flat cube oriented by its normal.
    visualization_msgs::msg::Marker wall;
    wall.header.frame_id = body_frame_;
    wall.header.stamp = stamp;
    wall.ns = "place_wall";
    wall.id = 0;
    wall.type = visualization_msgs::msg::Marker::CUBE;
    wall.action = visualization_msgs::msg::Marker::ADD;
    wall.pose.position.x = t.wall.centroid[0];
    wall.pose.position.y = t.wall.centroid[1];
    wall.pose.position.z = t.wall.centroid[2];
    {
      // Orient cube +x to the wall normal.
      Eigen::Vector3f x = t.wall.normal.normalized();
      Eigen::Vector3f up(0, 0, 1);
      Eigen::Vector3f y = up.cross(x); y.normalize();
      Eigen::Vector3f z = x.cross(y);
      Eigen::Matrix3f R; R.col(0) = x; R.col(1) = y; R.col(2) = z;
      Eigen::Quaternionf q(R);
      wall.pose.orientation.x = q.x(); wall.pose.orientation.y = q.y();
      wall.pose.orientation.z = q.z(); wall.pose.orientation.w = q.w();
    }
    wall.scale.x = 0.01;
    wall.scale.y = std::max(0.02, t.wall.width);
    wall.scale.z = std::max(0.02, t.wall.visible_height);
    wall.color.r = cr; wall.color.g = cg; wall.color.b = cb; wall.color.a = 0.5f;
    arr.markers.push_back(wall);

    // Normal arrow.
    visualization_msgs::msg::Marker arrow;
    arrow.header = wall.header;
    arrow.ns = "place_wall_normal";
    arrow.id = 1;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    geometry_msgs::msg::Point p0, p1;
    p0.x = t.wall.centroid[0]; p0.y = t.wall.centroid[1]; p0.z = t.wall.centroid[2];
    p1.x = p0.x + t.wall.normal[0] * 0.15;
    p1.y = p0.y + t.wall.normal[1] * 0.15;
    p1.z = p0.z + t.wall.normal[2] * 0.15;
    arrow.points = {p0, p1};
    arrow.scale.x = 0.01; arrow.scale.y = 0.02; arrow.scale.z = 0.0;
    arrow.color.r = 1.0f; arrow.color.g = 0.9f; arrow.color.b = 0.1f;
    arrow.color.a = 0.9f;
    arr.markers.push_back(arrow);

    // Text label.
    visualization_msgs::msg::Marker txt;
    txt.header = wall.header;
    txt.ns = "place_wall_text";
    txt.id = 2;
    txt.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    txt.action = visualization_msgs::msg::Marker::ADD;
    txt.pose.position.x = t.wall.centroid[0];
    txt.pose.position.y = t.wall.centroid[1];
    txt.pose.position.z = t.wall.centroid[2] + 0.08;
    txt.scale.z = 0.04;
    txt.color.r = 1.0f; txt.color.g = 1.0f; txt.color.b = 1.0f; txt.color.a = 0.9f;
    {
      std::ostringstream ts;
      ts.precision(2); ts.setf(std::ios::fixed);
      ts << t.color.name << " @" << t.wall.front_distance << "m w"
         << t.wall.width << " h" << t.wall.visible_height;
      txt.text = ts.str();
    }
    arr.markers.push_back(txt);

    pub_markers_->publish(arr);

    // Wall inlier cloud.
    if (t.wall.inlier_cloud && !t.wall.inlier_cloud->empty()) {
      sensor_msgs::msg::PointCloud2 cmsg;
      pcl::toROSMsg(*t.wall.inlier_cloud, cmsg);
      cmsg.header.frame_id = body_frame_;
      cmsg.header.stamp = stamp;
      pub_cloud_->publish(cmsg);
    }
  }

  // params
  std::string body_frame_, cloud_topic_, tof_topic_;
  double period_s_;
  bool require_tof_;
  TofGateParams tof_p_;
  DeskParams desk_p_;
  WallParams wall_p_;
  ColorParams color_p_;

  // io
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr tof_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_info_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::PointCloud2::SharedPtr last_cloud_;
  sensor_msgs::msg::Range::SharedPtr last_tof_;
  rclcpp::Time last_tof_stamp_{0, 0, RCL_ROS_TIME};
};

}  // namespace place_box_detection

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<place_box_detection::PlaceBoxDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
