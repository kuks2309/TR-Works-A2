#include "place_box_detection/wall_detector.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <pcl/ModelCoefficients.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace place_box_detection
{

namespace
{
constexpr double kDeg2Rad = M_PI / 180.0;

// Pass-through filter on one axis ("x","y","z"); returns the kept subset.
CloudT::Ptr passThrough(const CloudT::Ptr & in, const std::string & axis,
                        double lo, double hi)
{
  CloudT::Ptr out(new CloudT);
  pcl::PassThrough<PointT> pt;
  pt.setInputCloud(in);
  pt.setFilterFieldName(axis);
  pt.setFilterLimits(static_cast<float>(lo), static_cast<float>(hi));
  pt.filter(*out);
  return out;
}

double medianOf(std::vector<float> & v)
{
  if (v.empty()) return 0.0;
  const size_t n = v.size() / 2;
  std::nth_element(v.begin(), v.begin() + n, v.end());
  return v[n];
}
}  // namespace

CloudT::Ptr transformToBody(const CloudT & cloud_cam,
                            const Eigen::Affine3f & body_T_cam)
{
  CloudT::Ptr out(new CloudT);
  pcl::transformPointCloud(cloud_cam, *out, body_T_cam);
  return out;
}

bool detectDeskPlane(const CloudT & cloud_body, const DeskParams & params,
                     DeskPlane & out)
{
  out = DeskPlane{};
  CloudT::Ptr in(new CloudT(cloud_body));
  CloudT::Ptr band = passThrough(in, "z", params.z_min, params.z_max);
  if (static_cast<int>(band->size()) < params.min_inliers) {
    return false;
  }

  pcl::SACSegmentation<PointT> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);  // normal ~ given axis
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setAxis(Eigen::Vector3f(0, 0, 1));                // desk normal is +z
  seg.setEpsAngle(params.axis_eps_deg * kDeg2Rad);
  seg.setDistanceThreshold(params.ransac_thresh);
  seg.setMaxIterations(params.max_iterations);

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coef(new pcl::ModelCoefficients);
  seg.setInputCloud(band);
  seg.segment(*inliers, *coef);
  if (static_cast<int>(inliers->indices.size()) < params.min_inliers) {
    return false;
  }

  Eigen::Vector4f c(coef->values[0], coef->values[1], coef->values[2],
                    coef->values[3]);
  if (c[2] < 0) c = -c;                                  // orient normal +z

  Eigen::Vector3f centroid(0, 0, 0);
  for (int idx : inliers->indices) {
    const PointT & p = (*band)[idx];
    centroid += Eigen::Vector3f(p.x, p.y, p.z);
  }
  centroid /= static_cast<float>(inliers->indices.size());

  out.ok = true;
  out.coef = c;
  out.centroid = centroid;
  out.z = centroid[2];
  out.tilt_deg = std::acos(std::min(1.0, std::abs(static_cast<double>(c[2])))) /
                 kDeg2Rad;
  out.inliers = static_cast<int>(inliers->indices.size());
  return true;
}

bool detectVerticalWall(const CloudT & cloud_body, const DeskPlane & desk,
                        const TofCandidate & tof, const WallParams & params,
                        WallPlane & out)
{
  out = WallPlane{};
  if (!desk.ok) return false;

  // Band: everything above the desk top (front face of the standing wall).
  CloudT::Ptr in(new CloudT(cloud_body));
  CloudT::Ptr above = passThrough(in, "z", desk.z + params.height_margin, 1e3);

  // Optional TOF distance gate in x (forward).
  if (params.use_depth_gate && tof.present) {
    above = passThrough(above, "x", tof.distance_m - params.depth_gate_tol,
                        tof.distance_m + params.depth_gate_tol);
  }
  if (static_cast<int>(above->size()) < params.min_inliers) {
    return false;
  }

  pcl::SACSegmentation<PointT> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);  // plane parallel to axis = vertical
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setAxis(Eigen::Vector3f(0, 0, 1));           // parallel to body +z -> upright
  seg.setEpsAngle(params.axis_eps_deg * kDeg2Rad);
  seg.setDistanceThreshold(params.ransac_thresh);
  seg.setMaxIterations(params.max_iterations);

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coef(new pcl::ModelCoefficients);
  seg.setInputCloud(above);
  seg.segment(*inliers, *coef);
  if (static_cast<int>(inliers->indices.size()) < params.min_inliers) {
    return false;
  }

  Eigen::Vector4f c(coef->values[0], coef->values[1], coef->values[2],
                    coef->values[3]);
  Eigen::Vector3f n(c[0], c[1], c[2]);
  const double nn = n.norm();
  if (nn < 1e-9) return false;
  n /= nn;
  c /= static_cast<float>(nn);

  // Reject non-upright planes (e.g. a residual desk slab).
  const double nz_abs = std::abs(static_cast<double>(n[2]));
  if (nz_abs > params.max_nz) {
    return false;
  }

  // Orient the normal toward the robot (-x).
  if (n[0] > 0) { n = -n; c = -c; }
  if (params.require_faces_robot && n[0] >= 0) {
    return false;
  }

  // Gather inlier geometry.
  CloudT::Ptr wall(new CloudT);
  wall->reserve(inliers->indices.size());
  std::vector<float> xs;
  xs.reserve(inliers->indices.size());
  float ymin = 1e9f, ymax = -1e9f, zmin = 1e9f, zmax = -1e9f;
  Eigen::Vector3f centroid(0, 0, 0);
  for (int idx : inliers->indices) {
    const PointT & p = (*above)[idx];
    wall->push_back(p);
    xs.push_back(p.x);
    ymin = std::min(ymin, p.y); ymax = std::max(ymax, p.y);
    zmin = std::min(zmin, p.z); zmax = std::max(zmax, p.z);
    centroid += Eigen::Vector3f(p.x, p.y, p.z);
  }
  centroid /= static_cast<float>(wall->size());

  const double width = ymax - ymin;
  const double visible_height = zmax - zmin;
  if (width < params.min_width || visible_height < params.min_visible_height) {
    return false;
  }

  out.ok = true;
  out.coef = c;
  out.normal = n;
  out.centroid = centroid;
  out.front_distance = medianOf(xs);
  out.width = width;
  out.visible_height = visible_height;
  out.yaw_deg = std::atan2(static_cast<double>(n[1]),
                           static_cast<double>(n[0])) / kDeg2Rad;
  out.nz_abs = nz_abs;
  out.inliers = static_cast<int>(wall->size());
  out.inlier_cloud = wall;
  return true;
}

}  // namespace place_box_detection
