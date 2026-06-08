// Shared data types for the place-box detection pipeline.
//
// Pipeline (per the design):
//   Stage 1  TOF gate         -> TofCandidate  (wall candidate present + distance)
//   Stage 2a desk plane       -> DeskPlane     (horizontal support, ~0.726 m)
//   Stage 2b vertical wall    -> WallPlane     (upright face above the desk)
//   Stage 3  color classify   -> ColorResult   (one of the 5 yolov8 mini-box colors)
//
// All geometry is expressed in the BODY frame (openarmx_body_link0):
//   x = forward, y = left, z = up.
#ifndef PLACE_BOX_DETECTION_TYPES_HPP_
#define PLACE_BOX_DETECTION_TYPES_HPP_

#include <string>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace place_box_detection
{

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;

// ----------------------------- Stage 1: TOF -------------------------------
struct TofGateParams
{
  double min_range_m = 0.30;   // closer than this is treated as noise/self
  double max_range_m = 1.20;   // farther than this is "no wall in reach"
  double stale_timeout_s = 0.5;  // a reading older than this is ignored
  // TOF sensor X position in the body frame, added to the raw range to get the
  // surface body-x. Cross-validated against the D435 cloud (Δ<1mm), the raw TOF
  // range already equals the wall body-x, so the effective offset is 0.
  double mount_x_offset_m = 0.0;
};

struct TofCandidate
{
  bool present = false;        // is a plausible wall candidate in range?
  double distance_m = 0.0;     // measured standoff to the candidate
  std::string reason;          // human-readable explanation (logging/UI)
};

// ----------------------------- Stage 2a: desk -----------------------------
struct DeskParams
{
  double z_min = 0.55;         // body-z search band for the desk top
  double z_max = 0.85;
  double ransac_thresh = 0.006;
  double axis_eps_deg = 12.0;  // max tilt of desk normal from body +z
  int    min_inliers = 2000;
  int    max_iterations = 300;
};

struct DeskPlane
{
  bool ok = false;
  Eigen::Vector4f coef{0, 0, 1, 0};   // ax+by+cz+d=0, normal oriented +z
  Eigen::Vector3f centroid{0, 0, 0};
  double z = 0.0;                      // desk-top height (body z)
  double tilt_deg = 0.0;              // normal deviation from vertical
  int inliers = 0;
};

// ----------------------------- Stage 2b: wall -----------------------------
struct WallParams
{
  double height_margin = 0.03;   // start the wall band this far above desk top
  double depth_gate_tol = 0.10;  // keep points within +/- this of TOF distance (x)
  bool   use_depth_gate = true;  // gate the cloud by the TOF candidate distance
  double ransac_thresh = 0.008;
  double axis_eps_deg = 15.0;    // wall must be parallel to body +z within this
  double max_nz = 0.20;          // |normal_z| upper bound -> upright face
  double min_width = 0.10;       // min visible width (y extent) to accept
  double min_visible_height = 0.06;  // min visible height; NOT the true height
                                     // (top is clipped by camera tilt)
  int    min_inliers = 800;
  int    max_iterations = 400;
  bool   require_faces_robot = true; // accept only walls whose normal points -x
};

struct WallPlane
{
  bool ok = false;
  Eigen::Vector4f coef{1, 0, 0, 0};
  Eigen::Vector3f normal{-1, 0, 0};   // unit, oriented toward the robot (-x)
  Eigen::Vector3f centroid{0, 0, 0};  // visible-face centroid (body)
  double front_distance = 0.0;        // body-x of the front face (place ref)
  double width = 0.0;                 // visible width (y extent)
  double visible_height = 0.0;        // visible height above desk (FOV-clipped)
  double yaw_deg = 0.0;               // normal heading in body xy (0 = faces robot)
  double nz_abs = 0.0;               // |normal_z| (0 = perfectly upright)
  int inliers = 0;
  CloudT::Ptr inlier_cloud;           // for the color stage
};

// ----------------------------- Stage 3: color -----------------------------
// The 5 fixed yolov8 mini-box-seg classes (training order): blue, green,
// orange, red, yellow. The big place-box color is added later (id 5+).
enum class ColorLabel { Unknown = -1, Blue = 0, Green = 1, Orange = 2, Red = 3, Yellow = 4 };

struct ColorParams
{
  double min_saturation = 0.25;  // below -> washed out / gray -> Unknown
  double min_value = 0.15;       // below -> too dark -> Unknown
  double min_confidence = 0.40;  // dominant-color vote fraction to accept
  int    subsample_max = 4000;   // cap points scanned for speed
};

struct ColorResult
{
  ColorLabel label = ColorLabel::Unknown;
  int class_id = -1;             // matches yolov8 class id (0..4), -1 unknown
  std::string name = "unknown";
  double confidence = 0.0;       // fraction of inliers voting the dominant color
  double h = 0.0, s = 0.0, v = 0.0;  // mean HSV of the dominant-color points
};

inline std::string toString(ColorLabel c)
{
  switch (c) {
    case ColorLabel::Blue:   return "blue";
    case ColorLabel::Green:  return "green";
    case ColorLabel::Orange: return "orange";
    case ColorLabel::Red:    return "red";
    case ColorLabel::Yellow: return "yellow";
    default:                 return "unknown";
  }
}

// ----------------------------- Aggregate ----------------------------------
struct PlaceTarget
{
  TofCandidate tof;
  DeskPlane desk;
  WallPlane wall;
  ColorResult color;
  bool ok = false;   // full pipeline succeeded (tof + desk + wall)
};

}  // namespace place_box_detection

#endif  // PLACE_BOX_DETECTION_TYPES_HPP_
