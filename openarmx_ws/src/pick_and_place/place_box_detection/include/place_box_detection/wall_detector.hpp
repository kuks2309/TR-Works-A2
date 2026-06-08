// Stage 2: cloud-based plane detection (body frame).
//
//   detectDeskPlane     -> the horizontal support surface (~0.726 m). PCL
//                          SACMODEL_PERPENDICULAR_PLANE, normal forced near +z.
//   detectVerticalWall  -> the upright place wall above the desk. PCL
//                          SACMODEL_PARALLEL_PLANE (plane parallel to +z axis =
//                          vertical), optionally gated by the TOF distance.
//
// Both expect a cloud already expressed in the body frame. transformToBody is a
// convenience for callers holding a camera-frame cloud + TF.
#ifndef PLACE_BOX_DETECTION_WALL_DETECTOR_HPP_
#define PLACE_BOX_DETECTION_WALL_DETECTOR_HPP_

#include <Eigen/Geometry>

#include "place_box_detection/types.hpp"

namespace place_box_detection
{

// Transform a camera-frame cloud into the body frame using a body<-camera affine.
CloudT::Ptr transformToBody(const CloudT & cloud_cam,
                            const Eigen::Affine3f & body_T_cam);

// Stage 2a: fit the desk top. Returns true and fills `out` on success.
bool detectDeskPlane(const CloudT & cloud_body, const DeskParams & params,
                     DeskPlane & out);

// Stage 2b: fit the upright wall sitting above `desk.z`. The TOF candidate (when
// present and params.use_depth_gate) restricts the search to points near the
// measured standoff in x. Returns true and fills `out` on success.
bool detectVerticalWall(const CloudT & cloud_body, const DeskPlane & desk,
                        const TofCandidate & tof, const WallParams & params,
                        WallPlane & out);

}  // namespace place_box_detection

#endif  // PLACE_BOX_DETECTION_WALL_DETECTOR_HPP_
