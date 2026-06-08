#include "place_box_detection/tof_gate.hpp"

#include <cmath>

namespace place_box_detection
{

TofCandidate detectWallCandidateFromTof(double range_m, double age_s,
                                        const TofGateParams & params)
{
  TofCandidate c;

  if (!std::isfinite(range_m)) {
    c.reason = "no/invalid TOF reading";
    return c;
  }
  if (age_s >= params.stale_timeout_s) {
    c.reason = "TOF reading stale (" + std::to_string(age_s) + "s)";
    return c;
  }
  if (range_m < params.min_range_m) {
    c.reason = "too close (" + std::to_string(range_m) + "m) -> noise/self";
    return c;
  }
  if (range_m > params.max_range_m) {
    c.reason = "no wall within reach (" + std::to_string(range_m) + "m)";
    return c;
  }

  c.present = true;
  // Report the surface position in body-x (range + sensor forward mount offset),
  // so the cloud depth-gate centres on the wall's body-x directly.
  c.distance_m = range_m + params.mount_x_offset_m;
  c.reason = "wall candidate @ range " + std::to_string(range_m) + "m -> body_x "
             + std::to_string(c.distance_m) + "m";
  return c;
}

}  // namespace place_box_detection
