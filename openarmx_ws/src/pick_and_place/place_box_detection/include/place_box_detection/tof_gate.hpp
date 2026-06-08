// Stage 1: TOF wall-candidate gate.
//
// The VL53L0X is a single-point laser ranger. It cannot describe a plane; it
// only answers "is something at distance d in front of me?". We use it as a
// cheap trigger: if a plausible standoff is measured, hand that distance to the
// cloud stage so the heavier RANSAC only runs when a wall is likely present.
#ifndef PLACE_BOX_DETECTION_TOF_GATE_HPP_
#define PLACE_BOX_DETECTION_TOF_GATE_HPP_

#include "place_box_detection/types.hpp"

namespace place_box_detection
{

// Decide whether a single TOF range reading represents a wall candidate.
//   range_m   : latest measured distance in meters (NaN/inf -> not present)
//   age_s     : how old the reading is, in seconds (>= stale_timeout -> reject)
// Returns a TofCandidate carrying presence, the standoff distance, and a reason.
TofCandidate detectWallCandidateFromTof(double range_m, double age_s,
                                        const TofGateParams & params);

}  // namespace place_box_detection

#endif  // PLACE_BOX_DETECTION_TOF_GATE_HPP_
