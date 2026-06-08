// Stage 3: color classification.
//
// Classifies the RGB of the detected wall inliers into one of the 5 fixed
// yolov8 mini-box-seg colors {blue, green, orange, red, yellow}. Self-contained
// (HSV voting) so it works before the big-box color is trained into yolov8 —
// the big box reports Unknown until its hue range is added.
#ifndef PLACE_BOX_DETECTION_COLOR_CLASSIFIER_HPP_
#define PLACE_BOX_DETECTION_COLOR_CLASSIFIER_HPP_

#include <cstdint>

#include "place_box_detection/types.hpp"

namespace place_box_detection
{

// Convert one 8-bit RGB triple to HSV. h in [0,360), s,v in [0,1].
void rgbToHsv(uint8_t r, uint8_t g, uint8_t b, double & h, double & s, double & v);

// Map a single HSV sample to one of the 5 colors (or Unknown).
ColorLabel hsvToColor(double h, double s, double v, const ColorParams & params);

// Classify a cloud of wall inlier points by majority HSV vote.
ColorResult classifyColor(const CloudT & inliers, const ColorParams & params);

}  // namespace place_box_detection

#endif  // PLACE_BOX_DETECTION_COLOR_CLASSIFIER_HPP_
