#include "place_box_detection/color_classifier.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace place_box_detection
{

void rgbToHsv(uint8_t r8, uint8_t g8, uint8_t b8, double & h, double & s, double & v)
{
  const double r = r8 / 255.0, g = g8 / 255.0, b = b8 / 255.0;
  const double mx = std::max({r, g, b});
  const double mn = std::min({r, g, b});
  const double d = mx - mn;
  v = mx;
  s = (mx <= 1e-9) ? 0.0 : d / mx;
  if (d <= 1e-9) {
    h = 0.0;
    return;
  }
  if (mx == r)      h = 60.0 * std::fmod((g - b) / d, 6.0);
  else if (mx == g) h = 60.0 * ((b - r) / d + 2.0);
  else              h = 60.0 * ((r - g) / d + 4.0);
  if (h < 0) h += 360.0;
}

ColorLabel hsvToColor(double h, double s, double v, const ColorParams & params)
{
  if (s < params.min_saturation || v < params.min_value) {
    return ColorLabel::Unknown;
  }
  // red/orange boundary data-tuned to 9.0 deg (was 15.0): on this D435, the
  // measured red box reads H~6.1 (p95 8.9) and orange H~10.3 (p05 8.3); a 15 deg
  // cut mislabeled orange as red. green(142)/blue(217)/yellow(~50) are far and
  // unaffected. NOTE: red/orange margin is thin (~4 deg, overlapping tails) —
  // re-tune with more samples if lighting/box changes.
  // Data-tuned on this D435 from 5 measured boxes (H_med):
  //   red 6.1, orange 10.3, yellow 46.7, green 142, blue 217.
  // red/orange split = 9 (overlapping tails, ~4 deg margin -> the tight one);
  // orange/yellow split = 30 (centred in the empty 13-45 gap for margin).
  if (h >= 345.0 || h < 9.0)   return ColorLabel::Red;
  if (h < 30.0)                return ColorLabel::Orange;
  if (h < 70.0)                return ColorLabel::Yellow;
  if (h < 170.0)               return ColorLabel::Green;
  if (h < 265.0)               return ColorLabel::Blue;
  return ColorLabel::Unknown;  // magenta/violet band -> future big-box color
}

ColorResult classifyColor(const CloudT & inliers, const ColorParams & params)
{
  ColorResult res;
  const size_t n = inliers.size();
  if (n == 0) return res;

  const size_t stride =
    std::max<size_t>(1, n / static_cast<size_t>(std::max(1, params.subsample_max)));

  std::array<int, 5> votes{0, 0, 0, 0, 0};
  std::array<double, 5> sin_sum{0, 0, 0, 0, 0};
  std::array<double, 5> cos_sum{0, 0, 0, 0, 0};
  std::array<double, 5> s_sum{0, 0, 0, 0, 0};
  std::array<double, 5> v_sum{0, 0, 0, 0, 0};
  int scanned = 0;

  for (size_t i = 0; i < n; i += stride) {
    const PointT & p = inliers[i];
    double h, s, v;
    rgbToHsv(p.r, p.g, p.b, h, s, v);
    ++scanned;
    ColorLabel c = hsvToColor(h, s, v, params);
    if (c == ColorLabel::Unknown) continue;
    const int k = static_cast<int>(c);
    ++votes[k];
    const double rad = h * M_PI / 180.0;
    sin_sum[k] += std::sin(rad);
    cos_sum[k] += std::cos(rad);
    s_sum[k] += s;
    v_sum[k] += v;
  }
  if (scanned == 0) return res;

  int best = -1, best_votes = 0;
  for (int k = 0; k < 5; ++k) {
    if (votes[k] > best_votes) { best_votes = votes[k]; best = k; }
  }
  if (best < 0) return res;  // all unknown

  const double conf = static_cast<double>(best_votes) / scanned;
  if (conf < params.min_confidence) {
    res.confidence = conf;  // report it, but stay Unknown
    return res;
  }

  res.label = static_cast<ColorLabel>(best);
  res.class_id = best;
  res.name = toString(res.label);
  res.confidence = conf;
  double mean_h = std::atan2(sin_sum[best], cos_sum[best]) * 180.0 / M_PI;
  if (mean_h < 0) mean_h += 360.0;
  res.h = mean_h;
  res.s = s_sum[best] / best_votes;
  res.v = v_sum[best] / best_votes;
  return res;
}

}  // namespace place_box_detection
