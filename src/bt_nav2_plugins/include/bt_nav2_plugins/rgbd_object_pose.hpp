#ifndef BT_NAV2_PLUGINS__RGBD_OBJECT_POSE_HPP_
#define BT_NAV2_PLUGINS__RGBD_OBJECT_POSE_HPP_

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace bt_nav2_plugins::rgbd_pose
{

struct Point3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

inline double percentile(std::vector<double> values, double fraction)
{
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  fraction = std::clamp(fraction, 0.0, 1.0);
  const double index = fraction * static_cast<double>(values.size() - 1);
  const auto lower = static_cast<std::size_t>(std::floor(index));
  const auto upper = static_cast<std::size_t>(std::ceil(index));
  std::nth_element(values.begin(), values.begin() + lower, values.end());
  const double lower_value = values[lower];
  if (upper == lower) {
    return lower_value;
  }
  std::nth_element(values.begin(), values.begin() + upper, values.end());
  return lower_value + (values[upper] - lower_value) * (index - static_cast<double>(lower));
}

// Keep the coherent foreground range while rejecting isolated short returns and
// the support/background behind the detected object.  A real depth discontinuity
// wins; otherwise the caller-provided object span is used.
inline double foregroundDepthLimit(
  std::vector<double> depths,
  double maximum_object_depth_span)
{
  if (depths.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  std::sort(depths.begin(), depths.end());
  const std::size_t count = depths.size();
  const std::size_t near_index = std::min(count - 1, count / 20);  // fifth percentile
  const double near_depth = depths[near_index];
  const double span = std::clamp(maximum_object_depth_span, 0.03, 0.20);
  const double span_limit = near_depth + span;

  const std::size_t minimum_cluster = std::max<std::size_t>(4, count / 10);
  double largest_gap = 0.0;
  double gap_limit = span_limit;
  if (count > 2 * minimum_cluster) {
    for (std::size_t i = minimum_cluster; i + minimum_cluster < count; ++i) {
      if (depths[i] > span_limit) {
        break;
      }
      const double gap = depths[i] - depths[i - 1];
      if (gap > largest_gap) {
        largest_gap = gap;
        gap_limit = 0.5 * (depths[i] + depths[i - 1]);
      }
    }
  }

  // RGB-D depth noise is millimetric at manipulation range.  A 15 mm empty
  // interval is therefore geometry, not noise; smaller gaps should not split a
  // curved object surface into artificial front/back halves.
  return largest_gap >= 0.015 ? gap_limit : span_limit;
}

inline bool robustBoundsCenter(
  const std::vector<Point3> & points,
  Point3 & center,
  Point3 & dimensions)
{
  if (points.size() < 8) {
    return false;
  }
  std::vector<double> xs;
  std::vector<double> ys;
  std::vector<double> zs;
  xs.reserve(points.size());
  ys.reserve(points.size());
  zs.reserve(points.size());
  for (const auto & point : points) {
    xs.push_back(point.x);
    ys.push_back(point.y);
    zs.push_back(point.z);
  }
  const double x_low = percentile(xs, 0.02);
  const double x_high = percentile(xs, 0.98);
  const double y_low = percentile(ys, 0.02);
  const double y_high = percentile(ys, 0.98);
  const double z_low = percentile(zs, 0.02);
  const double z_high = percentile(zs, 0.98);
  center = {(x_low + x_high) * 0.5, (y_low + y_high) * 0.5, (z_low + z_high) * 0.5};
  dimensions = {x_high - x_low, y_high - y_low, z_high - z_low};
  return std::isfinite(center.x) && std::isfinite(center.y) && std::isfinite(center.z);
}

// Algebraic least-squares sphere fitting is a conventional model fit for a
// segmented RGB-D cloud.  It recovers the volume center instead of the biased
// centroid of the camera-visible hemisphere.
inline bool fitSphereCenter(
  const std::vector<Point3> & points,
  Point3 & center,
  double & radius)
{
  if (points.size() < 16) {
    return false;
  }
  cv::Mat system(static_cast<int>(points.size()), 4, CV_64F);
  cv::Mat observations(static_cast<int>(points.size()), 1, CV_64F);
  for (std::size_t i = 0; i < points.size(); ++i) {
    const auto & point = points[i];
    system.at<double>(static_cast<int>(i), 0) = 2.0 * point.x;
    system.at<double>(static_cast<int>(i), 1) = 2.0 * point.y;
    system.at<double>(static_cast<int>(i), 2) = 2.0 * point.z;
    system.at<double>(static_cast<int>(i), 3) = 1.0;
    observations.at<double>(static_cast<int>(i), 0) =
      point.x * point.x + point.y * point.y + point.z * point.z;
  }
  cv::Mat solution;
  if (!cv::solve(system, observations, solution, cv::DECOMP_SVD)) {
    return false;
  }
  center = {
    solution.at<double>(0, 0),
    solution.at<double>(1, 0),
    solution.at<double>(2, 0)};
  const double radius_squared =
    solution.at<double>(3, 0) + center.x * center.x +
    center.y * center.y + center.z * center.z;
  if (!std::isfinite(radius_squared) || radius_squared <= 0.0) {
    return false;
  }
  radius = std::sqrt(radius_squared);
  if (radius < 0.01 || radius > 0.20) {
    return false;
  }

  double squared_error = 0.0;
  for (const auto & point : points) {
    const double distance = std::sqrt(
      std::pow(point.x - center.x, 2) +
      std::pow(point.y - center.y, 2) +
      std::pow(point.z - center.z, 2));
    squared_error += std::pow(distance - radius, 2);
  }
  const double rms_error = std::sqrt(squared_error / static_cast<double>(points.size()));
  return std::isfinite(rms_error) && rms_error <= std::max(0.008, radius * 0.20);
}

// For an upright round object, recover the hidden axis from its visible front
// surface and its lateral silhouette.  This is the geometric cylinder model
// used instead of advancing along the downward camera ray (which also changes Z).
inline bool fitVerticalRoundCenter(
  const std::vector<Point3> & points,
  const Point3 & camera_origin,
  Point3 & center,
  double & radius,
  double & height)
{
  if (points.size() < 16) {
    return false;
  }
  double mean_x = 0.0;
  double mean_y = 0.0;
  for (const auto & point : points) {
    mean_x += point.x;
    mean_y += point.y;
  }
  mean_x /= static_cast<double>(points.size());
  mean_y /= static_cast<double>(points.size());
  double direction_x = mean_x - camera_origin.x;
  double direction_y = mean_y - camera_origin.y;
  const double direction_norm = std::hypot(direction_x, direction_y);
  if (direction_norm < 1e-6) {
    return false;
  }
  direction_x /= direction_norm;
  direction_y /= direction_norm;
  const double lateral_x = -direction_y;
  const double lateral_y = direction_x;

  std::vector<double> forward;
  std::vector<double> lateral;
  std::vector<double> vertical;
  forward.reserve(points.size());
  lateral.reserve(points.size());
  vertical.reserve(points.size());
  for (const auto & point : points) {
    forward.push_back(point.x * direction_x + point.y * direction_y);
    lateral.push_back(point.x * lateral_x + point.y * lateral_y);
    vertical.push_back(point.z);
  }
  const double lateral_low = percentile(lateral, 0.02);
  const double lateral_high = percentile(lateral, 0.98);
  radius = 0.5 * (lateral_high - lateral_low);
  const double center_lateral = 0.5 * (lateral_low + lateral_high);
  const double center_forward = percentile(forward, 0.02) + radius;
  const double z_low = percentile(vertical, 0.02);
  const double z_high = percentile(vertical, 0.98);
  height = z_high - z_low;
  center = {
    center_forward * direction_x + center_lateral * lateral_x,
    center_forward * direction_y + center_lateral * lateral_y,
    0.5 * (z_low + z_high)};
  return std::isfinite(center.x) && std::isfinite(center.y) && std::isfinite(center.z) &&
         radius >= 0.01 && radius <= 0.15 && height >= 0.02 && height <= 0.40;
}

inline std::string lowercase(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char character) {return static_cast<char>(std::tolower(character));});
  return value;
}

}  // namespace bt_nav2_plugins::rgbd_pose

#endif  // BT_NAV2_PLUGINS__RGBD_OBJECT_POSE_HPP_
