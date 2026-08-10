#include <cmath>
#include <vector>

#include "gtest/gtest.h"
#include "bt_nav2_plugins/rgbd_object_pose.hpp"

namespace pose = bt_nav2_plugins::rgbd_pose;

TEST(RgbdObjectPose, ForegroundDepthLimitRejectsSupportSurface)
{
  std::vector<double> depths;
  for (int index = 0; index < 80; ++index) {
    depths.push_back(0.60 + 0.0005 * static_cast<double>(index % 20));
  }
  for (int index = 0; index < 40; ++index) {
    depths.push_back(0.76 + 0.0005 * static_cast<double>(index % 10));
  }
  depths.push_back(0.31);  // isolated invalid short return

  const double limit = pose::foregroundDepthLimit(depths, 0.20);
  EXPECT_GT(limit, 0.60);
  EXPECT_LT(limit, 0.76);
}

TEST(RgbdObjectPose, SphereFitRecoversCenterFromVisibleHemisphere)
{
  constexpr double pi = 3.14159265358979323846;
  const pose::Point3 expected{0.62, -0.08, 0.405};
  constexpr double expected_radius = 0.035;
  std::vector<pose::Point3> points;
  for (int polar_index = 1; polar_index <= 12; ++polar_index) {
    const double polar = (0.5 * pi) * static_cast<double>(polar_index) / 13.0;
    for (int azimuth_index = 0; azimuth_index < 28; ++azimuth_index) {
      // A camera-visible cap is deliberately used rather than a full sphere.
      const double azimuth =
        -0.75 * pi + 1.5 * pi * static_cast<double>(azimuth_index) / 27.0;
      const double noise = 0.0004 * std::sin(3.0 * azimuth + polar);
      const double radius = expected_radius + noise;
      points.push_back({
        expected.x + radius * std::sin(polar) * std::cos(azimuth),
        expected.y + radius * std::sin(polar) * std::sin(azimuth),
        expected.z + radius * std::cos(polar)});
    }
  }

  pose::Point3 fitted;
  double radius = 0.0;
  ASSERT_TRUE(pose::fitSphereCenter(points, fitted, radius));
  EXPECT_NEAR(fitted.x, expected.x, 0.0015);
  EXPECT_NEAR(fitted.y, expected.y, 0.0015);
  EXPECT_NEAR(fitted.z, expected.z, 0.0015);
  EXPECT_NEAR(radius, expected_radius, 0.0015);
}

TEST(RgbdObjectPose, UprightCylinderFitDoesNotAdvanceDownCameraRay)
{
  constexpr double pi = 3.14159265358979323846;
  const pose::Point3 camera{0.0, 0.0, 1.15};
  const pose::Point3 expected{0.62, 0.04, 0.431};
  constexpr double expected_radius = 0.033;
  constexpr double expected_height = 0.122;
  const double view_angle = std::atan2(expected.y - camera.y, expected.x - camera.x);
  std::vector<pose::Point3> points;

  // Visible half of the cylindrical side wall.
  for (int angle_index = 0; angle_index <= 36; ++angle_index) {
    const double angle =
      view_angle + 0.5 * pi + pi * static_cast<double>(angle_index) / 36.0;
    for (int height_index = 0; height_index <= 18; ++height_index) {
      const double z =
        expected.z - expected_height * 0.5 +
        expected_height * static_cast<double>(height_index) / 18.0;
      points.push_back({
        expected.x + expected_radius * std::cos(angle),
        expected.y + expected_radius * std::sin(angle),
        z});
    }
  }

  pose::Point3 fitted;
  double radius = 0.0;
  double height = 0.0;
  ASSERT_TRUE(pose::fitVerticalRoundCenter(points, camera, fitted, radius, height));
  EXPECT_NEAR(fitted.x, expected.x, 0.003);
  EXPECT_NEAR(fitted.y, expected.y, 0.003);
  EXPECT_NEAR(fitted.z, expected.z, 0.003);
  EXPECT_NEAR(radius, expected_radius, 0.003);
  EXPECT_NEAR(height, expected_height, 0.006);
}

