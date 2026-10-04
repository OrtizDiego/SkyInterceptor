#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <vector>

#include "perception/groundtruth_sensor.hpp"

namespace interceptor
{
namespace
{

OcclusionCylinder cylinder(double x, double y, double radius, double height)
{
  OcclusionCylinder c;
  c.center = Eigen::Vector2d(x, y);
  c.radius = radius;
  c.height = height;
  return c;
}

// --- Occlusion ------------------------------------------------------------

TEST(OcclusionTest, ParsesFlatCylinderList)
{
  const auto cylinders = parseOcclusionCylinders({20.0, 7.0, 4.0, 6.5, 4.0, -5.0, 1.6, 5.1});
  ASSERT_EQ(cylinders.size(), 2u);
  EXPECT_DOUBLE_EQ(cylinders[0].center.x(), 20.0);
  EXPECT_DOUBLE_EQ(cylinders[0].center.y(), 7.0);
  EXPECT_DOUBLE_EQ(cylinders[0].radius, 4.0);
  EXPECT_DOUBLE_EQ(cylinders[0].height, 6.5);
  EXPECT_DOUBLE_EQ(cylinders[1].center.y(), -5.0);
  EXPECT_TRUE(parseOcclusionCylinders({}).empty());
}

TEST(OcclusionTest, RejectsMalformedCylinderList)
{
  EXPECT_THROW(parseOcclusionCylinders({1.0, 2.0, 3.0}), std::invalid_argument);
  EXPECT_THROW(parseOcclusionCylinders({0.0, 0.0, 0.0, 5.0}), std::invalid_argument);
  EXPECT_THROW(parseOcclusionCylinders({0.0, 0.0, 1.0, -1.0}), std::invalid_argument);
}

TEST(OcclusionTest, SegmentThroughCylinderIsBlocked)
{
  const auto tree = cylinder(0.0, 0.0, 1.0, 5.0);
  EXPECT_TRUE(segmentIntersectsCylinder({-5.0, 0.0, 1.0}, {5.0, 0.0, 1.0}, tree));
  // Diagonal, climbing from ground level to above the drone
  EXPECT_TRUE(segmentIntersectsCylinder({-5.0, -5.0, 0.5}, {5.0, 5.0, 10.0}, tree));
}

TEST(OcclusionTest, SegmentBesideCylinderIsClear)
{
  const auto tree = cylinder(0.0, 0.0, 1.0, 5.0);
  EXPECT_FALSE(segmentIntersectsCylinder({-5.0, 1.5, 1.0}, {5.0, 1.5, 1.0}, tree));
  EXPECT_FALSE(segmentIntersectsCylinder({-5.0, -1.5, 1.0}, {5.0, -1.5, 1.0}, tree));
}

TEST(OcclusionTest, TangentSegmentIsBlocked)
{
  const auto tree = cylinder(0.0, 0.0, 1.0, 5.0);
  EXPECT_TRUE(segmentIntersectsCylinder({-5.0, 1.0, 1.0}, {5.0, 1.0, 1.0}, tree));
}

TEST(OcclusionTest, SegmentOverTheTopIsClear)
{
  const auto tree = cylinder(0.0, 0.0, 1.0, 5.0);
  EXPECT_FALSE(segmentIntersectsCylinder({-5.0, 0.0, 6.0}, {5.0, 0.0, 6.0}, tree));
  // Descends steeply but only reaches canopy height after passing the cylinder
  EXPECT_FALSE(segmentIntersectsCylinder({-2.0, 0.0, 20.0}, {3.0, 0.0, 0.0}, tree));
}

TEST(OcclusionTest, SegmentEndingShortOfCylinderIsClear)
{
  const auto tree = cylinder(10.0, 0.0, 1.0, 5.0);
  EXPECT_FALSE(segmentIntersectsCylinder({0.0, 0.0, 1.0}, {8.0, 0.0, 1.0}, tree));
  // The same line, but now the target stands behind the tree
  EXPECT_TRUE(segmentIntersectsCylinder({0.0, 0.0, 1.0}, {12.0, 0.0, 1.0}, tree));
}

TEST(OcclusionTest, EndpointInsideCylinderIsBlocked)
{
  const auto tree = cylinder(0.0, 0.0, 2.0, 5.0);
  EXPECT_TRUE(segmentIntersectsCylinder({0.5, 0.0, 1.0}, {10.0, 0.0, 1.0}, tree));
}

TEST(OcclusionTest, VerticalSegment)
{
  const auto tree = cylinder(0.0, 0.0, 1.0, 5.0);
  EXPECT_TRUE(segmentIntersectsCylinder({0.2, 0.0, 10.0}, {0.2, 0.0, 1.0}, tree));
  EXPECT_FALSE(segmentIntersectsCylinder({0.2, 0.0, 10.0}, {0.2, 0.0, 6.0}, tree));
  EXPECT_FALSE(segmentIntersectsCylinder({3.0, 0.0, 10.0}, {3.0, 0.0, 1.0}, tree));
}

TEST(OcclusionTest, LineOfSightChecksEveryCylinder)
{
  const Eigen::Vector3d drone(0.0, 0.0, 5.0);
  const Eigen::Vector3d person(20.0, 0.0, 1.0);
  EXPECT_FALSE(isLineOfSightBlocked(drone, person, {}));
  EXPECT_FALSE(isLineOfSightBlocked(drone, person, {cylinder(10.0, 5.0, 1.0, 8.0)}));
  EXPECT_TRUE(
    isLineOfSightBlocked(
      drone, person, {cylinder(10.0, 5.0, 1.0, 8.0), cylinder(10.0, 0.5, 1.0, 8.0)}));
}

// --- Noise and dropout ----------------------------------------------------

GroundTruthSensorConfig sensorConfig(double noise_std, double dropout_prob)
{
  GroundTruthSensorConfig config;
  config.pos_noise_std = noise_std;
  config.dropout_prob = dropout_prob;
  return config;
}

TEST(GroundTruthSensorTest, RejectsInvalidConfig)
{
  EXPECT_THROW(GroundTruthSensor(sensorConfig(-0.1, 0.0), 1), std::invalid_argument);
  EXPECT_THROW(GroundTruthSensor(sensorConfig(0.2, -0.1), 1), std::invalid_argument);
  EXPECT_THROW(GroundTruthSensor(sensorConfig(0.2, 1.1), 1), std::invalid_argument);
  EXPECT_THROW(GroundTruthSensor(sensorConfig(std::nan(""), 0.0), 1), std::invalid_argument);
}

TEST(GroundTruthSensorTest, NoNoiseNoDropoutIsExact)
{
  GroundTruthSensor sensor(sensorConfig(0.0, 0.0), 1);
  const Eigen::Vector3d truth(3.0, -2.0, 1.0);
  for (int i = 0; i < 100; ++i) {
    const auto measured = sensor.measure(truth);
    ASSERT_TRUE(measured.has_value());
    EXPECT_EQ(*measured, truth);
  }
}

TEST(GroundTruthSensorTest, DropoutExtremes)
{
  GroundTruthSensor never(sensorConfig(0.2, 0.0), 1);
  GroundTruthSensor always(sensorConfig(0.2, 1.0), 1);
  for (int i = 0; i < 1000; ++i) {
    EXPECT_TRUE(never.measure(Eigen::Vector3d::Zero()).has_value());
    EXPECT_FALSE(always.measure(Eigen::Vector3d::Zero()).has_value());
  }
}

TEST(GroundTruthSensorTest, DropoutRateMatchesProbability)
{
  const double p = 0.3;
  const int n = 20000;
  GroundTruthSensor sensor(sensorConfig(0.2, p), 42);
  int dropped = 0;
  for (int i = 0; i < n; ++i) {
    if (!sensor.measure(Eigen::Vector3d::Zero())) {
      ++dropped;
    }
  }
  // Binomial standard deviation of the rate is sqrt(p (1 - p) / n) ~ 0.003; allow 5 sigma
  EXPECT_NEAR(static_cast<double>(dropped) / n, p, 5.0 * std::sqrt(p * (1.0 - p) / n));
}

TEST(GroundTruthSensorTest, NoiseIsZeroMeanWithConfiguredStd)
{
  const double sigma = 0.2;
  const int n = 20000;
  const Eigen::Vector3d truth(10.0, 5.0, 1.0);
  GroundTruthSensor sensor(sensorConfig(sigma, 0.0), 7);

  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  Eigen::Vector3d sum_sq = Eigen::Vector3d::Zero();
  for (int i = 0; i < n; ++i) {
    const Eigen::Vector3d error = *sensor.measure(truth) - truth;
    sum += error;
    sum_sq += error.cwiseProduct(error);
  }
  const Eigen::Vector3d mean = sum / n;
  const Eigen::Vector3d std_dev = (sum_sq / n - mean.cwiseProduct(mean)).cwiseSqrt();
  for (int axis = 0; axis < 3; ++axis) {
    // Standard error of the mean is sigma / sqrt(n) ~ 0.0014 m
    EXPECT_NEAR(mean[axis], 0.0, 5.0 * sigma / std::sqrt(n)) << "axis " << axis;
    EXPECT_NEAR(std_dev[axis], sigma, 0.05 * sigma) << "axis " << axis;
  }
}

TEST(GroundTruthSensorTest, SameSeedIsReproducible)
{
  GroundTruthSensor a(sensorConfig(0.2, 0.2), 123);
  GroundTruthSensor b(sensorConfig(0.2, 0.2), 123);
  GroundTruthSensor c(sensorConfig(0.2, 0.2), 124);
  bool differs_from_c = false;
  for (int i = 0; i < 200; ++i) {
    const auto ma = a.measure(Eigen::Vector3d::Zero());
    const auto mb = b.measure(Eigen::Vector3d::Zero());
    const auto mc = c.measure(Eigen::Vector3d::Zero());
    ASSERT_EQ(ma.has_value(), mb.has_value());
    if (ma) {
      EXPECT_EQ(*ma, *mb);
    }
    differs_from_c |= ma.has_value() != mc.has_value() || (ma && mc && *ma != *mc);
  }
  EXPECT_TRUE(differs_from_c);
}

}  // namespace
}  // namespace interceptor
