#include <gtest/gtest.h>

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "common/parameters.hpp"

namespace interceptor
{
namespace
{

class ParametersTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

TEST_F(ParametersTest, LoadsDefaultsWhenNoOverrides)
{
  auto node = std::make_shared<rclcpp::Node>("parameters_defaults_test");
  Parameters::loadFromNode(node.get());

  EXPECT_DOUBLE_EQ(Parameters::stereo.baseline, 0.12);
  EXPECT_DOUBLE_EQ(Parameters::stereo.fx, 535.4);
  EXPECT_DOUBLE_EQ(Parameters::guidance.nav_constant_far, 4.0);
  EXPECT_DOUBLE_EQ(Parameters::guidance.max_acceleration, 20.0);
  EXPECT_DOUBLE_EQ(Parameters::controller.min_altitude, 2.0);
}

TEST_F(ParametersTest, AppliesParameterOverrides)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {
        {"stereo.baseline", 0.2},
        {"ekf.measurement_noise_pos", 0.05},
        {"guidance.max_acceleration", 15.0},
        {"controller.max_velocity", 10.0},
      });
  auto node = std::make_shared<rclcpp::Node>("parameters_overrides_test", options);
  Parameters::loadFromNode(node.get());

  EXPECT_DOUBLE_EQ(Parameters::stereo.baseline, 0.2);
  EXPECT_DOUBLE_EQ(Parameters::ekf.measurement_noise_pos, 0.05);
  EXPECT_DOUBLE_EQ(Parameters::guidance.max_acceleration, 15.0);
  EXPECT_DOUBLE_EQ(Parameters::controller.max_velocity, 10.0);
  // Untouched values keep their defaults.
  EXPECT_DOUBLE_EQ(Parameters::stereo.fx, 535.4);
}

}  // namespace
}  // namespace interceptor
