/**
 * @file test_follow_scenario.cpp
 * @brief Unit tests for the FOLLOW scenarios: the path of the target, the speed,
 *        the start offset of the seed and the parsing of the scenario names.
 *
 * The scenarios are ROS-free, so everything is driven with a synthetic clock: no
 * simulator, no messages, no timing surprises.
 */

#include <gtest/gtest.h>
#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <string>

#include "follow/follow_scenario.hpp"

namespace interceptor
{
namespace follow
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

/// The loop of the walker scenario: 8 x 12 m with 1.5 m corners.
ScenarioPath walkerPath()
{
  return ScenarioPath::loop(Eigen::Vector2d(8.5, 6.0), 4.0, 6.0, 1.5);
}

}  // namespace

// --- The names ---------------------------------------------------------------

TEST(FollowScenario, TheNamesAreParsed)
{
  EXPECT_EQ(parseFollowScenario("walker"), FollowScenarioType::WALKER);
  EXPECT_EQ(parseFollowScenario("JOGGER"), FollowScenarioType::JOGGER);
  EXPECT_EQ(parseFollowScenario("Occlusion"), FollowScenarioType::OCCLUSION);
  EXPECT_EQ(parseFollowScenario("bystander"), FollowScenarioType::BYSTANDER);
  EXPECT_FALSE(parseFollowScenario("intercept").has_value());
  EXPECT_FALSE(parseFollowScenario("").has_value());
}

TEST(FollowScenario, TheNamesArePrinted)
{
  EXPECT_EQ(toString(FollowScenarioType::WALKER), "walker");
  EXPECT_EQ(toString(FollowScenarioType::JOGGER), "jogger");
  EXPECT_EQ(toString(FollowScenarioType::OCCLUSION), "occlusion");
  EXPECT_EQ(toString(FollowScenarioType::BYSTANDER), "bystander");
}

// --- The path ----------------------------------------------------------------

TEST(FollowScenario, TheLoopIsClosed)
{
  const ScenarioPath path = walkerPath();
  EXPECT_NEAR(path.position(0.0).x(), path.position(path.length()).x(), 1e-9);
  EXPECT_NEAR(path.position(0.0).y(), path.position(path.length()).y(), 1e-9);
  EXPECT_NEAR(
    path.position(-1.0).x(), path.position(path.length() - 1.0).x(), 1e-9);
  EXPECT_NEAR(
    path.position(-1.0).y(), path.position(path.length() - 1.0).y(), 1e-9);
}

TEST(FollowScenario, ThePerimeterOfTheLoop)
{
  // Two legs of 8 - 2 * 1.5 = 5 m in x, two of 12 - 2 * 1.5 = 9 m in y and four
  // quarter circles of 1.5 m.
  const ScenarioPath path = walkerPath();
  EXPECT_NEAR(path.length(), 2.0 * 5.0 + 2.0 * 9.0 + 2.0 * kPi * 1.5, 1e-9);
}

TEST(FollowScenario, ThePathStartsAtTheBottomLegGoingEast)
{
  const ScenarioPath path = walkerPath();
  const Eigen::Vector2d position = path.position(0.0);
  EXPECT_NEAR(position.x(), 8.5 - 4.0 + 1.5, 1e-9);
  EXPECT_NEAR(position.y(), 6.0 - 6.0, 1e-9);
  const Eigen::Vector2d tangent = path.tangent(0.0);
  EXPECT_NEAR(tangent.x(), 1.0, 1e-6);
  EXPECT_NEAR(tangent.y(), 0.0, 1e-6);
}

TEST(FollowScenario, ThePathGoesCounterClockwise)
{
  const ScenarioPath path = walkerPath();
  // A quarter of the perimeter later, the path is on the right leg, going north.
  const double s = 0.25 * path.length();
  const Eigen::Vector2d position = path.position(s);
  EXPECT_GT(position.x(), 8.0);
  const Eigen::Vector2d tangent = path.tangent(s);
  EXPECT_NEAR(tangent.x(), 0.0, 1e-6);
  EXPECT_NEAR(tangent.y(), 1.0, 1e-6);
}

TEST(FollowScenario, TheTangentIsAlwaysAUnitVector)
{
  const ScenarioPath path = walkerPath();
  for (int i = 0; i < 200; ++i) {
    const double s = path.length() * i / 200.0;
    EXPECT_NEAR(path.tangent(s).norm(), 1.0, 1e-9);
  }
}

TEST(FollowScenario, TheCornerRadiusIsClampedToTheLoop)
{
  // A radius larger than half of the smaller side would make the fillets overlap.
  const ScenarioPath path = ScenarioPath::loop(Eigen::Vector2d(0.0, 0.0), 2.0, 2.0, 5.0);
  EXPECT_GT(path.length(), 0.0);
  for (int i = 0; i < 100; ++i) {
    const double s = path.length() * i / 100.0;
    EXPECT_TRUE(path.position(s).allFinite());
  }
}

TEST(FollowScenario, ThePathStaysInsideTheLoop)
{
  const ScenarioPath path = walkerPath();
  for (int i = 0; i < 200; ++i) {
    const Eigen::Vector2d position = path.position(path.length() * i / 200.0);
    EXPECT_GE(position.x(), 8.5 - 4.0 - 1e-9);
    EXPECT_LE(position.x(), 8.5 + 4.0 + 1e-9);
    EXPECT_GE(position.y(), 6.0 - 6.0 - 1e-9);
    EXPECT_LE(position.y(), 6.0 + 6.0 + 1e-9);
  }
}

// --- The scenario ------------------------------------------------------------

TEST(FollowScenario, TheTargetWalksAtTheConfiguredSpeed)
{
  FollowScenarioParams params;
  params.path = walkerPath();
  params.speed = 1.4;
  params.start_phase = 0.0;
  const FollowScenario scenario(params);

  const FollowScenarioState first = scenario.state(0.0);
  const FollowScenarioState later = scenario.state(1.0);
  const double walked = (later.target_position - first.target_position).norm();
  // The path is straight at the start, so the distance walked is the speed.
  EXPECT_NEAR(walked, 1.4, 1e-6);
  EXPECT_NEAR(later.target_velocity.norm(), 1.4, 1e-9);
}

TEST(FollowScenario, TheYawFollowsTheDirectionOfTravel)
{
  FollowScenarioParams params;
  params.path = walkerPath();
  params.speed = 1.4;
  const FollowScenario scenario(params);
  const FollowScenarioState state = scenario.state(0.0);
  EXPECT_NEAR(state.target_yaw, 0.0, 1e-6);
  EXPECT_NEAR(state.target_velocity.y(), 0.0, 1e-9);
  EXPECT_GT(state.target_velocity.x(), 0.0);
}

TEST(FollowScenario, TheSeedShiftsTheStart)
{
  FollowScenarioParams params;
  params.path = walkerPath();
  params.speed = 1.4;
  params.seed = 7;
  const FollowScenario scenario(params);
  const FollowScenarioState state = scenario.state(0.0);
  // The seed moves the target along its path, so it does not start at the same
  // place as the run without a seed.
  EXPECT_GT(state.target_position.x(), 6.0);
}

TEST(FollowScenario, TheRunIsReproducible)
{
  FollowScenarioParams params;
  params.path = walkerPath();
  params.speed = 1.4;
  params.seed = 3;
  const FollowScenario first(params);
  const FollowScenario second(params);
  for (const double t : {0.0, 1.5, 12.25, 60.0}) {
    const FollowScenarioState a = first.state(t);
    const FollowScenarioState b = second.state(t);
    EXPECT_NEAR(a.target_position.x(), b.target_position.x(), 1e-12);
    EXPECT_NEAR(a.target_position.y(), b.target_position.y(), 1e-12);
    EXPECT_NEAR(a.target_yaw, b.target_yaw, 1e-12);
  }
}

TEST(FollowScenario, TheBystanderStandsStill)
{
  FollowScenarioParams params;
  params.path = walkerPath();
  params.bystander_enabled = true;
  params.bystander_position = Eigen::Vector2d(16.5, 2.0);
  const FollowScenario scenario(params);
  const FollowScenarioState a = scenario.state(0.0);
  const FollowScenarioState b = scenario.state(30.0);
  EXPECT_NEAR(a.bystander_position.x(), 16.5, 1e-9);
  EXPECT_NEAR(a.bystander_position.y(), 2.0, 1e-9);
  EXPECT_NEAR(a.bystander_position.x(), b.bystander_position.x(), 1e-12);
}

// --- The geometry of the scenarios -------------------------------------------

// The scenarios of the plan. The drone flies 8 m behind the target and 5 m above
// it, and the safety filter keeps it 7.3 m away from the centre of an oak (the
// canopy's radius, the 3 m standoff and the drone's own radius) and 4.9 m from the
// centre of a pine. The paths have to stay outside those radii.
TEST(FollowScenario, ThePathsKeepOutOfTheTrees)
{
  const std::vector<Eigen::Vector2d> oaks = {
    {20.0, 7.0}, {7.0, 22.0}, {-4.0, 10.0}, {18.0, 23.0}, {24.0, 14.0}};
  const std::vector<Eigen::Vector2d> pines = {
    {4.0, -5.0}, {27.0, 6.0}, {-3.0, 22.0}, {26.0, 26.0}};

  // The loops of the four scenarios.
  const std::vector<ScenarioPath> paths = {
    ScenarioPath::loop(Eigen::Vector2d(8.5, 6.0), 4.0, 6.0, 1.5),    // walker
    ScenarioPath::loop(Eigen::Vector2d(8.5, 6.0), 4.0, 6.0, 2.5),    // jogger
    ScenarioPath::loop(Eigen::Vector2d(11.0, 6.5), 6.5, 5.5, 2.0),   // occlusion
    ScenarioPath::loop(Eigen::Vector2d(8.5, 6.0), 4.0, 6.0, 1.5),    // bystander
  };

  for (size_t p = 0; p < paths.size(); ++p) {
    const ScenarioPath & path = paths[p];
    for (int i = 0; i < 4000; ++i) {
      const Eigen::Vector2d position = path.position(path.length() * i / 4000.0);
      for (const Eigen::Vector2d & oak : oaks) {
        // The occlusion scenario walks the target through the canopy on purpose:
        // the tree it walks through is the one at (20, 7), and the whole crossing
        // is exempt from the keep-out of the trees.
        const bool intentional = p == 2 && (oak - Eigen::Vector2d(20.0, 7.0)).norm() < 1e-6;
        if (!intentional) {
          EXPECT_GE((position - oak).norm(), 7.3 - 1e-6);
        }
      }
      for (const Eigen::Vector2d & pine : pines) {
        EXPECT_GE((position - pine).norm(), 4.9 - 1e-6);
      }
    }
  }
}

TEST(FollowScenario, TheOcclusionScenarioWalksThroughTheCanopy)
{
  const ScenarioPath path = ScenarioPath::loop(Eigen::Vector2d(11.0, 6.5), 6.5, 5.5, 2.0);
  double closest = 1e9;
  for (int i = 0; i < 4000; ++i) {
    const Eigen::Vector2d position = path.position(path.length() * i / 4000.0);
    closest = std::min(closest, (position - Eigen::Vector2d(20.0, 7.0)).norm());
  }
  // The path goes through the canopy, so the line of sight to the target is
  // blocked: the target is inside the 4 m canopy of the tree.
  EXPECT_LT(closest, 3.0);
}

TEST(FollowScenario, TheBystanderStandsNextToThePath)
{
  const ScenarioPath path = ScenarioPath::loop(Eigen::Vector2d(8.5, 6.0), 4.0, 6.0, 1.5);
  const Eigen::Vector2d bystander(16.5, 2.0);
  double closest = 1e9;
  for (int i = 0; i < 4000; ++i) {
    const Eigen::Vector2d position = path.position(path.length() * i / 4000.0);
    closest = std::min(closest, (position - bystander).norm());
  }
  // The second person is 4 m from the path: close enough for the keep-out to
  // matter, far enough for the tracker not to confuse the two.
  EXPECT_NEAR(closest, 4.0, 0.05);
  EXPECT_GT(closest, 3.0);
}

}  // namespace follow
}  // namespace interceptor
