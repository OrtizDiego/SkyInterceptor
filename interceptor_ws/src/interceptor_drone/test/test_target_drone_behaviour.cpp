/**
 * @file test_target_drone_behaviour.cpp
 * @brief The gtests of the target drone behaviours (plan 3.1).
 *
 * The two properties the behaviours have to keep are checked for every one of
 * them: the published position is the exact integral of the published velocity,
 * and the published velocity is continuous.
 */

#include <gtest/gtest.h>
#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <vector>

#include "guidance/target_drone_behaviour.hpp"

namespace interceptor
{
namespace guidance
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

/// The parameters of a behaviour, with the start pose at a known place.
TargetBehaviourParams defaultParams(TargetBehaviour behaviour)
{
  TargetBehaviourParams params;
  params.behaviour = behaviour;
  params.start_position = Eigen::Vector3d(10.0, 5.0, 12.0);
  params.start_yaw = 0.0;
  return params;
}

/// A square loop of 20 m a side, at a constant altitude.
TargetBehaviourParams squareLoopParams(double speed = 6.0, double turn_radius = 3.0)
{
  TargetBehaviourParams params = defaultParams(TargetBehaviour::WAYPOINTS);
  params.waypoints = {
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(20.0, 0.0, 0.0),
    Eigen::Vector3d(20.0, 20.0, 0.0),
    Eigen::Vector3d(0.0, 20.0, 0.0),
  };
  params.waypoint_speed = speed;
  params.waypoint_turn_radius = turn_radius;
  return params;
}

// ---------------------------------------------------------------------------
// The names
// ---------------------------------------------------------------------------

TEST(TargetDroneBehaviour, ParsesTheBehaviourNames)
{
  EXPECT_EQ(parseTargetBehaviour("hover"), TargetBehaviour::HOVER);
  EXPECT_EQ(parseTargetBehaviour("waypoints"), TargetBehaviour::WAYPOINTS);
  EXPECT_EQ(parseTargetBehaviour("straight"), TargetBehaviour::STRAIGHT);
  EXPECT_EQ(parseTargetBehaviour("weave"), TargetBehaviour::WEAVE);
  EXPECT_EQ(parseTargetBehaviour("curve"), TargetBehaviour::CURVE);
  EXPECT_FALSE(parseTargetBehaviour("spiral").has_value());
  EXPECT_FALSE(parseTargetBehaviour("").has_value());

  // The name of a behaviour is the name it is written with in the config file.
  for (const TargetBehaviour behaviour :
    {TargetBehaviour::HOVER, TargetBehaviour::WAYPOINTS, TargetBehaviour::STRAIGHT,
      TargetBehaviour::WEAVE, TargetBehaviour::CURVE})
  {
    const auto parsed = parseTargetBehaviour(toString(behaviour));
    ASSERT_TRUE(parsed.has_value());
    EXPECT_EQ(*parsed, behaviour);
  }
}

// ---------------------------------------------------------------------------
// The properties every behaviour keeps
// ---------------------------------------------------------------------------

TEST(TargetDroneBehaviour, ThePositionIsTheIntegralOfTheVelocity)
{
  const std::vector<TargetBehaviour> behaviours = {
    TargetBehaviour::HOVER, TargetBehaviour::WAYPOINTS, TargetBehaviour::STRAIGHT,
    TargetBehaviour::WEAVE, TargetBehaviour::CURVE};

  for (const TargetBehaviour behaviour : behaviours) {
    TargetDroneBehaviour drone(defaultParams(behaviour));
    const double dt = 0.02;
    Eigen::Vector3d previous = drone.state().position;
    for (int i = 0; i < 500; ++i) {
      const TargetBehaviourState & state = drone.step(dt);
      // The finite difference of the published positions is exactly the published
      // velocity: the tracker's estimate of the velocity matches what the drone
      // really does.
      const Eigen::Vector3d difference = state.position - previous - state.velocity * dt;
      EXPECT_LT(difference.norm(), 1e-12) << "behaviour " << toString(behaviour);
      previous = state.position;
    }
  }
}

TEST(TargetDroneBehaviour, TheVelocityOfEveryBehaviourIsContinuous)
{
  const std::vector<TargetBehaviour> behaviours = {
    TargetBehaviour::HOVER, TargetBehaviour::WAYPOINTS, TargetBehaviour::STRAIGHT,
    TargetBehaviour::WEAVE, TargetBehaviour::CURVE};

  for (const TargetBehaviour behaviour : behaviours) {
    TargetDroneBehaviour drone(defaultParams(behaviour));
    const double dt = 0.02;
    Eigen::Vector3d previous = drone.state().velocity;
    for (int i = 0; i < 500; ++i) {
      const TargetBehaviourState & state = drone.step(dt);
      // No behaviour steps its velocity: the biggest jump is the one of the
      // curvature of the fillets of the waypoints, 6 m/s over a 3 m radius.
      EXPECT_LT((state.velocity - previous).norm(), 0.5)
        << "behaviour " << toString(behaviour) << " at step " << i;
      previous = state.velocity;
    }
  }
}

TEST(TargetDroneBehaviour, TheBehaviourStartsAtItsStartPose)
{
  TargetBehaviourParams params = defaultParams(TargetBehaviour::CURVE);
  params.start_position = Eigen::Vector3d(3.0, -4.0, 12.0);
  TargetDroneBehaviour drone(params);

  EXPECT_NEAR(drone.state().position.x(), 3.0, 1e-12);
  EXPECT_NEAR(drone.state().position.y(), -4.0, 1e-12);
  EXPECT_NEAR(drone.state().position.z(), 12.0, 1e-12);

  // A reset puts the intruder back at the start pose and the clock back at zero.
  drone.step(1.0);
  drone.reset();
  EXPECT_NEAR(drone.state().position.x(), 3.0, 1e-12);
  EXPECT_NEAR(drone.state().time, 0.0, 1e-12);
}

// ---------------------------------------------------------------------------
// HOVER
// ---------------------------------------------------------------------------

TEST(TargetDroneBehaviour, HoverHoldsThePosition)
{
  TargetBehaviourParams params = defaultParams(TargetBehaviour::HOVER);
  params.hover_drift = 0.0;
  TargetDroneBehaviour drone(params);
  const Eigen::Vector3d start = drone.state().position;

  for (int i = 0; i < 1500; ++i) {
    const TargetBehaviourState & state = drone.step(0.02);
    EXPECT_LT((state.position - start).norm(), 1e-12);
    EXPECT_LT(state.velocity.norm(), 1e-12);
    EXPECT_LT(state.acceleration.norm(), 1e-12);
  }
}

TEST(TargetDroneBehaviour, TheHoverDriftStaysInsideItsRadius)
{
  TargetBehaviourParams params = defaultParams(TargetBehaviour::HOVER);
  params.hover_drift = 2.0;
  params.hover_drift_period = 20.0;
  TargetDroneBehaviour drone(params);
  const Eigen::Vector3d start = drone.state().position;

  double furthest = 0.0;
  for (int i = 0; i < 3000; ++i) {
    const TargetBehaviourState & state = drone.step(0.02);
    furthest = std::max(furthest, (state.position - start).norm());
  }
  // The drone wanders at most as far as the radius of the drift.
  EXPECT_LT(furthest, 2.0 + 1e-3);
  EXPECT_GT(furthest, 1.0);
  // The drift starts and ends at rest, so the velocity is continuous.
  EXPECT_LT(drone.state().velocity.norm(), 1e-9);
}

// ---------------------------------------------------------------------------
// STRAIGHT
// ---------------------------------------------------------------------------

TEST(TargetDroneBehaviour, TheStraightRunKeepsItsSpeed)
{
  TargetBehaviourParams params = defaultParams(TargetBehaviour::STRAIGHT);
  params.start_position = Eigen::Vector3d::Zero();
  params.straight_speed = 10.0;
  params.straight_direction_yaw = 0.0;
  params.straight_duration = 20.0;
  params.straight_ramp_time = 1.0;

  // The ramp up is a raised cosine: half the speed at half the ramp, and the
  // acceleration is at its largest there.
  TargetDroneBehaviour drone(params);
  TargetBehaviourState state = drone.step(0.5);
  EXPECT_NEAR(state.velocity.x(), 5.0, 1e-9);
  EXPECT_NEAR(state.velocity.y(), 0.0, 1e-9);
  EXPECT_NEAR(state.acceleration.x(), 0.5 * 10.0 * kPi * std::sin(0.5 * kPi), 1e-9);

  // The cruise: the full speed, in the configured direction.
  state = drone.step(4.5);
  EXPECT_NEAR(state.velocity.norm(), 10.0, 1e-9);
  EXPECT_NEAR(state.velocity.x(), 10.0, 1e-9);

  // The distance of the run: the cruise plus the two half ramps. The steps are
  // small here, so that the position is the integral of the velocity.
  TargetDroneBehaviour fine(params);
  for (int i = 0; i < 1000; ++i) {
    state = fine.step(0.02);
  }
  EXPECT_NEAR(state.time, 20.0, 1e-9);
  // The run ends with a ramp down, so the drone has stopped.
  EXPECT_NEAR(state.velocity.norm(), 0.0, 1e-9);
  EXPECT_NEAR(state.position.x(), 10.0 * (20.0 - 1.0), 0.05);
  EXPECT_NEAR(state.position.y(), 0.0, 1e-9);
}

TEST(TargetDroneBehaviour, TheStraightRunStopsAtTheConfiguredDistance)
{
  TargetBehaviourParams params = defaultParams(TargetBehaviour::STRAIGHT);
  params.start_position = Eigen::Vector3d::Zero();
  params.straight_speed = 10.0;
  params.straight_distance = 100.0;
  params.straight_duration = 0.0;
  params.straight_ramp_time = 1.0;
  TargetDroneBehaviour drone(params);

  TargetBehaviourState state;
  for (int i = 0; i < 2000; ++i) {
    state = drone.step(0.02);
  }
  // The run covered the 100 m of the configuration and stopped.
  EXPECT_NEAR(state.position.x(), 100.0, 0.05);
  EXPECT_NEAR(state.velocity.norm(), 0.0, 1e-9);
}

TEST(TargetDroneBehaviour, TheStraightRunDoesNotStepItsVelocity)
{
  TargetBehaviourParams params = defaultParams(TargetBehaviour::STRAIGHT);
  params.start_position = Eigen::Vector3d::Zero();
  params.straight_speed = 10.0;
  params.straight_duration = 10.0;
  params.straight_ramp_time = 1.0;
  TargetDroneBehaviour drone(params);

  Eigen::Vector3d previous = drone.state().velocity;
  for (int i = 0; i < 1000; ++i) {
    const TargetBehaviourState & state = drone.step(0.01);
    // The biggest acceleration of the raised cosine is v_max * pi / (2 * ramp).
    EXPECT_LT((state.velocity - previous).norm(), 10.0 * kPi / 2.0 * 0.01 + 1e-9);
    previous = state.velocity;
  }
}

// ---------------------------------------------------------------------------
// WEAVE
// ---------------------------------------------------------------------------

TEST(TargetDroneBehaviour, TheWeaveHasTheConfiguredAmplitudeAndFrequency)
{
  TargetBehaviourParams params = defaultParams(TargetBehaviour::WEAVE);
  params.weave_speed = 6.0;
  params.weave_amplitude = 2.0;
  params.weave_frequency = 0.25;
  params.weave_direction_yaw = 0.0;
  TargetDroneBehaviour drone(params);

  const double dt = 0.005;
  const double period = 1.0 / params.weave_frequency;
  const int steps = static_cast<int>(4.0 * period / dt);

  // The lateral offset of the weave, the drone flies along +x.
  std::vector<double> lateral;
  std::vector<double> times;
  for (int i = 0; i < steps; ++i) {
    const TargetBehaviourState & state = drone.step(dt);
    lateral.push_back(state.position.y() - params.start_position.y());
    times.push_back(state.time);
  }

  // The amplitude of the lateral offset is the configured one.
  const double lowest = *std::min_element(lateral.begin(), lateral.end());
  const double highest = *std::max_element(lateral.begin(), lateral.end());
  EXPECT_NEAR(0.5 * (highest - lowest), params.weave_amplitude, 0.02 * params.weave_amplitude);
  // The weave is centred on the start position.
  EXPECT_NEAR(0.5 * (highest + lowest), 0.0, 0.02 * params.weave_amplitude);

  // The period of the weave is the configured one: the offset has its maximum
  // once a period.
  std::vector<double> peaks;
  for (size_t i = 1; i + 1 < lateral.size(); ++i) {
    if (lateral[i] > lateral[i - 1] && lateral[i] >= lateral[i + 1]) {
      peaks.push_back(times[i]);
    }
  }
  ASSERT_GE(peaks.size(), 3u);
  double measured = 0.0;
  for (size_t i = 1; i < peaks.size(); ++i) {
    measured += peaks[i] - peaks[i - 1];
  }
  measured /= static_cast<double>(peaks.size() - 1);
  EXPECT_NEAR(measured, period, 0.02 * period);
}

TEST(TargetDroneBehaviour, TheWeaveKeepsItsForwardSpeed)
{
  TargetBehaviourParams params = defaultParams(TargetBehaviour::WEAVE);
  params.weave_speed = 6.0;
  params.weave_amplitude = 2.0;
  params.weave_frequency = 0.25;
  params.weave_direction_yaw = 0.0;
  TargetDroneBehaviour drone(params);

  const double dt = 0.01;
  for (int i = 0; i < 100; ++i) {
    const TargetBehaviourState & state = drone.step(dt);
    // The forward speed is the configured one, whatever the weave does.
    EXPECT_NEAR(state.velocity.x(), params.weave_speed, 1e-9);
  }
  // And the drone really moves forward at that speed.
  const double forward = drone.state().position.x() - params.start_position.x();
  EXPECT_NEAR(forward, params.weave_speed * drone.state().time, 1e-6);
}

// ---------------------------------------------------------------------------
// WAYPOINTS
// ---------------------------------------------------------------------------

TEST(TargetDroneBehaviour, TheWaypointsAreFlownAtTheConfiguredSpeed)
{
  TargetDroneBehaviour drone(squareLoopParams(6.0));
  const double dt = 0.02;
  for (int i = 0; i < 1000; ++i) {
    const TargetBehaviourState & state = drone.step(dt);
    // The speed of the loop is constant: the vertical speed is zero, because the
    // waypoints are all at the same altitude.
    EXPECT_NEAR(state.velocity.norm(), 6.0, 1e-9);
  }
}

TEST(TargetDroneBehaviour, TheWaypointsAreVisitedInOrder)
{
  TargetDroneBehaviour drone(squareLoopParams(6.0, 3.0));
  const double dt = 0.01;

  // The closest approach of the path to a corner: the fillet cuts the corner, so
  // the drone passes R * (1 / cos(turn / 2) - 1) away from the waypoint.
  const double turn = kPi / 2.0;
  const double cut = 3.0 * (1.0 / std::cos(turn / 2.0) - 1.0);

  std::vector<double> closest(4, 1e9);
  std::vector<int> order;
  for (int i = 0; i < 4000; ++i) {
    const TargetBehaviourState & state = drone.step(dt);
    for (size_t k = 0; k < 4; ++k) {
      const double distance = (state.position - drone.params().waypoints[k]).norm();
      if (distance < closest[k]) {
        closest[k] = distance;
      }
    }
    // The waypoint the drone is closest to is the next one of the list.
    size_t nearest = 0;
    for (size_t k = 1; k < 4; ++k) {
      if ((state.position - drone.params().waypoints[k]).norm() <
        (state.position - drone.params().waypoints[nearest]).norm())
      {
        nearest = k;
      }
    }
    if (order.empty() || order.back() != static_cast<int>(nearest)) {
      order.push_back(static_cast<int>(nearest));
    }
  }

  // Every waypoint is visited. The drone starts on the first one, and it passes
  // the others at the distance the fillet cuts the corner.
  EXPECT_LT(closest[0], 0.1);
  for (size_t k = 1; k < 4; ++k) {
    EXPECT_NEAR(closest[k], cut, 0.2) << "waypoint " << k;
  }
  // The waypoints are visited in the order of the list, and the loop starts again.
  ASSERT_GE(order.size(), 5u);
  EXPECT_EQ(order[0], 0);
  for (size_t i = 1; i < order.size(); ++i) {
    EXPECT_EQ(order[i], (order[i - 1] + 1) % 4);
  }
}

TEST(TargetDroneBehaviour, TheVelocityStaysContinuousAcrossAWaypointTurn)
{
  TargetDroneBehaviour drone(squareLoopParams(6.0, 3.0));
  const double dt = 0.02;

  double previous_heading = 0.0;
  bool first = true;
  double biggest_jump = 0.0;
  for (int i = 0; i < 2000; ++i) {
    const TargetBehaviourState & state = drone.step(dt);
    if (!first) {
      double jump = std::atan2(
        std::sin(state.yaw - previous_heading), std::cos(state.yaw - previous_heading));
      biggest_jump = std::max(biggest_jump, std::abs(jump));
    }
    previous_heading = state.yaw;
    first = false;
  }
  // The drone turns at the rate of the fillet: v / R, and the heading never
  // jumps.
  const double max_rate = 6.0 / 3.0;
  EXPECT_LT(biggest_jump, max_rate * dt * 1.001);
}

TEST(TargetDroneBehaviour, TheWaypointLoopRepeats)
{
  TargetDroneBehaviour drone(squareLoopParams(6.0, 3.0));
  const double dt = 0.005;

  // The length of the loop: the straights of the rounded square and its fillets.
  const double perimeter = 4.0 * (20.0 - 2.0 * 3.0) + 2.0 * kPi * 3.0;
  const double loop_time = perimeter / 6.0;
  // The drone starts on the first waypoint and flies a short lead-in to the loop.
  const double lead_in = 3.0;

  // The position at the start of the loop.
  for (int i = 0; i < static_cast<int>(lead_in / 6.0 / dt); ++i) {
    drone.step(dt);
  }
  const Eigen::Vector3d start = drone.state().position;

  for (int i = 0; i < static_cast<int>(loop_time / dt); ++i) {
    drone.step(dt);
  }
  // After one loop the drone is back where the loop started.
  EXPECT_LT((drone.state().position - start).norm(), 0.2);
  // And the loop does not drift: the drone is on the same point of the path, in
  // the same direction.
  const double heading = drone.state().yaw;
  EXPECT_LT(std::abs(std::atan2(std::sin(heading), std::cos(heading))), 0.05);
}

// ---------------------------------------------------------------------------
// CURVE
// ---------------------------------------------------------------------------

TEST(TargetDroneBehaviour, TheCurveProfileAcceleratesAndTurns)
{
  TargetBehaviourParams params = defaultParams(TargetBehaviour::CURVE);
  params.final_speed = 6.0;
  params.accel_time = 4.0;
  params.turn_rate = 1.2;
  params.start_yaw = 0.0;
  TargetDroneBehaviour drone(params);

  // The speed ramps up to the cruise speed.
  TargetBehaviourState state = drone.step(2.0);
  EXPECT_NEAR(state.velocity.norm(), 3.0, 1e-9);
  // The heading turns at the configured rate.
  EXPECT_NEAR(state.yaw, 2.4, 1e-9);
  // The acceleration has the tangential and the lateral part.
  EXPECT_NEAR(state.acceleration.x(), 1.5 * std::cos(2.4) - 3.0 * 1.2 * std::sin(2.4), 1e-9);
  EXPECT_NEAR(state.acceleration.y(), 1.5 * std::sin(2.4) + 3.0 * 1.2 * std::cos(2.4), 1e-9);

  // After the ramp the speed is the cruise speed and the turn continues.
  state = drone.step(3.0);
  EXPECT_NEAR(state.velocity.norm(), 6.0, 1e-9);
  EXPECT_NEAR(state.yaw, 6.0, 1e-9);
}

TEST(TargetDroneBehaviour, TheCurveProfileFollowsItsArc)
{
  TargetBehaviourParams params = defaultParams(TargetBehaviour::CURVE);
  params.final_speed = 6.0;
  params.accel_time = 4.0;
  params.turn_rate = 1.2;
  params.start_position = Eigen::Vector3d::Zero();
  TargetDroneBehaviour drone(params);

  // After the ramp the drone flies a circle of the radius v / turn_rate, so it
  // stays at the same distance from the centre of that circle.
  const double radius = 6.0 / 1.2;
  drone.step(4.0);
  const TargetBehaviourState & at_ramp = drone.state();
  const Eigen::Vector3d centre = at_ramp.position +
    radius * Eigen::Vector3d(-std::sin(at_ramp.yaw), std::cos(at_ramp.yaw), 0.0);

  double distance = 0.0;
  for (int i = 0; i < 500; ++i) {
    const TargetBehaviourState & state = drone.step(0.02);
    distance = (state.position - centre).norm();
  }
  EXPECT_NEAR(distance, radius, 0.05);
}

}  // namespace
}  // namespace guidance
}  // namespace interceptor
