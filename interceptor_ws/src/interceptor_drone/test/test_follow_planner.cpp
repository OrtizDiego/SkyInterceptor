/**
 * @file test_follow_planner.cpp
 * @brief Unit tests for the follow planner: the framing presets, the velocity caps,
 *        the heading filter, the lost-track machine, the operator track override and
 *        the mode gate.
 *
 * The planner is ROS-free, so everything is driven with synthetic poses on a
 * synthetic clock: no simulator, no messages, no timing surprises.
 */

#include <gtest/gtest.h>
#include <Eigen/Dense>

#include <cmath>
#include <string>

#include "follow/follow_planner.hpp"

namespace interceptor
{
namespace follow
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kDt = 0.02;  // s, the 50 Hz timer of the node

/// The planner with the plan's defaults, except the offsets, which the tests spell
/// out so the expected geometry is obvious.
FollowPlannerParams testParams()
{
  FollowPlannerParams params;
  params.offset_behind = 8.0;
  params.offset_side = 6.0;
  params.offset_height = 5.0;
  params.kp_pos = 0.8;
  return params;
}

/// A confirmed, valid track.
FollowTarget targetAt(
  int id, const Eigen::Vector3d & position, const Eigen::Vector3d & velocity, double heading)
{
  FollowTarget target;
  target.track_id = id;
  target.position = position;
  target.velocity = velocity;
  target.heading = heading;
  target.confirmed = true;
  target.is_valid = true;
  return target;
}

FollowState droneAt(double x, double y, double z = 0.0)
{
  FollowState state;
  state.position = Eigen::Vector3d(x, y, z);
  return state;
}

/// A planner that is already in the FOLLOW mode, with the target delivered.
FollowPlanner plannerInFollow(
  const FollowPlannerParams & params, const FollowTarget & target, double time_s = 0.0)
{
  FollowPlanner planner(params);
  planner.setMode(FollowMode::FOLLOW, time_s);
  planner.onTarget(target, time_s);
  return planner;
}

// ---------------------------------------------------------------------------
// The presets
// ---------------------------------------------------------------------------

TEST(FramingPreset, ParsesTheNames)
{
  EXPECT_EQ(parseFramingPreset("BEHIND"), FramingPreset::BEHIND);
  EXPECT_EQ(parseFramingPreset("side"), FramingPreset::SIDE);
  EXPECT_EQ(parseFramingPreset("Lead"), FramingPreset::LEAD);
  EXPECT_EQ(parseFramingPreset("orbit"), FramingPreset::ORBIT);
  EXPECT_FALSE(parseFramingPreset("nonsense").has_value());
  EXPECT_EQ(toString(FramingPreset::ORBIT), "ORBIT");
}

TEST(FramingPreset, BehindSitsAlongTheHeading)
{
  FollowPlannerParams params = testParams();
  params.preset = FramingPreset::BEHIND;

  // The target is at (10, 0, 0) and drives along +x, so "behind" is -x.
  const FollowTarget target = targetAt(1, Eigen::Vector3d(10, 0, 0), Eigen::Vector3d(2, 0, 0), 0.0);
  FollowPlanner planner = plannerInFollow(params, target);

  const FollowCommand command = planner.update(droneAt(0, 0), 0.0);

  EXPECT_EQ(command.phase, FollowPhase::TRACKING);
  EXPECT_TRUE(command.active);
  // p_des = p_t + R_z(0) * [-8, 0, 5]
  EXPECT_NEAR(command.desired_position.x(), 2.0, 1e-9);
  EXPECT_NEAR(command.desired_position.y(), 0.0, 1e-9);
  EXPECT_NEAR(command.desired_position.z(), 5.0, 1e-9);
  // v_des = v_t + kp * (p_des - p) = (2, 0, 0) + 0.8 * (2, 0, 5)
  EXPECT_NEAR(command.velocity.x(), 3.6, 1e-9);
  EXPECT_NEAR(command.velocity.y(), 0.0, 1e-9);
  EXPECT_NEAR(command.velocity.z(), 4.0, 1e-9);
  // The camera looks at the target.
  EXPECT_NEAR(command.yaw, 0.0, 1e-9);
  EXPECT_EQ(command.source, kSourceFollow);
}

TEST(FramingPreset, SideIsLateral)
{
  FollowPlannerParams params = testParams();
  params.preset = FramingPreset::SIDE;

  const FollowTarget target = targetAt(1, Eigen::Vector3d(10, 0, 0), Eigen::Vector3d(2, 0, 0), 0.0);
  FollowPlanner planner = plannerInFollow(params, target);

  const FollowCommand command = planner.update(droneAt(0, 0), 0.0);

  // p_des = p_t + R_z(0) * [0, 6, 5]
  EXPECT_NEAR(command.desired_position.x(), 10.0, 1e-9);
  EXPECT_NEAR(command.desired_position.y(), 6.0, 1e-9);
  EXPECT_NEAR(command.desired_position.z(), 5.0, 1e-9);
  EXPECT_NEAR(command.velocity.x(), 10.0, 1e-9);
  EXPECT_NEAR(command.velocity.y(), 4.8, 1e-9);
  EXPECT_NEAR(command.velocity.z(), 4.0, 1e-9);
}

TEST(FramingPreset, LeadIsInFront)
{
  FollowPlannerParams params = testParams();
  params.preset = FramingPreset::LEAD;

  const FollowTarget target = targetAt(1, Eigen::Vector3d(10, 0, 0), Eigen::Vector3d(2, 0, 0), 0.0);
  FollowPlanner planner = plannerInFollow(params, target);

  const FollowCommand command = planner.update(droneAt(0, 0), 0.0);

  // p_des = p_t + R_z(0) * [8, 0, 5]
  EXPECT_NEAR(command.desired_position.x(), 18.0, 1e-9);
  EXPECT_NEAR(command.desired_position.y(), 0.0, 1e-9);
  EXPECT_NEAR(command.desired_position.z(), 5.0, 1e-9);
}

TEST(FramingPreset, TheFrameFollowsTheTargetHeading)
{
  FollowPlannerParams params = testParams();
  params.preset = FramingPreset::BEHIND;

  // The target drives along +y, so "behind" is -y.
  const FollowTarget target =
    targetAt(1, Eigen::Vector3d(10, 0, 0), Eigen::Vector3d(0, 2, 0), kPi / 2);
  FollowPlanner planner = plannerInFollow(params, target);

  const FollowCommand command = planner.update(droneAt(0, 0), 0.0);

  // R_z(pi/2) * [-8, 0, 5] = [0, -8, 5]
  EXPECT_NEAR(command.desired_position.x(), 10.0, 1e-9);
  EXPECT_NEAR(command.desired_position.y(), -8.0, 1e-9);
  EXPECT_NEAR(command.desired_position.z(), 5.0, 1e-9);
}

TEST(FramingPreset, OrbitStartsBehindAndWalksAround)
{
  FollowPlannerParams params = testParams();
  params.preset = FramingPreset::ORBIT;
  params.orbit_rate = 0.5;  // rad/s

  const FollowTarget target = targetAt(1, Eigen::Vector3d(10, 0, 0), Eigen::Vector3d(2, 0, 0), 0.0);
  FollowPlanner planner = plannerInFollow(params, target);

  // The first frame of the orbit is the BEHIND position.
  const FollowCommand first = planner.update(droneAt(0, 0), 0.0);
  EXPECT_NEAR(first.desired_position.x(), 2.0, 1e-9);
  EXPECT_NEAR(first.desired_position.y(), 0.0, 1e-9);

  // One second later the orbit phase is 0.5 rad: the framing point has walked around
  // the target. The phase is advanced after the framing point is computed, so the
  // command of this cycle used the phase of the previous one.
  double t = 0.0;
  FollowCommand command = first;
  double phase_of_command = 0.0;
  for (int i = 0; i < 50; ++i) {
    t += kDt;
    planner.onTarget(target, t);
    phase_of_command = planner.orbitPhase();
    command = planner.update(droneAt(0, 0), t);
  }
  const double angle = phase_of_command + kPi;  // psi + phase + pi
  EXPECT_NEAR(command.desired_position.x(), 10.0 + 8.0 * std::cos(angle), 1e-6);
  EXPECT_NEAR(command.desired_position.y(), 8.0 * std::sin(angle), 1e-6);
  EXPECT_NEAR(planner.orbitPhase(), 0.5, 1e-6);
}

// ---------------------------------------------------------------------------
// The velocity caps
// ---------------------------------------------------------------------------

TEST(FollowVelocity, IsCappedAtFifteenMetresPerSecond)
{
  FollowPlannerParams params = testParams();

  const FollowTarget target = targetAt(1, Eigen::Vector3d(100, 0, 0), Eigen::Vector3d::Zero(), 0.0);
  FollowPlanner planner = plannerInFollow(params, target);

  const FollowCommand command = planner.update(droneAt(0, 0), 0.0);

  EXPECT_NEAR(command.velocity.norm(), 15.0, 1e-9);
  // The direction is kept, only the magnitude is capped.
  EXPECT_GT(command.velocity.x(), 0.0);
}

TEST(FollowVelocity, TheVerticalComponentIsCappedOnItsOwn)
{
  FollowPlannerParams params = testParams();
  params.max_vertical_speed = 2.0;

  // The target is far above, so the position term asks for a fast climb.
  const FollowTarget target = targetAt(1, Eigen::Vector3d(0, 0, 100), Eigen::Vector3d::Zero(), 0.0);
  FollowPlanner planner = plannerInFollow(params, target);

  const FollowCommand command = planner.update(droneAt(0, 0), 0.0);

  EXPECT_NEAR(command.velocity.z(), 2.0, 1e-9);
}

TEST(FollowVelocity, TheTargetVelocityIsAFeedForward)
{
  FollowPlannerParams params = testParams();

  // The target drives along +x at 3 m/s and the drone is exactly on the framing
  // point, so the whole command is the feed-forward.
  const FollowTarget target = targetAt(1, Eigen::Vector3d(8, 0, 0), Eigen::Vector3d(3, 0, 0), 0.0);
  FollowPlanner planner = plannerInFollow(params, target);

  const FollowCommand command = planner.update(droneAt(0, 0, 5), 0.0);

  EXPECT_NEAR(command.velocity.x(), 3.0, 1e-9);
  EXPECT_NEAR(command.velocity.y(), 0.0, 1e-9);
  EXPECT_NEAR(command.velocity.z(), 0.0, 1e-9);
}

// ---------------------------------------------------------------------------
// The heading filter
// ---------------------------------------------------------------------------

TEST(HeadingFilter, AStepInTheTargetHeadingDoesNotStepTheCommand)
{
  FollowPlannerParams params = testParams();
  params.heading_tau = 0.8;

  // The target stands still on the spot and only its heading changes.
  const FollowTarget target = targetAt(1, Eigen::Vector3d(10, 0, 0), Eigen::Vector3d(2, 0, 0), 0.0);
  FollowPlanner planner = plannerInFollow(params, target);

  double t = 0.0;
  FollowCommand command = planner.update(droneAt(0, 0), t);
  const double yaw_before = command.yaw;

  // The target turns by 90 degrees in one frame.
  const FollowTarget turned =
    targetAt(1, Eigen::Vector3d(10, 0, 0), Eigen::Vector3d(0, 2, 0), kPi / 2);
  t += kDt;
  planner.onTarget(turned, t);
  command = planner.update(droneAt(0, 0), t);

  // The commanded yaw is the bearing to the target: it does not depend on the target's
  // heading, so it does not step.
  EXPECT_NEAR(command.yaw, yaw_before, 1e-9);
  // The framing point moves, but it slides: it is still much closer to the old one
  // than to the new one.
  const double moved = (command.desired_position - Eigen::Vector3d(2, 0, 5)).norm();
  EXPECT_LT(moved, 1.0);
  EXPECT_GT(moved, 0.0);
  // And the filtered heading has barely moved.
  EXPECT_LT(std::abs(planner.smoothedHeading()), 0.1);
}

TEST(HeadingFilter, TheFilteredHeadingConverges)
{
  FollowPlannerParams params = testParams();
  params.heading_tau = 0.8;

  const FollowTarget target = targetAt(1, Eigen::Vector3d(10, 0, 0), Eigen::Vector3d(2, 0, 0), 0.0);
  FollowPlanner planner = plannerInFollow(params, target);

  double t = 0.0;
  planner.update(droneAt(0, 0), t);

  const FollowTarget turned =
    targetAt(1, Eigen::Vector3d(10, 0, 0), Eigen::Vector3d(0, 2, 0), kPi / 2);
  for (int i = 0; i < 500; ++i) {
    t += kDt;
    planner.onTarget(turned, t);
    planner.update(droneAt(0, 0), t);
  }

  // After ten seconds the filter has caught up with the turn.
  EXPECT_NEAR(planner.smoothedHeading(), kPi / 2, 1e-3);
}

TEST(HeadingFilter, TheYawFollowsTheTargetEvenWhileItTurns)
{
  FollowPlannerParams params = testParams();

  // The target walks north, so the drone has to look north.
  const FollowTarget target =
    targetAt(1, Eigen::Vector3d(10, 10, 0), Eigen::Vector3d(0, 2, 0), kPi / 2);
  FollowPlanner planner = plannerInFollow(params, target);

  const FollowCommand command = planner.update(droneAt(0, 0), 0.0);

  EXPECT_NEAR(command.yaw, std::atan2(10.0, 10.0), 1e-9);
}

// ---------------------------------------------------------------------------
// The lost-track machine
// ---------------------------------------------------------------------------

TEST(LostTrack, HoldsThenComesBackHome)
{
  FollowPlannerParams params = testParams();
  params.lost_hold_time = 2.0;
  params.lost_rtl_time = 10.0;

  const FollowTarget target = targetAt(1, Eigen::Vector3d(10, 0, 0), Eigen::Vector3d(2, 0, 0), 0.0);
  FollowPlanner planner = plannerInFollow(params, target);

  double t = 0.0;
  planner.update(droneAt(0, 0), t);
  // The home is where the follow started.
  EXPECT_TRUE(planner.hasHome());
  EXPECT_NEAR(planner.home().x(), 0.0, 1e-9);

  // The target disappears. For the first two seconds the drone keeps flying the last
  // command.
  FollowCommand command;
  for (int i = 0; i < 50; ++i) {
    t += kDt;
    command = planner.update(droneAt(0, 0), t);
  }
  EXPECT_EQ(command.phase, FollowPhase::COAST);
  EXPECT_GT(command.velocity.norm(), 0.0);

  // After the hold time the drone hovers.
  for (int i = 0; i < 100; ++i) {
    t += kDt;
    command = planner.update(droneAt(0, 0), t);
  }
  EXPECT_EQ(command.phase, FollowPhase::HOLD);
  EXPECT_NEAR(command.velocity.norm(), 0.0, 1e-12);
  EXPECT_EQ(command.source, kSourceHold);

  // After the RTL time it goes back to the home position.
  for (int i = 0; i < 400; ++i) {
    t += kDt;
    command = planner.update(droneAt(10, 0), t);
  }
  EXPECT_EQ(command.phase, FollowPhase::RTL);
  EXPECT_EQ(command.source, kSourceRtl);
  // The velocity points at the home, which is behind the drone now.
  EXPECT_LT(command.velocity.x(), 0.0);
  EXPECT_NEAR(command.velocity.x(), 0.8 * (0.0 - 10.0), 1e-9);
}

TEST(LostTrack, TheTargetComingBackResumesTheFollow)
{
  FollowPlannerParams params = testParams();

  const FollowTarget target = targetAt(1, Eigen::Vector3d(10, 0, 0), Eigen::Vector3d(2, 0, 0), 0.0);
  FollowPlanner planner = plannerInFollow(params, target);

  double t = 0.0;
  planner.update(droneAt(0, 0), t);

  // The target goes away for a second, then comes back.
  for (int i = 0; i < 50; ++i) {
    t += kDt;
    planner.update(droneAt(0, 0), t);
  }
  EXPECT_EQ(planner.phase(), FollowPhase::COAST);

  t += kDt;
  planner.onTarget(target, t);
  const FollowCommand command = planner.update(droneAt(0, 0), t);

  EXPECT_EQ(command.phase, FollowPhase::TRACKING);
  EXPECT_EQ(command.source, kSourceFollow);
  EXPECT_NEAR(planner.lostDuration(), 0.0, 1e-12);
}

TEST(LostTrack, WithoutAHomeTheDroneKeepsHolding)
{
  FollowPlannerParams params = testParams();
  params.lost_rtl_time = 1.0;

  // The target is never usable, so the follow never starts and there is no home.
  FollowPlanner planner(params);
  planner.setMode(FollowMode::FOLLOW, 0.0);
  planner.onTarget(targetAt(1, Eigen::Vector3d(10, 0, 0), Eigen::Vector3d(2, 0, 0), 0.0), 0.0);
  // The track is tentative, so it is not a target.
  FollowTarget tentative = targetAt(1, Eigen::Vector3d(10, 0, 0), Eigen::Vector3d(2, 0, 0), 0.0);
  tentative.confirmed = false;
  planner.onTarget(tentative, 0.0);

  double t = 0.0;
  FollowCommand command;
  for (int i = 0; i < 200; ++i) {
    t += kDt;
    planner.onTarget(tentative, t);
    command = planner.update(droneAt(0, 0), t);
  }

  EXPECT_FALSE(planner.hasHome());
  EXPECT_EQ(command.phase, FollowPhase::HOLD);
  EXPECT_NEAR(command.velocity.norm(), 0.0, 1e-12);
}

TEST(LostTrack, AStaleMessageCountsAsLost)
{
  FollowPlannerParams params = testParams();
  params.target_timeout = 0.5;

  const FollowTarget target = targetAt(1, Eigen::Vector3d(10, 0, 0), Eigen::Vector3d(2, 0, 0), 0.0);
  FollowPlanner planner = plannerInFollow(params, target);

  double t = 0.0;
  EXPECT_EQ(planner.update(droneAt(0, 0), t).phase, FollowPhase::TRACKING);

  // No new message: after the timeout the target is lost.
  t += 0.6;
  const FollowCommand command = planner.update(droneAt(0, 0), t);
  EXPECT_NE(command.phase, FollowPhase::TRACKING);
}

TEST(LostTrack, AnUnconfirmedOrInvalidTrackIsNotFollowed)
{
  FollowPlannerParams params = testParams();

  FollowTarget tentative = targetAt(1, Eigen::Vector3d(10, 0, 0), Eigen::Vector3d(2, 0, 0), 0.0);
  tentative.confirmed = false;
  FollowPlanner planner(params);
  planner.setMode(FollowMode::FOLLOW, 0.0);
  planner.onTarget(tentative, 0.0);
  EXPECT_NE(planner.update(droneAt(0, 0), 0.0).phase, FollowPhase::TRACKING);

  FollowTarget coasting = targetAt(1, Eigen::Vector3d(10, 0, 0), Eigen::Vector3d(2, 0, 0), 0.0);
  coasting.is_valid = false;
  FollowPlanner planner2(params);
  planner2.setMode(FollowMode::FOLLOW, 0.0);
  planner2.onTarget(coasting, 0.0);
  EXPECT_NE(planner2.update(droneAt(0, 0), 0.0).phase, FollowPhase::TRACKING);
}

// ---------------------------------------------------------------------------
// The operator's track and the mode gate
// ---------------------------------------------------------------------------

TEST(OperatorTrack, OnlyTheSelectedTrackIsFollowed)
{
  FollowPlannerParams params = testParams();

  // The operator picked track 7, but the tracker publishes track 3.
  FollowPlanner planner(params);
  planner.setOperatorTrackId(7);
  planner.setMode(FollowMode::FOLLOW, 0.0);
  planner.onTarget(targetAt(3, Eigen::Vector3d(10, 0, 0), Eigen::Vector3d(2, 0, 0), 0.0), 0.0);

  FollowCommand command = planner.update(droneAt(0, 0), 0.0);
  EXPECT_NE(command.phase, FollowPhase::TRACKING);
  EXPECT_NE(command.source, kSourceFollow);

  // The right track arrives: now it is followed.
  planner.onTarget(targetAt(7, Eigen::Vector3d(10, 0, 0), Eigen::Vector3d(2, 0, 0), 0.0), 0.0);
  command = planner.update(droneAt(0, 0), 0.0);
  EXPECT_EQ(command.phase, FollowPhase::TRACKING);
  EXPECT_EQ(command.source, kSourceFollow);
}

TEST(OperatorTrack, TheSelectionIsIgnoredWhenTheOperatorDidNotChoose)
{
  FollowPlannerParams params = testParams();

  FollowPlanner planner(params);
  planner.setOperatorTrackId(-1);
  planner.setMode(FollowMode::FOLLOW, 0.0);
  planner.onTarget(targetAt(3, Eigen::Vector3d(10, 0, 0), Eigen::Vector3d(2, 0, 0), 0.0), 0.0);

  EXPECT_EQ(planner.update(droneAt(0, 0), 0.0).phase, FollowPhase::TRACKING);
}

TEST(ModeGate, ThePlannerIsSilentOutsideTheFollowMode)
{
  FollowPlannerParams params = testParams();
  const FollowTarget target = targetAt(1, Eigen::Vector3d(10, 0, 0), Eigen::Vector3d(2, 0, 0), 0.0);

  FollowPlanner planner(params);
  planner.onTarget(target, 0.0);

  planner.setMode(FollowMode::INTERCEPT, 0.0);
  FollowCommand command = planner.update(droneAt(0, 0), 0.0);
  EXPECT_FALSE(command.active);
  EXPECT_EQ(command.phase, FollowPhase::IDLE);

  planner.setMode(FollowMode::HOLD, 0.0);
  command = planner.update(droneAt(0, 0), 0.0);
  EXPECT_FALSE(command.active);
  EXPECT_EQ(command.phase, FollowPhase::IDLE);

  // And in FOLLOW it speaks again.
  planner.setMode(FollowMode::FOLLOW, 0.0);
  command = planner.update(droneAt(0, 0), 0.0);
  EXPECT_TRUE(command.active);
  EXPECT_EQ(command.phase, FollowPhase::TRACKING);
}

TEST(ModeGate, EnteringTheFollowModeStartsANewSession)
{
  FollowPlannerParams params = testParams();
  const FollowTarget target = targetAt(1, Eigen::Vector3d(10, 0, 0), Eigen::Vector3d(2, 0, 0), 0.0);

  FollowPlanner planner(params);
  planner.setMode(FollowMode::FOLLOW, 0.0);
  planner.onTarget(target, 0.0);
  planner.update(droneAt(5, 0), 0.0);
  EXPECT_TRUE(planner.hasHome());
  EXPECT_NEAR(planner.home().x(), 5.0, 1e-9);

  // Leaving FOLLOW and coming back forgets the old home: the new session has its own.
  planner.setMode(FollowMode::HOLD, 1.0);
  planner.setMode(FollowMode::FOLLOW, 2.0);
  EXPECT_FALSE(planner.hasHome());
}

}  // namespace
}  // namespace follow
}  // namespace interceptor
