/**
 * @file test_guidance.cpp
 * @brief Unit tests for the PN guidance law.
 *
 * The tests check the geometry (LOS rate, closing speed, time to go), the two guidance
 * laws and the closed-loop behaviour: pure PN must intercept a constant-velocity target,
 * and augmented PN must intercept an accelerating target that pure PN misses.
 */

#include <gtest/gtest.h>

#include <cmath>

#include "guidance/pn_guidance.hpp"

using namespace interceptor::guidance;  // NOLINT(build/namespaces)

namespace
{

constexpr double kEps = 1e-9;

/// Engagement with the interceptor at the origin, at rest, and the target on the +x axis.
EngagementState makeEngagement(
  const Eigen::Vector3d & target_position, const Eigen::Vector3d & target_velocity,
  const Eigen::Vector3d & target_acceleration = Eigen::Vector3d::Zero())
{
  EngagementState state;
  state.interceptor_position = Eigen::Vector3d::Zero();
  state.interceptor_velocity = Eigen::Vector3d::Zero();
  state.target_position = target_position;
  state.target_velocity = target_velocity;
  state.target_acceleration = target_acceleration;
  return state;
}

/// Same, but the interceptor is already moving towards the target.
EngagementState makeClosingEngagement(
  const Eigen::Vector3d & target_position, const Eigen::Vector3d & target_velocity,
  const Eigen::Vector3d & interceptor_velocity)
{
  EngagementState state = makeEngagement(target_position, target_velocity);
  state.interceptor_velocity = interceptor_velocity;
  return state;
}

/// A target flying straight away along +x at constant speed: the classic PN case.
EngagementState constantVelocityEngagement(double range = 50.0, double speed = 5.0)
{
  return makeEngagement(Eigen::Vector3d(range, 0.0, 0.0), Eigen::Vector3d(speed, 0.0, 0.0));
}

}  // namespace

// ---------------------------------------------------------------------------
// Law parsing
// ---------------------------------------------------------------------------

TEST(PnGuidanceTest, ParsesLawNames)
{
  EXPECT_EQ(parseGuidanceLaw("pn"), GuidanceLaw::PURE_PN);
  EXPECT_EQ(parseGuidanceLaw("PN"), GuidanceLaw::PURE_PN);
  EXPECT_EQ(parseGuidanceLaw("apn"), GuidanceLaw::AUGMENTED_PN);
  EXPECT_EQ(parseGuidanceLaw("APN"), GuidanceLaw::AUGMENTED_PN);
  EXPECT_EQ(parseGuidanceLaw("Apn"), GuidanceLaw::AUGMENTED_PN);
}

TEST(PnGuidanceTest, RejectsUnknownLawNames)
{
  EXPECT_FALSE(parseGuidanceLaw("").has_value());
  EXPECT_FALSE(parseGuidanceLaw("proportional").has_value());
  EXPECT_FALSE(parseGuidanceLaw("apn2").has_value());
}

TEST(PnGuidanceTest, LawNamesRoundTrip)
{
  EXPECT_EQ(toString(GuidanceLaw::PURE_PN), "pn");
  EXPECT_EQ(toString(GuidanceLaw::AUGMENTED_PN), "apn");
  EXPECT_EQ(parseGuidanceLaw(toString(GuidanceLaw::PURE_PN)), GuidanceLaw::PURE_PN);
  EXPECT_EQ(parseGuidanceLaw(toString(GuidanceLaw::AUGMENTED_PN)), GuidanceLaw::AUGMENTED_PN);
}

// ---------------------------------------------------------------------------
// LOS kinematics
// ---------------------------------------------------------------------------

TEST(LosKinematicsTest, ComputesRangeAndLosUnit)
{
  const EngagementState state = makeEngagement(
    Eigen::Vector3d(30.0, 40.0, 0.0), Eigen::Vector3d::Zero());
  const LosKinematics los = computeLosKinematics(state);

  EXPECT_NEAR(los.range, 50.0, kEps);
  EXPECT_NEAR(los.los_unit.x(), 0.6, kEps);
  EXPECT_NEAR(los.los_unit.y(), 0.8, kEps);
  EXPECT_NEAR(los.los_unit.norm(), 1.0, kEps);
}

TEST(LosKinematicsTest, HandlesARecedingTarget)
{
  const EngagementState state = constantVelocityEngagement(50.0, 5.0);
  const LosKinematics los = computeLosKinematics(state);

  EXPECT_NEAR(los.range, 50.0, kEps);
  EXPECT_NEAR(los.closing_velocity, -5.0, kEps);
  EXPECT_NEAR(los.los_rate.norm(), 0.0, kEps);
}

TEST(LosKinematicsTest, ClosingSpeedIsPositiveWhenClosing)
{
  // Target at +x moving towards the interceptor at -x: the range shrinks.
  const EngagementState closing = makeEngagement(
    Eigen::Vector3d(50.0, 0.0, 0.0), Eigen::Vector3d(-5.0, 0.0, 0.0));
  const LosKinematics los = computeLosKinematics(closing);
  EXPECT_NEAR(los.closing_velocity, 5.0, kEps);
  EXPECT_NEAR(los.time_to_go, 10.0, kEps);
}

TEST(LosKinematicsTest, ClosingSpeedIsNegativeWhenSeparating)
{
  const EngagementState separating = makeEngagement(
    Eigen::Vector3d(50.0, 0.0, 0.0), Eigen::Vector3d(5.0, 0.0, 0.0));
  const LosKinematics los = computeLosKinematics(separating);
  EXPECT_NEAR(los.closing_velocity, -5.0, kEps);
  // No finite time to go when the target is not approaching.
  EXPECT_NEAR(los.time_to_go, 0.0, kEps);
}

TEST(LosKinematicsTest, LosRateIsZeroForAClosingCollinearTarget)
{
  // The relative velocity is entirely along the LOS, so it does not rotate the LOS.
  const EngagementState state = makeEngagement(
    Eigen::Vector3d(50.0, 0.0, 0.0), Eigen::Vector3d(-5.0, 0.0, 0.0));
  const LosKinematics los = computeLosKinematics(state);
  EXPECT_NEAR(los.los_rate.norm(), 0.0, kEps);
}

TEST(LosKinematicsTest, LosRateMeasuresTheRotationOfTheLos)
{
  // Target 100 m away on +x, moving at 1 m/s along +y: the LOS rotates at
  // (1 m/s * 100 m) / (100 m)^2 = 0.01 rad/s. The rate vector is the relative
  // velocity perpendicular to the LOS divided by the range, so it points along +y.
  const EngagementState state = makeEngagement(
    Eigen::Vector3d(100.0, 0.0, 0.0), Eigen::Vector3d(0.0, 1.0, 0.0));
  const LosKinematics los = computeLosKinematics(state);
  EXPECT_NEAR(los.los_rate.y(), 0.01, 1e-12);
  EXPECT_NEAR(los.los_rate.x(), 0.0, 1e-12);
  EXPECT_NEAR(los.los_rate.norm(), 0.01, 1e-12);
}

TEST(LosKinematicsTest, DegenerateRangeDoesNotCrash)
{
  const EngagementState state = makeEngagement(
    Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  const LosKinematics los = computeLosKinematics(state);
  EXPECT_NEAR(los.range, 0.0, kEps);
  EXPECT_NEAR(los.closing_velocity, 0.0, kEps);
  EXPECT_NEAR(los.los_rate.norm(), 0.0, kEps);
}

// ---------------------------------------------------------------------------
// Pure PN
// ---------------------------------------------------------------------------

TEST(PurePnTest, CommandIsPerpendicularToTheLos)
{
  // The interceptor is moving towards the target, so the engagement is closing.
  const EngagementState state = makeClosingEngagement(
    Eigen::Vector3d(100.0, 0.0, 0.0), Eigen::Vector3d(0.0, 1.0, 0.0),
    Eigen::Vector3d(5.0, 0.0, 0.0));
  const LosKinematics los = computeLosKinematics(state);
  const Eigen::Vector3d accel = purePn(los, 4.0);

  EXPECT_NEAR(accel.dot(los.los_unit), 0.0, 1e-12);
  EXPECT_GT(accel.norm(), 0.0);
}

TEST(PurePnTest, MagnitudeIsNavConstantTimesClosingSpeedTimesLosRate)
{
  const EngagementState state = makeEngagement(
    Eigen::Vector3d(100.0, 0.0, 0.0), Eigen::Vector3d(0.0, 1.0, 0.0));
  const LosKinematics los = computeLosKinematics(state);
  const Eigen::Vector3d accel = purePn(los, 4.0);

  EXPECT_NEAR(accel.norm(), 4.0 * los.closing_velocity * los.los_rate.norm(), 1e-12);
}

TEST(PurePnTest, NoCommandForACollinearTarget)
{
  // Nothing to correct: the target is already on the LOS.
  const EngagementState state = makeEngagement(
    Eigen::Vector3d(100.0, 0.0, 0.0), Eigen::Vector3d(-2.0, 0.0, 0.0));
  const LosKinematics los = computeLosKinematics(state);
  EXPECT_NEAR(purePn(los, 4.0).norm(), 0.0, kEps);
}

TEST(PurePnTest, CommandTurnsTheLosOntoTheTarget)
{
  // The target slides to +y, so the interceptor must accelerate towards +y as well.
  const EngagementState state = makeClosingEngagement(
    Eigen::Vector3d(100.0, 0.0, 0.0), Eigen::Vector3d(0.0, 1.0, 0.0),
    Eigen::Vector3d(5.0, 0.0, 0.0));
  const LosKinematics los = computeLosKinematics(state);
  const Eigen::Vector3d accel = purePn(los, 4.0);
  EXPECT_GT(accel.y(), 0.0);
}

// ---------------------------------------------------------------------------
// Augmented PN
// ---------------------------------------------------------------------------

TEST(AugmentedPnTest, AddsTheTargetAccelerationTerm)
{
  const EngagementState state = makeEngagement(
    Eigen::Vector3d(100.0, 0.0, 0.0), Eigen::Vector3d(0.0, 1.0, 0.0),
    Eigen::Vector3d(0.0, 2.0, 0.0));
  const LosKinematics los = computeLosKinematics(state);

  const Eigen::Vector3d pure = purePn(los, 4.0);
  const Eigen::Vector3d augmented = augmentedPn(los, state.target_acceleration, 4.0, 0.5, true);

  // The target acceleration is along +y, perpendicular to the LOS, so it is added
  // with the (N/2) factor: 4.0 * 0.5 * 2.0 = 4.0 m/s^2.
  EXPECT_NEAR(augmented.y() - pure.y(), 4.0, 1e-12);
  EXPECT_NEAR(augmented.x(), pure.x(), 1e-12);
}

TEST(AugmentedPnTest, WithZeroGainItEqualsPurePn)
{
  const EngagementState state = makeEngagement(
    Eigen::Vector3d(100.0, 0.0, 0.0), Eigen::Vector3d(0.0, 1.0, 0.0),
    Eigen::Vector3d(0.0, 2.0, 0.0));
  const LosKinematics los = computeLosKinematics(state);

  const Eigen::Vector3d augmented =
    augmentedPn(los, state.target_acceleration, 4.0, 0.0, true);
  EXPECT_NEAR((augmented - purePn(los, 4.0)).norm(), 0.0, 1e-12);
}

TEST(AugmentedPnTest, ProjectsTheTargetAccelerationPerpendicularToTheLos)
{
  // The target accelerates along the LOS (away): the perpendicular part is zero, so
  // the augmented law reduces to the pure one.
  const EngagementState state = makeEngagement(
    Eigen::Vector3d(100.0, 0.0, 0.0), Eigen::Vector3d(0.0, 1.0, 0.0),
    Eigen::Vector3d(3.0, 0.0, 0.0));
  const LosKinematics los = computeLosKinematics(state);

  const Eigen::Vector3d augmented =
    augmentedPn(los, state.target_acceleration, 4.0, 0.5, true);
  const Eigen::Vector3d pure = purePn(los, 4.0);
  EXPECT_NEAR((augmented - pure).norm(), 0.0, 1e-12);
}

TEST(AugmentedPnTest, WithoutProjectionTheAlongLosPartIsKept)
{
  const EngagementState state = makeEngagement(
    Eigen::Vector3d(100.0, 0.0, 0.0), Eigen::Vector3d(0.0, 1.0, 0.0),
    Eigen::Vector3d(3.0, 0.0, 0.0));
  const LosKinematics los = computeLosKinematics(state);

  const Eigen::Vector3d augmented =
    augmentedPn(los, state.target_acceleration, 4.0, 0.5, false);
  const Eigen::Vector3d pure = purePn(los, 4.0);
  // The along-LOS component 3.0 * (4.0 * 0.5) = 6.0 m/s^2 is added along +x.
  EXPECT_NEAR(augmented.x() - pure.x(), 6.0, 1e-12);
}

// ---------------------------------------------------------------------------
// Zero-effort miss and saturation
// ---------------------------------------------------------------------------

TEST(ZeroEffortMissTest, IsTheRangeWhenTheTargetCrossesPerpendicularly)
{
  // The interceptor sits still and the target passes 100 m away, perpendicular to the
  // LOS. Neither vehicle accelerates, so the target is never hit: the miss distance
  // is the full range.
  const EngagementState state = makeEngagement(
    Eigen::Vector3d(100.0, 0.0, 0.0), Eigen::Vector3d(0.0, 1.0, 0.0));
  const LosKinematics los = computeLosKinematics(state);
  EXPECT_NEAR(zeroEffortMiss(los), 100.0, 1e-9);
}

TEST(ZeroEffortMissTest, IsZeroWhenTheTargetMovesStraightAtTheInterceptor)
{
  // The target comes straight down the LOS: without any manoeuvre it is hit.
  const EngagementState state = makeEngagement(
    Eigen::Vector3d(100.0, 0.0, 0.0), Eigen::Vector3d(-1.0, 0.0, 0.0));
  const LosKinematics los = computeLosKinematics(state);
  EXPECT_NEAR(zeroEffortMiss(los), 0.0, 1e-9);
}

TEST(ZeroEffortMissTest, IsTheRangeWhenNothingMoves)
{
  const EngagementState state = makeEngagement(
    Eigen::Vector3d(30.0, 40.0, 0.0), Eigen::Vector3d::Zero());
  const LosKinematics los = computeLosKinematics(state);
  EXPECT_NEAR(zeroEffortMiss(los), 50.0, 1e-9);
}

TEST(SaturationTest, LimitsTheNormAndKeepsTheDirection)
{
  const Eigen::Vector3d accel(30.0, 40.0, 0.0);  // norm 50
  const Eigen::Vector3d saturated = saturateAcceleration(accel, 15.0);
  EXPECT_NEAR(saturated.norm(), 15.0, 1e-12);
  EXPECT_NEAR(saturated.x() / 30.0, saturated.y() / 40.0, 1e-12);
}

TEST(SaturationTest, LeavesSmallCommandsAlone)
{
  const Eigen::Vector3d accel(1.0, 2.0, 3.0);
  EXPECT_NEAR((saturateAcceleration(accel, 15.0) - accel).norm(), 0.0, 1e-12);
}

TEST(SaturationTest, HandlesAZeroCommand)
{
  EXPECT_NEAR(saturateAcceleration(Eigen::Vector3d::Zero(), 15.0).norm(), 0.0, 1e-12);
}

// ---------------------------------------------------------------------------
// Velocity setpoint
// ---------------------------------------------------------------------------

TEST(VelocitySetpointTest, AlongTheLosItCommandsTheClosingSpeed)
{
  // The target is closing along the LOS at 5 m/s and the LOS is the +x axis.
  const EngagementState state = makeClosingEngagement(
    Eigen::Vector3d(100.0, 0.0, 0.0), Eigen::Vector3d(-5.0, 0.0, 0.0),
    Eigen::Vector3d(0.0, 0.0, 0.0));
  const LosKinematics los = computeLosKinematics(state);

  const Eigen::Vector3d velocity =
    pnVelocitySetpoint(los, Eigen::Vector3d::Zero(), 0.2, 8.0);
  EXPECT_NEAR(velocity.x(), 5.0, 1e-12);
  EXPECT_NEAR(velocity.y(), 0.0, 1e-12);
}

TEST(VelocitySetpointTest, TheClosingSpeedIsCapped)
{
  const EngagementState state = makeClosingEngagement(
    Eigen::Vector3d(100.0, 0.0, 0.0), Eigen::Vector3d(-20.0, 0.0, 0.0),
    Eigen::Vector3d(0.0, 0.0, 0.0));
  const LosKinematics los = computeLosKinematics(state);

  const Eigen::Vector3d velocity =
    pnVelocitySetpoint(los, Eigen::Vector3d::Zero(), 0.2, 8.0);
  EXPECT_NEAR(velocity.x(), 8.0, 1e-12);
}

TEST(VelocitySetpointTest, TheGuidanceAccelerationIsTheLateralPart)
{
  // The target drifts to +y, so the guidance pushes the velocity sideways.
  const EngagementState state = makeClosingEngagement(
    Eigen::Vector3d(100.0, 0.0, 0.0), Eigen::Vector3d(0.0, 1.0, 0.0),
    Eigen::Vector3d(5.0, 0.0, 0.0));
  const LosKinematics los = computeLosKinematics(state);

  const Eigen::Vector3d accel = purePn(los, 4.0);
  const Eigen::Vector3d velocity = pnVelocitySetpoint(los, accel, 0.2, 8.0);

  // The lateral part is the acceleration times the gain.
  EXPECT_NEAR(velocity.y(), accel.y() * 0.2, 1e-12);
  EXPECT_GT(velocity.y(), 0.0);
}

TEST(VelocitySetpointTest, TheSetpointIsTheOnlyLateralAuthority)
{
  // With no guidance acceleration the setpoint points along the LOS, so the vehicle
  // does not drift towards the target's lateral motion on its own.
  const EngagementState state = makeClosingEngagement(
    Eigen::Vector3d(100.0, 0.0, 0.0), Eigen::Vector3d(0.0, 5.0, 0.0),
    Eigen::Vector3d(5.0, 0.0, 0.0));
  const LosKinematics los = computeLosKinematics(state);

  const Eigen::Vector3d velocity =
    pnVelocitySetpoint(los, Eigen::Vector3d::Zero(), 0.2, 8.0);
  EXPECT_NEAR(velocity.y(), 0.0, 1e-12);
  // The LOS is not exactly the +x axis here, so the setpoint follows it.
  EXPECT_NEAR(velocity.dot(los.los_unit), velocity.norm(), 1e-12);
}

// ---------------------------------------------------------------------------
// PnGuidance
// ---------------------------------------------------------------------------

TEST(PnGuidanceTest, ReportsSaturation)
{
  GuidanceParams params;
  params.law = GuidanceLaw::PURE_PN;
  params.nav_constant = 4.0;
  params.max_accel = 2.0;
  const PnGuidance guidance(params);

  // A large LOS rate with a big closing speed gives a large command.
  EngagementState state = makeEngagement(
    Eigen::Vector3d(20.0, 0.0, 0.0), Eigen::Vector3d(0.0, 40.0, 0.0));
  state.interceptor_velocity = Eigen::Vector3d(-30.0, 0.0, 0.0);

  const GuidanceResult result = guidance.update(state);
  EXPECT_TRUE(result.saturated);
  EXPECT_NEAR(result.acceleration.norm(), 2.0, 1e-12);
  EXPECT_GT(result.acceleration_unsaturated.norm(), 2.0);
}

TEST(PnGuidanceTest, SelectsTheRequestedLaw)
{
  GuidanceParams params;
  params.law = GuidanceLaw::PURE_PN;
  const PnGuidance pure(params);

  EngagementState state = makeEngagement(
    Eigen::Vector3d(100.0, 0.0, 0.0), Eigen::Vector3d(0.0, 1.0, 0.0),
    Eigen::Vector3d(0.0, 2.0, 0.0));

  const GuidanceResult pure_result = pure.update(state);
  EXPECT_EQ(pure_result.law, GuidanceLaw::PURE_PN);
  EXPECT_NEAR(pure_result.acceleration.y(), purePn(pure_result.los, 4.0).y(), 1e-12);

  params.law = GuidanceLaw::AUGMENTED_PN;
  const PnGuidance augmented(params);
  const GuidanceResult apn_result = augmented.update(state);
  EXPECT_EQ(apn_result.law, GuidanceLaw::AUGMENTED_PN);
  EXPECT_GT(apn_result.acceleration.y(), pure_result.acceleration.y());
}

// ---------------------------------------------------------------------------
// Closed loop: the law must actually intercept
// ---------------------------------------------------------------------------

namespace
{

/// One step of a closed-loop interception: the guidance produces an acceleration, the
/// interceptor accelerates towards it (bounded), both vehicles move.
void stepInterception(
  const PnGuidance & guidance, Eigen::Vector3d & interceptor_position,
  Eigen::Vector3d & interceptor_velocity, const Eigen::Vector3d & target_position,
  const Eigen::Vector3d & target_velocity, const Eigen::Vector3d & target_acceleration,
  double dt, double max_speed)
{
  EngagementState state;
  state.interceptor_position = interceptor_position;
  state.interceptor_velocity = interceptor_velocity;
  state.target_position = target_position;
  state.target_velocity = target_velocity;
  state.target_acceleration = target_acceleration;

  const GuidanceResult result = guidance.update(state);
  interceptor_velocity += result.acceleration * dt;
  const double speed = interceptor_velocity.norm();
  if (speed > max_speed) {
    interceptor_velocity *= max_speed / speed;
  }
  interceptor_position += interceptor_velocity * dt;
}

/// Runs a whole engagement and returns the closest range reached [m].
///
/// The target moves with a constant acceleration, so its state is integrated with the
/// same step as the interceptor.
double runEngagement(
  GuidanceLaw law, const Eigen::Vector3d & interceptor_position,
  const Eigen::Vector3d & interceptor_velocity, const Eigen::Vector3d & target_position,
  const Eigen::Vector3d & target_velocity, const Eigen::Vector3d & target_acceleration,
  double duration, double dt, double max_speed)
{
  GuidanceParams params;
  params.law = law;
  params.nav_constant = 4.0;
  params.max_accel = 15.0;
  const PnGuidance guidance(params);

  Eigen::Vector3d interceptor_pos = interceptor_position;
  Eigen::Vector3d interceptor_vel = interceptor_velocity;
  Eigen::Vector3d target_pos = target_position;
  Eigen::Vector3d target_vel = target_velocity;

  double min_range = 1e9;
  for (double t = 0.0; t < duration; t += dt) {
    stepInterception(
      guidance, interceptor_pos, interceptor_vel, target_pos, target_vel, target_acceleration,
      dt, max_speed);
    min_range = std::min(min_range, (target_pos - interceptor_pos).norm());

    // The target keeps accelerating and moving.
    target_pos += target_vel * dt;
    target_vel += target_acceleration * dt;
  }
  return min_range;
}

}  // namespace

TEST(ClosedLoopTest, PurePnInterceptsAConstantVelocityTarget)
{
  // The interceptor is faster than the target but starts off to one side, so it has to
  // lead it. Pure PN must still close the range to zero.
  const double min_range = runEngagement(
    GuidanceLaw::PURE_PN,
    Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(12.0, 0.0, 0.0),
    Eigen::Vector3d(40.0, 0.0, 0.0), Eigen::Vector3d(4.0, 0.0, 0.0),
    Eigen::Vector3d::Zero(), 20.0, 0.002, 20.0);

  EXPECT_LT(min_range, 1.0);
}

TEST(ClosedLoopTest, AugmentedPnInterceptsAnAcceleratingTargetThatPurePnMisses)
{
  // The target accelerates sideways, so the LOS bends away from the interceptor.
  // Pure PN cannot follow that bend and misses, augmented PN compensates it.
  //
  // The speed cap of the interceptor is deliberately not binding here: the PN law
  // assumes the interceptor can realise the acceleration it asks for. A platform that
  // cannot keep up with the target is a different problem (the guidance saturates
  // and the capture range is never reached), not a property of the law.
  const Eigen::Vector3d interceptor_position(0.0, 0.0, 0.0);
  const Eigen::Vector3d interceptor_velocity(12.0, 0.0, 0.0);
  const Eigen::Vector3d target_position(40.0, 0.0, 0.0);
  const Eigen::Vector3d target_velocity(4.0, 0.0, 0.0);
  const Eigen::Vector3d target_acceleration(0.0, 3.0, 0.0);

  const double pure_miss = runEngagement(
    GuidanceLaw::PURE_PN, interceptor_position, interceptor_velocity, target_position,
    target_velocity, target_acceleration, 20.0, 0.002, 20.0);
  const double apn_miss = runEngagement(
    GuidanceLaw::AUGMENTED_PN, interceptor_position, interceptor_velocity, target_position,
    target_velocity, target_acceleration, 20.0, 0.002, 20.0);

  // Augmented PN must intercept, pure PN must miss by a wide margin.
  EXPECT_LT(apn_miss, 1.0);
  EXPECT_GT(pure_miss, 2.0);
}
