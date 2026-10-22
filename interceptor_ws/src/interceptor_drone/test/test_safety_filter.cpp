#include <gtest/gtest.h>
#include <Eigen/Dense>

#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include "safety/safety_filter.hpp"

namespace interceptor
{
namespace safety
{
namespace
{

// TargetClass values, as in common/mission.hpp.
constexpr int kPerson = 0;
constexpr int kCar = 1;
constexpr int kTruck = 2;
constexpr int kBicycle = 3;
constexpr int kUav = 4;

constexpr double kPi = 3.14159265358979323846;

/// The filter with the plan's defaults, except the drone radius: the tests measure
/// distances between centres, so the surface-to-surface correction is left at zero
/// unless a test asks for it.
SafetyFilterParams testParams()
{
  SafetyFilterParams params;
  params.drone_radius = 0.0;
  return params;
}

KeepOutTarget person(int id, double x, double y, double z = 0.0)
{
  KeepOutTarget target;
  target.track_id = id;
  target.class_id = kPerson;
  target.position = Eigen::Vector3d(x, y, z);
  return target;
}

/// A setpoint that flies the drone at a given horizontal velocity.
FilterSetpoint velocitySetpoint(const Eigen::Vector3d & velocity)
{
  FilterSetpoint setpoint;
  setpoint.velocity = velocity;
  setpoint.position_valid = false;
  return setpoint;
}

/// A setpoint that asks the drone to fly to a position.
FilterSetpoint positionSetpoint(const Eigen::Vector3d & position)
{
  FilterSetpoint setpoint;
  setpoint.position = position;
  setpoint.position_valid = true;
  return setpoint;
}

/// The drone's state. The default altitude is the height of a person: the tests
/// are about the keep-out of a person at the drone's own level, which is the
/// case the plan's horizontal rule is written for.
FilterState stateAt(double x, double y, double z = 1.5)
{
  FilterState state;
  state.position = Eigen::Vector3d(x, y, z);
  return state;
}

/// The largest closing speed the barrier allows, computed the way the filter does.
double maxClosingSpeed(
  double h, double closing, double target_closing, const SafetyFilterParams & params)
{
  const double plan_bound = params.cbf_alpha * h;
  const double lag_bound =
    closing * (1.0 - 2.0 * params.barrier_omega * params.velocity_lag_tau) +
    params.barrier_omega * params.barrier_omega * params.velocity_lag_tau * h;
  double bound = std::min(plan_bound, lag_bound);
  if (target_closing > 0.0) {
    bound = std::min(bound, 0.0);
  }
  return bound;
}

/// The horizontal standoff of a track at a given height above it, computed the
/// way the filter computes it (see the header of safety_filter.hpp).
double horizontalStandoff(
  double vertical_separation, const SafetyFilterParams & params, double sigma)
{
  const double relief =
    std::max(0.0, vertical_separation - params.keepout_horizontal_altitude);
  const double nominal =
    params.d_min_horizontal - relief * params.keepout_horizontal_taper;
  if (nominal <= 0.0) {
    return 0.0;
  }
  return nominal + params.uncertainty_gain * sigma;
}

/// A constraint the velocity has to satisfy: normal . v >= bound.
struct VelocityConstraint
{
  Eigen::Vector3d normal;
  double bound = 0.0;
  std::string name;
};

/// The constraints the filter enforces for one track, computed the way the
/// filter computes them: the 3D barrier (the safety distance) and, when it is
/// in force, the horizontal barrier (the plan's flat standoff).
std::vector<VelocityConstraint> trackConstraints(
  const SafetyFilterParams & params, const KeepOutTarget & target, const FilterState & state)
{
  std::vector<VelocityConstraint> out;

  // The 3D barrier.
  const Eigen::Vector3d delta = state.position - target.position;
  const double distance3 = delta.norm();
  if (distance3 > 1e-6) {
    const Eigen::Vector3d normal = delta / distance3;
    const double h3 = distance3 -
      (params.d_min_3d + params.uncertainty_gain * target.position_sigma) -
      params.drone_radius;
    const double target_los = normal.dot(target.velocity);
    const double closing = target_los - normal.dot(state.velocity);
    const double c_cmd_max =
      maxClosingSpeed(h3, closing, target_los, params);
    out.push_back({normal, target_los - c_cmd_max, "3D"});
  }

  // The horizontal barrier, when the standoff is not given up at this height.
  const double standoff =
    horizontalStandoff(state.position.z() - target.position.z(), params, target.position_sigma);
  const Eigen::Vector2d delta2 = state.position.head<2>() - target.position.head<2>();
  const double distance2 = delta2.norm();
  if (standoff > 0.0 && distance2 > 1e-6) {
    const Eigen::Vector3d normal(delta2.x() / distance2, delta2.y() / distance2, 0.0);
    const double h2 = distance2 - standoff - params.drone_radius;
    const double target_los = normal.dot(target.velocity);
    const double closing = target_los - normal.dot(state.velocity);
    const double c_cmd_max =
      maxClosingSpeed(h2, closing, target_los, params);
    out.push_back({normal, target_los - c_cmd_max, "horizontal"});
  }
  return out;
}

/// Checks that a velocity satisfies every constraint of a track.
void expectSatisfiesTrack(
  const SafetyFilterParams & params, const KeepOutTarget & target, const FilterState & state,
  const Eigen::Vector3d & velocity)
{
  for (const VelocityConstraint & constraint : trackConstraints(params, target, state)) {
    EXPECT_GE(constraint.normal.dot(velocity), constraint.bound - 1e-6)
      << "the " << constraint.name << " constraint of track " << target.track_id
      << " was violated";
  }
}

/// Runs the filter with the default "everything is fresh" arguments.
FilterResult run(
  SafetyFilter & filter, const FilterSetpoint & setpoint, const FilterState & state,
  const std::vector<KeepOutTarget> & targets, const std::vector<Obstacle> & obstacles = {},
  FilterMode mode = FilterMode::FOLLOW)
{
  return filter.filter(
    setpoint, state, targets, obstacles, mode, 10.0, 0.0, 0.0, false, true);
}

/// The plant of the closed loop: the autopilot's velocity loop, a first-order lag.
/// Its time constant is the one measured on the X500 flight model (0.34 s).
struct LagPlant
{
  FilterState state;
  double tau = 0.34;  // s

  void step(const Eigen::Vector3d & command, double dt)
  {
    state.velocity += (command - state.velocity) * (1.0 - std::exp(-dt / tau));
    state.position += state.velocity * dt;
  }
};

/// Closes the filter around a plant that flies towards a person: the filter is asked
/// for a velocity every tick and the plant follows it. Returns the smallest 3D
/// margin above the safety distance that was reached.
double closedLoopMinimumMargin(
  SafetyFilter & filter, const std::vector<KeepOutTarget> & targets, double seconds = 30.0,
  double dt = 0.02, const Eigen::Vector3d & initial_velocity = Eigen::Vector3d::Zero(),
  FilterMode mode = FilterMode::FOLLOW)
{
  LagPlant plant;
  plant.state = stateAt(0.0, 0.0);
  plant.state.velocity = initial_velocity;

  const double time_end = 10.0 + seconds;
  double minimum = std::numeric_limits<double>::infinity();
  for (double t = 10.0; t < time_end; t += dt) {
    // The planner always commands the drone straight at the person.
    const FilterSetpoint setpoint = velocitySetpoint(Eigen::Vector3d(10.0, 0.0, 0.0));
    const FilterResult result =
      filter.filter(setpoint, plant.state, targets, {}, mode, t, 0.0, 0.0, false, true);
    plant.step(result.velocity, dt);

    for (const KeepOutTarget & target : targets) {
      const double margin = (plant.state.position - target.position).norm() -
        filter.params().d_min_3d;
      minimum = std::min(minimum, margin);
    }
  }
  return minimum;
}

// ---------------------------------------------------------------------------
// The scenarios the plan asks for
// ---------------------------------------------------------------------------

/// A drone commanded straight at a standing person stops at >= d_min.
///
/// The check is on the outcome, not on the returned command: the barrier is a
/// constraint on the closing speed, and the drone coasts the last metres while the
/// velocity loop catches up.
TEST(SafetyFilterTest, CommandedAtAPersonStopsAtTheKeepOutDistance)
{
  SafetyFilter filter(testParams());
  const std::vector<KeepOutTarget> targets = {person(1, 30.0, 0.0)};

  const double margin = closedLoopMinimumMargin(filter, targets);

  EXPECT_GE(margin, -1e-3) << "the drone came within " << -margin << " m of the limit";
}

/// A person walking towards a hovering drone makes the drone back off.
TEST(SafetyFilterTest, APersonWalkingAtTheDroneMakesItBackOff)
{
  SafetyFilter filter(testParams());
  const FilterState state = stateAt(0.0, 0.0);
  KeepOutTarget walker = person(1, 8.0, 0.0);
  // The person walks towards the drone at 1.5 m/s.
  walker.velocity = Eigen::Vector3d(-1.5, 0.0, 0.0);
  const std::vector<KeepOutTarget> targets = {walker};

  const FilterSetpoint setpoint = velocitySetpoint(Eigen::Vector3d::Zero());
  const FilterResult result = run(filter, setpoint, state, targets);

  EXPECT_TRUE(result.safety_active);
  EXPECT_LT(result.velocity.x(), 0.0) << "the drone did not back off";
}

/// Two bystanders plus the target: no constraint is violated.
TEST(SafetyFilterTest, SeveralBystandersPlusTheTargetViolateNothing)
{
  SafetyFilter filter(testParams());
  const FilterState state = stateAt(0.0, 0.0);
  const std::vector<KeepOutTarget> targets = {
    person(1, 10.0, 0.0),
    person(2, -8.0, 6.0),
    person(3, 4.0, -12.0),
  };

  // Fly towards the middle of them.
  const FilterSetpoint setpoint = velocitySetpoint(Eigen::Vector3d(3.0, -2.0, 0.0));
  const FilterResult result = run(filter, setpoint, state, targets);

  // Every barrier has to hold for the command that comes out.
  for (const KeepOutTarget & target : targets) {
    expectSatisfiesTrack(filter.params(), target, state, result.velocity);
  }
}

/// A drone commanded straight at a tree stops at >= d_obstacle_min from its surface.
///
/// The check is on the outcome: the barrier limits the closing speed to alpha*h, so
/// far from the tree the drone may still fly towards it. What matters is that it never
/// gets closer than d_obstacle_min to the tree's surface.
TEST(SafetyFilterTest, CommandedAtATreeStopsAtTheObstacleDistance)
{
  SafetyFilter filter(testParams());
  const std::vector<Obstacle> obstacles = {{"tree", 20.0, 0.0, 0.6, 6.5}};

  // The same closed loop as for a person, but against a static obstacle. The drone
  // flies below the top of the tree, so the tree is a hazard for it.
  LagPlant plant;
  plant.state = stateAt(0.0, 0.0, 3.0);

  double minimum = std::numeric_limits<double>::infinity();
  for (double t = 10.0; t < 40.0; t += 0.02) {
    const FilterSetpoint setpoint = velocitySetpoint(Eigen::Vector3d(10.0, 0.0, 0.0));
    const FilterResult result = filter.filter(
      setpoint, plant.state, {}, obstacles, FilterMode::FOLLOW, t, 0.0, 0.0, false, true);
    plant.step(result.velocity, 0.02);
    const double distance = (plant.state.position.head<2>() -
      Eigen::Vector2d(obstacles[0].x, obstacles[0].y)).norm();
    minimum = std::min(minimum, distance);
  }

  // The distance to the tree's centre, and to its surface.
  EXPECT_GE(minimum - obstacles[0].radius, filter.params().d_obstacle_min - 1e-3)
    << "the drone came within " << minimum - obstacles[0].radius
    << " m of the tree's surface";
}

/// A noisy track (large covariance) gets a larger margin than a precise one.
///
/// The margin is inflated with the track's position uncertainty, so a poorly tracked
/// person is kept further away.
TEST(SafetyFilterTest, ANoisyTrackGetsALargerMargin)
{
  KeepOutTarget precise = person(1, 20.0, 0.0);
  precise.position_sigma = 0.0;
  KeepOutTarget noisy = person(1, 20.0, 0.0);
  noisy.position_sigma = 1.0;

  SafetyFilter precise_filter(testParams());
  SafetyFilter noisy_filter(testParams());

  const double precise_margin = closedLoopMinimumMargin(precise_filter, {precise});
  const double noisy_margin = closedLoopMinimumMargin(noisy_filter, {noisy});

  // The noisy track keeps the drone further away, because its margin is bigger.
  EXPECT_GT(noisy_margin, precise_margin);
  // And both hold the distance.
  EXPECT_GE(precise_margin, -1e-3);
  EXPECT_GE(noisy_margin, -1e-3);
}

/// The e-stop latches: the drone holds until it is released.
TEST(SafetyFilterTest, TheEStopLatches)
{
  SafetyFilter filter(testParams());
  const FilterState state = stateAt(0.0, 0.0);
  const FilterSetpoint setpoint = velocitySetpoint(Eigen::Vector3d(10.0, 0.0, 0.0));

  const FilterResult result = filter.filter(
    setpoint, state, {}, {}, FilterMode::FOLLOW, 10.0, 0.0, 0.0, true, true);

  EXPECT_TRUE(result.safety_active);
  EXPECT_EQ(result.velocity.norm(), 0.0);
  EXPECT_FALSE(result.feasible);
}

// ---------------------------------------------------------------------------
// The barrier itself
// ---------------------------------------------------------------------------

/// The drone already inside the keep-out is pushed back out, not just braked.
TEST(SafetyFilterTest, TheDroneInsideTheKeepOutIsPushedBackOut)
{
  SafetyFilter filter(testParams());
  // The person is at 3 m and the drone at 0 m: inside the 5 m keep-out.
  const FilterState state = stateAt(0.0, 0.0);
  const std::vector<KeepOutTarget> targets = {person(1, 3.0, 0.0)};

  const FilterSetpoint setpoint = velocitySetpoint(Eigen::Vector3d(5.0, 0.0, 0.0));
  const FilterResult result = run(filter, setpoint, state, targets);

  // The person is at +x, the drone at 0 m: the barrier pushes it away, so the
  // velocity has to point away from the person, that is towards -x.
  EXPECT_LT(result.velocity.x(), 0.0) << "the drone is still flying at the person";
}

/// The barrier is satisfied by the command that comes out, for a random sweep of
/// geometries.
TEST(SafetyFilterTest, TheBarrierAlwaysHoldsAfterTheFilter)
{
  SafetyFilter filter(testParams());
  std::mt19937 rng(12345);
  std::uniform_real_distribution<double> position(-30.0, 30.0);
  std::uniform_real_distribution<double> velocity(-8.0, 8.0);
  std::uniform_real_distribution<double> distance(1.0, 25.0);
  std::uniform_real_distribution<double> height(0.0, 12.0);

  int violations = 0;
  for (int trial = 0; trial < 2000; ++trial) {
    const FilterState state = stateAt(position(rng), position(rng), height(rng));
    const FilterSetpoint setpoint = velocitySetpoint(
      Eigen::Vector3d(velocity(rng), velocity(rng), 0.0));

    // A person somewhere around the drone, at a random distance and height.
    const double angle = 2.0 * kPi * (static_cast<double>(rng() % 3600) / 3600.0);
    KeepOutTarget target = person(
      1, state.position.x() + distance(rng) * std::cos(angle),
      state.position.y() + distance(rng) * std::sin(angle), height(rng));
    target.velocity = Eigen::Vector3d(velocity(rng), velocity(rng), 0.0);

    const FilterResult result = run(filter, setpoint, state, {target});

    // A hold is always allowed: it is the safe answer when nothing is feasible.
    if (!result.feasible) {
      continue;
    }
    for (const VelocityConstraint & constraint :
      trackConstraints(filter.params(), target, state))
    {
      if (constraint.normal.dot(result.velocity) < constraint.bound - 1e-6) {
        ++violations;
      }
    }
  }
  EXPECT_EQ(violations, 0) << violations << " barrier violations in 2000 random cases";
}

/// A randomized property test: 10,000 random states and commands, and after a
/// simulated step the 3D distance to the person never goes below d_min_3d - 0.05 m.
///
/// The drone flies the command it is given, through a lag, and the person walks with
/// its own velocity. The filter has to hold the distance in spite of the lag: that is
/// what the barrier is for, and it is the case the plan's relative-degree-1 form does
/// not cover.
///
/// The heights are random as well, so the property is checked in the whole 3D
/// geometry and not only in a horizontal plane.
TEST(SafetyFilterTest, RandomizedPropertyTheDistanceNeverDropsBelowTheLimit)
{
  SafetyFilterParams params = testParams();
  SafetyFilter filter(params);

  std::mt19937 rng(2024);
  std::uniform_real_distribution<double> position(-25.0, 25.0);
  std::uniform_real_distribution<double> velocity(-10.0, 10.0);
  std::uniform_real_distribution<double> person_speed(-2.0, 2.0);
  std::uniform_real_distribution<double> dt(0.01, 0.1);
  std::uniform_real_distribution<double> height(0.0, 12.0);

  // The drone moves with the command it gets, the person with its own velocity.
  FilterState state = stateAt(position(rng), position(rng), height(rng));
  KeepOutTarget target = person(1, position(rng), position(rng), height(rng));
  target.velocity = Eigen::Vector3d(person_speed(rng), person_speed(rng), 0.0);

  LagPlant plant;
  plant.state = state;
  plant.tau = 0.34;

  double minimum_distance = std::numeric_limits<double>::infinity();

  for (int step = 0; step < 10000; ++step) {
    const double step_dt = dt(rng);
    const FilterSetpoint setpoint = velocitySetpoint(
      Eigen::Vector3d(velocity(rng), velocity(rng), 0.0));

    const FilterResult result = run(filter, setpoint, plant.state, {target});

    // The drone flies the command it was given, through the lag.
    plant.step(result.velocity, step_dt);
    target.position += target.velocity * step_dt;

    const double distance = (plant.state.position - target.position).norm();
    minimum_distance = std::min(minimum_distance, distance);
  }

  // The filter never lets the drone closer than the limit, with a small tolerance
  // for the integration step.
  EXPECT_GE(minimum_distance, params.d_min_3d - 0.05)
    << "the drone came within " << minimum_distance << " m of the person";
}

/// The same property, from a state where the drone is already closing fast: the
/// barrier has to slow it down before the margin runs out.
TEST(SafetyFilterTest, TheDistanceHoldsWhenTheDroneIsAlreadyClosingFast)
{
  SafetyFilterParams params = testParams();
  SafetyFilter filter(params);

  // The drone is 12 m from the person (7 m of margin) and closing at 6 m/s.
  const std::vector<KeepOutTarget> targets = {person(1, 12.0, 0.0)};

  const double margin = closedLoopMinimumMargin(
    filter, targets, 30.0, 0.02, Eigen::Vector3d(6.0, 0.0, 0.0));

  EXPECT_GE(margin, -1e-3) << "the drone came within " << -margin << " m of the limit";
}

// ---------------------------------------------------------------------------
// The limits and the other conditions
// ---------------------------------------------------------------------------

/// A stale setpoint brakes the drone to a hover.
TEST(SafetyFilterTest, AStaleSetpointBrakesToHover)
{
  SafetyFilter filter(testParams());
  const FilterState state = stateAt(0.0, 0.0);
  const FilterSetpoint setpoint = velocitySetpoint(Eigen::Vector3d(10.0, 0.0, 0.0));

  const FilterResult result = filter.filter(
    setpoint, state, {}, {}, FilterMode::FOLLOW, 10.0, 1.0, 0.0, false, true);

  EXPECT_TRUE(result.safety_active);
  EXPECT_EQ(result.velocity.norm(), 0.0);
  EXPECT_EQ(result.reason, "stale setpoint");
}

/// Stale odometry brakes the drone to a hover.
TEST(SafetyFilterTest, StaleOdometryBrakesToHover)
{
  SafetyFilter filter(testParams());
  const FilterState state = stateAt(0.0, 0.0);
  const FilterSetpoint setpoint = velocitySetpoint(Eigen::Vector3d(10.0, 0.0, 0.0));

  const FilterResult result = filter.filter(
    setpoint, state, {}, {}, FilterMode::FOLLOW, 10.0, 0.0, 1.0, false, true);

  EXPECT_TRUE(result.safety_active);
  EXPECT_EQ(result.velocity.norm(), 0.0);
  EXPECT_EQ(result.reason, "stale odometry");
}

/// The speed cap applies to the planner's request.
TEST(SafetyFilterTest, TheSpeedCapLimitsTheCommand)
{
  SafetyFilter filter(testParams());
  const FilterState state = stateAt(0.0, 0.0);
  const FilterSetpoint setpoint = velocitySetpoint(Eigen::Vector3d(100.0, 0.0, 0.0));

  const FilterResult result = run(filter, setpoint, state, {});

  EXPECT_LE(result.velocity.head<2>().norm(), filter.params().speed_cap_follow + 1e-6);
}

/// The INTERCEPT speed cap is higher than the FOLLOW one.
TEST(SafetyFilterTest, TheSpeedCapDependsOnTheMode)
{
  SafetyFilter filter(testParams());
  const FilterState state = stateAt(0.0, 0.0);
  const FilterSetpoint setpoint = velocitySetpoint(Eigen::Vector3d(100.0, 0.0, 0.0));

  const FilterResult result = run(filter, setpoint, state, {}, {}, FilterMode::INTERCEPT);

  EXPECT_LE(result.velocity.head<2>().norm(), filter.params().speed_cap_intercept + 1e-6);
  EXPECT_GT(filter.speedCap(FilterMode::INTERCEPT), filter.speedCap(FilterMode::FOLLOW));
}

/// The altitude floor stops the drone from descending through it.
TEST(SafetyFilterTest, TheAltitudeFloorStopsTheDescent)
{
  SafetyFilter filter(testParams());
  const FilterState state = stateAt(0.0, 0.0, 1.0);
  const FilterSetpoint setpoint = velocitySetpoint(Eigen::Vector3d(0.0, 0.0, -5.0));

  const FilterResult result = run(filter, setpoint, state, {});

  EXPECT_GE(result.velocity.z(), 0.0);
  EXPECT_TRUE(result.safety_active);
}

/// The geofence stops the drone from flying out of the box.
TEST(SafetyFilterTest, TheGeofenceStopsTheDrone)
{
  SafetyFilter filter(testParams());
  // Near the x_min boundary.
  FilterState state = stateAt(filter.params().geofence_x_min + 0.1, 0.0);
  const FilterSetpoint setpoint = velocitySetpoint(Eigen::Vector3d(-5.0, 0.0, 0.0));

  const FilterResult result = run(filter, setpoint, state, {});

  EXPECT_GE(result.velocity.x(), 0.0);
  EXPECT_TRUE(result.safety_active);
}

/// A position setpoint is turned into a velocity, so the filter can act on it.
TEST(SafetyFilterTest, APositionSetpointIsTurnedIntoAVelocity)
{
  SafetyFilter filter(testParams());
  const FilterState state = stateAt(0.0, 0.0);
  // The setpoint is right on top of a person.
  const std::vector<KeepOutTarget> targets = {person(1, 2.0, 0.0)};
  const FilterSetpoint setpoint = positionSetpoint(Eigen::Vector3d(2.0, 0.0, 1.5));

  const FilterResult result = run(filter, setpoint, state, targets);

  // The filter pushes the drone away from the person, not towards it.
  EXPECT_LT(result.velocity.x(), 0.0);
}

/// A uav is never a keep-out target.
TEST(SafetyFilterTest, AUavIsNotAKeepOutTarget)
{
  SafetyFilter filter(testParams());
  const FilterState state = stateAt(0.0, 0.0);
  KeepOutTarget uav = person(1, 3.0, 0.0);
  uav.class_id = kUav;

  const FilterSetpoint setpoint = velocitySetpoint(Eigen::Vector3d(10.0, 0.0, 0.0));
  const FilterResult result = run(filter, setpoint, state, {uav});

  // The drone is free to fly at the uav: the command is not modified.
  EXPECT_FALSE(result.safety_active);
  EXPECT_GT(result.velocity.x(), 0.0);
}

/// A track that disappears keeps its keep-out for coast_timeout.
///
/// The person is 8 m away, so the margin is 3 m and the barrier allows a closing speed
/// of alpha*h = 3 m/s. The keep-out is still enforced after the track disappears: the
/// closing speed stays limited to the same value.
TEST(SafetyFilterTest, AStaleTrackKeepsItsKeepOut)
{
  SafetyFilter filter(testParams());
  const FilterState state = stateAt(0.0, 0.0);
  const std::vector<KeepOutTarget> targets = {person(1, 8.0, 0.0)};
  const FilterSetpoint setpoint = velocitySetpoint(Eigen::Vector3d(10.0, 0.0, 0.0));
  const double allowed =
    filter.params().cbf_alpha * (8.0 - filter.params().d_min_horizontal);

  // The track is reported once...
  const FilterResult first = run(filter, setpoint, state, targets);
  EXPECT_TRUE(first.safety_active);
  EXPECT_LE(first.velocity.x(), allowed + 1e-6);

  // ...and then it is gone: the keep-out has to stay.
  const FilterResult second = filter.filter(
    setpoint, state, {}, {}, FilterMode::FOLLOW, 10.5, 0.0, 0.0, false, true);
  EXPECT_TRUE(second.safety_active);
  EXPECT_LE(second.velocity.x(), allowed + 1e-6)
    << "the keep-out was forgotten too early";
}

/// After coast_timeout the keep-out is forgotten, and the filter says so.
TEST(SafetyFilterTest, AKeepOutIsForgottenAfterTheCoastTimeout)
{
  SafetyFilter filter(testParams());
  const FilterState state = stateAt(0.0, 0.0);
  const std::vector<KeepOutTarget> targets = {person(1, 8.0, 0.0)};
  const FilterSetpoint setpoint = velocitySetpoint(Eigen::Vector3d(10.0, 0.0, 0.0));

  run(filter, setpoint, state, targets);

  // The track is gone for longer than coast_timeout: the filter holds and says so.
  const FilterResult result = filter.filter(
    setpoint, state, {}, {}, FilterMode::FOLLOW,
    10.0 + filter.params().coast_timeout + 0.1, 0.0, 0.0, false, true);

  EXPECT_TRUE(result.safety_active);
  EXPECT_EQ(result.reason, "keep-out forgotten (track lost)");
  EXPECT_EQ(result.velocity.norm(), 0.0);
}

/// A boxed-in drone holds instead of choosing one distance to relax.
TEST(SafetyFilterTest, ABoxedInDroneHoldsInsteadOfRelaxingADistance)
{
  SafetyFilter filter(testParams());
  const FilterState state = stateAt(0.0, 0.0);
  // Two people on opposite sides: the drone cannot move without breaking one of
  // the two barriers.
  const std::vector<KeepOutTarget> targets = {
    person(1, 3.0, 0.0),
    person(2, -3.0, 0.0),
  };

  const FilterSetpoint setpoint = velocitySetpoint(Eigen::Vector3d(10.0, 0.0, 0.0));
  const FilterResult result = run(filter, setpoint, state, targets);

  // Either it holds, or the command it returns satisfies every barrier.
  if (result.feasible) {
    for (const KeepOutTarget & target : targets) {
      expectSatisfiesTrack(filter.params(), target, state, result.velocity);
    }
  } else {
    EXPECT_EQ(result.velocity.norm(), 0.0);
  }
}

/// The engagement gate refuses an INTERCEPT command when it is closed.
TEST(SafetyFilterTest, TheClosedEngagementGateHoldsTheDrone)
{
  SafetyFilter filter(testParams());
  const FilterState state = stateAt(0.0, 0.0);
  const FilterSetpoint setpoint = velocitySetpoint(Eigen::Vector3d(10.0, 0.0, 0.0));

  const FilterResult result = filter.filter(
    setpoint, state, {}, {}, FilterMode::INTERCEPT, 10.0, 0.0, 0.0, false, false);

  EXPECT_TRUE(result.safety_active);
  EXPECT_EQ(result.velocity.norm(), 0.0);
  EXPECT_EQ(result.reason, "engagement gate closed");
}

/// The vertical keep-out stops the drone from descending onto a person.
TEST(SafetyFilterTest, TheVerticalKeepOutStopsTheDescent)
{
  SafetyFilter filter(testParams());
  // The drone is 3 m above the person and 2 m away horizontally, inside 2 * d_min.
  const FilterState state = stateAt(0.0, 0.0, 3.0);
  const std::vector<KeepOutTarget> targets = {person(1, 2.0, 0.0)};

  const FilterSetpoint setpoint = velocitySetpoint(Eigen::Vector3d(0.0, 0.0, -3.0));
  const FilterResult result = run(filter, setpoint, state, targets);

  EXPECT_GE(result.velocity.z(), 0.0);
  EXPECT_TRUE(result.safety_active);
}

/// The keep-out wins over the speed cap: the distance is never traded away.
TEST(SafetyFilterTest, TheKeepOutWinsOverTheSpeedCap)
{
  SafetyFilterParams params = testParams();
  // A slow mode, so the cap really bites.
  params.speed_cap_follow = 1.0;
  SafetyFilter filter(params);

  const FilterState state = stateAt(0.0, 0.0);
  const std::vector<KeepOutTarget> targets = {person(1, 8.0, 0.0)};
  const FilterSetpoint setpoint = velocitySetpoint(Eigen::Vector3d(10.0, 0.0, 0.0));

  const FilterResult result = run(filter, setpoint, state, targets);

  // The command has to satisfy the barrier, even if that means going faster than
  // the cap: the keep-out is the top-level rule.
  expectSatisfiesTrack(filter.params(), targets[0], state, result.velocity);
}

// ---------------------------------------------------------------------------
// The keep-out of a person: 3D, with an altitude relief (P3.4)
// ---------------------------------------------------------------------------

/// A person 12 m below the drone does not push the interceptor sideways.
///
/// This is the problem the intercept guidance ran into: the plan's horizontal
/// keep-out has no altitude exemption, so an INTERCEPT over a park with
/// pedestrians was impossible. The 3D rule is the safety distance, and a person
/// 12 m below the drone is 12 m away, so the horizontal rule is given up.
TEST(SafetyFilterTest, APersonFarBelowTheDroneDoesNotPushItSideways)
{
  SafetyFilter filter(testParams());
  // The drone is 12 m above the person, right above it.
  const FilterState state = stateAt(0.0, 0.0, 12.0);
  const std::vector<KeepOutTarget> targets = {person(1, 0.0, 0.0)};

  const FilterSetpoint setpoint = velocitySetpoint(Eigen::Vector3d(10.0, 0.0, 0.0));
  const FilterResult result = run(filter, setpoint, state, targets);

  // The command is not modified: the drone is 12 m away from the person.
  EXPECT_FALSE(result.safety_active) << "the drone was pushed away from a person 12 m below";
  EXPECT_GT(result.velocity.x(), 0.0);
}

/// The horizontal standoff is in force while the drone is at the person's level.
TEST(SafetyFilterTest, TheHorizontalStandoffIsInForceAtThePersonsLevel)
{
  SafetyFilter filter(testParams());
  const FilterState state = stateAt(0.0, 0.0, 1.5);
  const std::vector<KeepOutTarget> targets = {person(1, 6.0, 0.0)};

  const FilterSetpoint setpoint = velocitySetpoint(Eigen::Vector3d(10.0, 0.0, 0.0));
  const FilterResult result = run(filter, setpoint, state, targets);

  // The person is 6 m away, so the margin is 1 m and the closing speed is
  // limited to alpha * h = 1 m/s.
  const double allowed = filter.params().cbf_alpha * (6.0 - filter.params().d_min_horizontal);
  EXPECT_TRUE(result.safety_active);
  EXPECT_LE(result.velocity.x(), allowed + 1e-6);
}

/// The drone never descends onto a person: the 3D keep-out stops it at the
/// safety distance, even when the horizontal standoff has been given up.
TEST(SafetyFilterTest, TheDroneNeverDescendsOntoAPerson)
{
  SafetyFilterParams params = testParams();
  SafetyFilter filter(params);

  // The drone hovers 12 m above a person and is commanded straight down at it.
  LagPlant plant;
  plant.state = stateAt(0.0, 0.0, 12.0);
  const std::vector<KeepOutTarget> targets = {person(1, 0.0, 0.0)};

  double minimum = std::numeric_limits<double>::infinity();
  for (double t = 10.0; t < 40.0; t += 0.02) {
    const FilterSetpoint setpoint = velocitySetpoint(Eigen::Vector3d(0.0, 0.0, -5.0));
    const FilterResult result = filter.filter(
      setpoint, plant.state, targets, {}, FilterMode::FOLLOW, t, 0.0, 0.0, false, true);
    plant.step(result.velocity, 0.02);
    minimum = std::min(minimum, plant.state.position.z());
  }

  // The distance to the person is the height: it may never drop below the
  // safety distance.
  EXPECT_GE(minimum, params.d_min_3d - 0.05)
    << "the drone descended to " << minimum << " m above the person";
}

/// An abort brakes the drone and says why.
TEST(SafetyFilterTest, AnAbortBrakesTheDroneToAHover)
{
  SafetyFilter filter(testParams());
  const FilterState state = stateAt(0.0, 0.0);
  const FilterSetpoint setpoint = velocitySetpoint(Eigen::Vector3d(10.0, 0.0, 0.0));

  const FilterResult result = filter.filter(
    setpoint, state, {}, {}, FilterMode::INTERCEPT, 10.0, 0.0, 0.0, false, true,
    "a person is 8 m from the capture point");

  EXPECT_TRUE(result.aborted);
  EXPECT_TRUE(result.safety_active);
  EXPECT_EQ(result.velocity.norm(), 0.0) << "the abort did not brake the drone";
  EXPECT_NE(result.reason.find("a person is 8 m"), std::string::npos)
    << "the reason does not say why the engagement was aborted: " << result.reason;
}

}  // namespace
}  // namespace safety
}  // namespace interceptor
