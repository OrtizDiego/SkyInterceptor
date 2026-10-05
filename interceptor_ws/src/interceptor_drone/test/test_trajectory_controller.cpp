#include <gtest/gtest.h>
#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <limits>

#include "control/trajectory_controller.hpp"
#include "flight/flight_controller.hpp"
#include "flight/quadrotor_model.hpp"
#include "flight/wind_model.hpp"

namespace interceptor
{
namespace control
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kPeriod = 0.02;  // 50 Hz, like the node

ControllerSetpoint positionSetpoint(const Eigen::Vector3d & position, uint8_t source = 1)
{
  ControllerSetpoint sp;
  sp.position = position;
  sp.position_valid = true;
  sp.source = source;
  return sp;
}

// Simple velocity plant: first-order lag from the commanded to the actual world velocity,
// plus an optional constant disturbance (wind drift) added to the velocity.
struct LagPlant
{
  ControllerState state;
  double tau = 0.4;  // s
  Eigen::Vector3d disturbance = Eigen::Vector3d::Zero();

  void step(const ControllerCommand & cmd, double dt)
  {
    const Eigen::Vector3d target = cmd.velocity + disturbance;
    state.velocity += (target - state.velocity) * (1.0 - std::exp(-dt / tau));
    state.position += state.velocity * dt;
    state.yaw = std::remainder(state.yaw + cmd.yaw_rate * dt, 2.0 * kPi);
  }
};

struct LoopResult
{
  double max_overshoot = 0.0;  // beyond the target along the travel direction [m]
  double settling_time = std::numeric_limits<double>::infinity();  // to within 0.1 m [s]
  Eigen::Vector3d max_integral = Eigen::Vector3d::Zero();
};

// Runs the controller against the lag plant for `seconds`, refreshing the setpoint each tick.
LoopResult runLag(
  TrajectoryController & controller, LagPlant & plant, const ControllerSetpoint & sp,
  double seconds, double start_time = 0.0)
{
  LoopResult result;
  const Eigen::Vector3d start = plant.state.position;
  const Eigen::Vector3d direction = (sp.position - start).normalized();
  const int steps = static_cast<int>(std::round(seconds / kPeriod));
  double last_outside = 0.0;
  for (int k = 0; k < steps; ++k) {
    const double t = start_time + k * kPeriod;
    controller.setSetpoint(sp, t);
    plant.step(controller.update(plant.state, t), kPeriod);
    const double along = (plant.state.position - sp.position).dot(direction);
    result.max_overshoot = std::max(result.max_overshoot, along);
    if ((plant.state.position - sp.position).norm() > 0.1) {
      last_outside = (k + 1) * kPeriod;
    }
    result.max_integral = result.max_integral.cwiseMax(controller.integral().cwiseAbs());
  }
  result.settling_time = last_outside;
  return result;
}

// --- Position loop --------------------------------------------------------------------

TEST(TrajectoryController, StepResponseSettlesWithoutOvershoot)
{
  TrajectoryController controller;
  LagPlant plant;
  const LoopResult r = runLag(controller, plant, positionSetpoint({10.0, -5.0, 3.0}), 30.0);
  EXPECT_LT(r.settling_time, 15.0);
  // The step is large enough to hit no limit; overshoot stays below 2 % of the 11.6 m step
  EXPECT_LT(r.max_overshoot, 0.02 * Eigen::Vector3d(10.0, -5.0, 3.0).norm());
  EXPECT_LT((plant.state.position - Eigen::Vector3d(10.0, -5.0, 3.0)).norm(), 0.02);
}

TEST(TrajectoryController, ProportionalGainSetsTheCommand)
{
  TrajectoryControllerParams p;
  p.ki_pos = 0.0;
  p.kd_pos = 0.0;
  p.kp_pos = 0.8;
  TrajectoryController controller(p);
  controller.setSetpoint(positionSetpoint({5.0, 0.0, -2.0}), 0.0);
  const ControllerCommand cmd = controller.update(ControllerState(), 0.0);
  EXPECT_FALSE(cmd.timed_out);
  EXPECT_NEAR(cmd.velocity.x(), 4.0, 1e-9);
  EXPECT_NEAR(cmd.velocity.z(), -1.6, 1e-9);
}

TEST(TrajectoryController, VelocityFeedForwardRemovesTrackingLag)
{
  // Target moving at 3 m/s along x; the setpoint carries its velocity.
  TrajectoryControllerParams p;
  p.ki_pos = 0.0;  // prove the feed-forward alone does it, not the integrator
  TrajectoryController controller(p);
  LagPlant plant;
  const double speed = 3.0;
  for (int k = 0; k < 1500; ++k) {
    const double t = k * kPeriod;
    ControllerSetpoint sp = positionSetpoint({speed * t, 0.0, 0.0});
    sp.velocity = Eigen::Vector3d(speed, 0.0, 0.0);
    controller.setSetpoint(sp, t);
    plant.step(controller.update(plant.state, t), kPeriod);
  }
  const double t_end = 1500 * kPeriod;
  EXPECT_NEAR(plant.state.position.x(), speed * t_end, 0.1);
  EXPECT_NEAR(plant.state.velocity.x(), speed, 0.05);
}

TEST(TrajectoryController, IntegratorRemovesSteadyStateErrorFromConstantDisturbance)
{
  const Eigen::Vector3d target(0.0, 0.0, 5.0);
  const Eigen::Vector3d drift(0.6, -0.3, 0.0);  // m/s, e.g. wind pushing the velocity

  TrajectoryControllerParams without = {};
  without.ki_pos = 0.0;
  TrajectoryController p_only(without);
  LagPlant plant_p;
  plant_p.disturbance = drift;
  plant_p.state.position = target;
  runLag(p_only, plant_p, positionSetpoint(target), 60.0);
  // Proportional control alone leaves an offset of about drift / kp
  EXPECT_GT((plant_p.state.position - target).norm(), 0.5);

  TrajectoryController pid;
  LagPlant plant;
  plant.disturbance = drift;
  plant.state.position = target;
  runLag(pid, plant, positionSetpoint(target), 200.0);
  EXPECT_LT((plant.state.position - target).norm(), 0.05);
  // The integral term carries the opposite of the disturbance
  EXPECT_NEAR(pid.integral().x(), -drift.x(), 0.05);
}

// --- Output limits and anti-windup ---------------------------------------------------

TEST(TrajectoryController, SaturationKeepsTheDirectionAndRespectsTheLimits)
{
  TrajectoryControllerParams p;
  p.ki_pos = 0.0;
  p.kd_pos = 0.0;
  p.max_horizontal_velocity = 6.0;
  p.max_vertical_velocity = 2.0;
  TrajectoryController controller(p);
  controller.setSetpoint(positionSetpoint({300.0, 400.0, 100.0}), 0.0);
  const ControllerCommand cmd = controller.update(ControllerState(), 0.0);
  EXPECT_NEAR(cmd.velocity.head<2>().norm(), 6.0, 1e-9);
  EXPECT_NEAR(cmd.velocity.y() / cmd.velocity.x(), 4.0 / 3.0, 1e-9);
  EXPECT_NEAR(cmd.velocity.z(), 2.0, 1e-9);

  controller.setSetpoint(positionSetpoint({0.0, 0.0, -100.0}), 0.1);
  EXPECT_NEAR(controller.update(ControllerState(), 0.1).velocity.z(), -2.0, 1e-9);
}

TEST(TrajectoryController, IntegratorStaysBoundedWhenTheOutputSaturates)
{
  TrajectoryControllerParams p;
  p.ki_pos = 0.5;  // aggressive, so that winding up would be obvious
  p.integral_zone = 1e9;  // no conditional integration: back-calculation alone must do it
  p.max_horizontal_velocity = 3.0;
  TrajectoryController controller(p);
  // The vehicle is stuck far away from the target for a minute
  const ControllerState stuck;
  const ControllerSetpoint sp = positionSetpoint({200.0, 0.0, 0.0});
  double max_integral = 0.0;
  for (int k = 0; k < 3000; ++k) {
    controller.setSetpoint(sp, k * kPeriod);
    const ControllerCommand cmd = controller.update(stuck, k * kPeriod);
    EXPECT_LE(cmd.velocity.head<2>().norm(), 3.0 + 1e-9);
    max_integral = std::max(max_integral, controller.integral().norm());
  }
  EXPECT_LE(max_integral, p.max_integral_velocity + 1e-9);
  // Back-calculation never winds up further because of saturation: with the target
  // far outside the limit the integrator sits near zero, not at its clamp.
  EXPECT_LT(controller.integral().norm(), 0.1);
}

TEST(TrajectoryController, IntegratorIsClampedWithoutSaturation)
{
  TrajectoryControllerParams p;
  p.max_integral_velocity = 0.4;
  p.ki_pos = 1.0;
  p.kp_pos = 0.01;  // small proportional term: never saturates, error persists
  p.integral_zone = 1e9;
  TrajectoryController controller(p);
  const ControllerState stuck;
  const ControllerSetpoint sp = positionSetpoint({0.0, 10.0, 0.0});
  for (int k = 0; k < 2000; ++k) {
    controller.setSetpoint(sp, k * kPeriod);
    controller.update(stuck, k * kPeriod);
  }
  EXPECT_NEAR(controller.integral().y(), 0.4, 1e-9);
  EXPECT_NEAR(controller.integral().head<2>().norm(), 0.4, 1e-9);
}

TEST(TrajectoryController, IntegratorOnlyAccumulatesCloseToTheSetpoint)
{
  TrajectoryController controller;  // integral_zone 2 m
  const ControllerState state;
  for (int k = 0; k < 500; ++k) {
    controller.setSetpoint(positionSetpoint({5.0, 0.0, 0.0}), k * kPeriod);  // 5 m away
    controller.update(state, k * kPeriod);
  }
  EXPECT_TRUE(controller.integral().isZero());
  for (int k = 500; k < 1000; ++k) {
    controller.setSetpoint(positionSetpoint({1.0, 0.0, 0.0}), k * kPeriod);  // 1 m away
    controller.update(state, k * kPeriod);
  }
  EXPECT_GT(controller.integral().x(), 0.1);
}

TEST(TrajectoryController, BackCalculationAloneLimitsOvershootAfterASaturatedApproach)
{
  TrajectoryControllerParams p;
  p.max_horizontal_velocity = 10.0;
  p.integral_zone = 1e9;  // integrates all the way: only back-calculation protects
  TrajectoryController controller(p);
  LagPlant plant;
  const LoopResult r = runLag(controller, plant, positionSetpoint({150.0, 0.0, 0.0}), 120.0);
  EXPECT_LT(r.max_overshoot, 3.0);
  EXPECT_LT((plant.state.position - Eigen::Vector3d(150.0, 0.0, 0.0)).norm(), 0.2);
}

TEST(TrajectoryController, NoWindupOvershootAfterALongSaturatedApproach)
{
  // A 150 m trip saturates the output for a long time. With a wound-up integrator the
  // vehicle would shoot past the target; here the overshoot stays small.
  TrajectoryControllerParams p;
  p.max_horizontal_velocity = 10.0;
  TrajectoryController controller(p);
  LagPlant plant;
  const LoopResult r = runLag(controller, plant, positionSetpoint({150.0, 0.0, 0.0}), 90.0);
  EXPECT_LT(r.max_overshoot, 1.0);
  EXPECT_LT((plant.state.position - Eigen::Vector3d(150.0, 0.0, 0.0)).norm(), 0.1);
}

// --- Velocity-only setpoints -------------------------------------------------------

TEST(TrajectoryController, VelocityOnlySetpointIsTrackedDirectly)
{
  TrajectoryController controller;
  ControllerSetpoint sp;
  sp.velocity = Eigen::Vector3d(2.0, -1.0, 0.5);
  sp.position = Eigen::Vector3d(1000.0, 1000.0, 1000.0);  // ignored: position_valid is false
  sp.position_valid = false;
  controller.setSetpoint(sp, 0.0);
  ControllerState state;
  state.position = Eigen::Vector3d(5.0, 5.0, 5.0);
  const ControllerCommand cmd = controller.update(state, 0.0);
  EXPECT_TRUE(cmd.velocity.isApprox(sp.velocity, 1e-12));
  EXPECT_TRUE(controller.integral().isZero());
}

TEST(TrajectoryController, VelocityOnlySetpointIsCapped)
{
  TrajectoryControllerParams p;
  p.max_horizontal_velocity = 10.0;
  TrajectoryController controller(p);
  ControllerSetpoint sp;
  sp.velocity = Eigen::Vector3d(30.0, 40.0, 0.0);
  controller.setSetpoint(sp, 0.0);
  EXPECT_NEAR(controller.update(ControllerState(), 0.0).velocity.head<2>().norm(), 10.0, 1e-9);
}

// --- Timeout -----------------------------------------------------------------------

TEST(TrajectoryController, CommandsZeroBeforeAnySetpoint)
{
  TrajectoryController controller;
  const ControllerCommand cmd = controller.update(ControllerState(), 1.0);
  EXPECT_TRUE(cmd.timed_out);
  EXPECT_TRUE(cmd.velocity.isZero());
  EXPECT_DOUBLE_EQ(cmd.yaw_rate, 0.0);
}

TEST(TrajectoryController, StaleSetpointCommandsZeroAfterTheTimeout)
{
  TrajectoryController controller;  // timeout 0.3 s
  ControllerSetpoint sp = positionSetpoint({10.0, 0.0, 0.0});
  sp.yaw = 1.0;
  controller.setSetpoint(sp, 5.0);

  ControllerCommand cmd = controller.update(ControllerState(), 5.2);
  EXPECT_FALSE(cmd.timed_out);
  EXPECT_GT(cmd.velocity.x(), 1.0);
  EXPECT_GT(cmd.yaw_rate, 0.0);

  cmd = controller.update(ControllerState(), 5.31);
  EXPECT_TRUE(cmd.timed_out);
  EXPECT_TRUE(cmd.velocity.isZero());
  EXPECT_DOUBLE_EQ(cmd.yaw_rate, 0.0);

  // A fresh setpoint resumes control
  controller.setSetpoint(sp, 5.4);
  EXPECT_FALSE(controller.update(ControllerState(), 5.4).timed_out);
}

TEST(TrajectoryController, TimeoutClearsTheIntegrator)
{
  TrajectoryController controller;
  const ControllerState state;
  for (int k = 0; k < 100; ++k) {
    controller.setSetpoint(positionSetpoint({0.0, 1.0, 0.0}), k * kPeriod);
    controller.update(state, k * kPeriod);
  }
  EXPECT_GT(controller.integral().norm(), 0.0);
  controller.update(state, 100 * kPeriod + 1.0);  // setpoint went stale
  EXPECT_TRUE(controller.integral().isZero());
}

TEST(TrajectoryController, BackwardClockJumpForgetsTheSetpoint)
{
  TrajectoryController controller;
  controller.setSetpoint(positionSetpoint({10.0, 0.0, 0.0}), 100.0);
  EXPECT_FALSE(controller.update(ControllerState(), 100.1).timed_out);
  // Simulation restarted: time jumps back, the old setpoint must not be reused
  EXPECT_TRUE(controller.update(ControllerState(), 2.0).timed_out);
  EXPECT_FALSE(controller.hasSetpoint());
}

// --- Resets -----------------------------------------------------------------------

TEST(TrajectoryController, ResetsTheIntegratorWhenTheSourceChanges)
{
  TrajectoryController controller;
  const ControllerState state;
  for (int k = 0; k < 100; ++k) {
    controller.setSetpoint(positionSetpoint({0.0, 1.0, 0.0}, 1), k * kPeriod);
    controller.update(state, k * kPeriod);
  }
  ASSERT_GT(controller.integral().norm(), 0.0);

  controller.setSetpoint(positionSetpoint({0.0, 1.0, 0.0}, 2), 100 * kPeriod);  // INTERCEPT
  EXPECT_TRUE(controller.integral().isZero());
}

TEST(TrajectoryController, ResetsTheIntegratorWhenPositionValidityChanges)
{
  TrajectoryController controller;
  const ControllerState state;
  for (int k = 0; k < 100; ++k) {
    controller.setSetpoint(positionSetpoint({0.0, 1.0, 0.0}), k * kPeriod);
    controller.update(state, k * kPeriod);
  }
  ASSERT_GT(controller.integral().norm(), 0.0);

  ControllerSetpoint velocity_only;
  velocity_only.source = 1;
  controller.setSetpoint(velocity_only, 100 * kPeriod);
  EXPECT_TRUE(controller.integral().isZero());
}

TEST(TrajectoryController, SameSourceKeepsTheIntegrator)
{
  TrajectoryController controller;
  const ControllerState state;
  for (int k = 0; k < 100; ++k) {
    controller.setSetpoint(positionSetpoint({0.0, 1.0, 0.0}), k * kPeriod);
    controller.update(state, k * kPeriod);
  }
  const Eigen::Vector3d before = controller.integral();
  controller.setSetpoint(positionSetpoint({0.0, 1.5, 0.0}), 100 * kPeriod);
  EXPECT_TRUE(controller.integral().isApprox(before, 1e-12));
}

TEST(TrajectoryController, ExplicitResetsClear)
{
  TrajectoryController controller;
  const ControllerState state;
  for (int k = 0; k < 100; ++k) {
    controller.setSetpoint(positionSetpoint({0.0, 1.0, 0.0}), k * kPeriod);
    controller.update(state, k * kPeriod);
  }
  controller.resetIntegrator();
  EXPECT_TRUE(controller.integral().isZero());
  EXPECT_TRUE(controller.hasSetpoint());
  controller.reset();
  EXPECT_FALSE(controller.hasSetpoint());
  EXPECT_TRUE(controller.update(state, 3.0).timed_out);
  EXPECT_TRUE(controller.hold().velocity.isZero());
}

TEST(TrajectoryController, RejectsNonFiniteSetpoints)
{
  TrajectoryController controller;
  ControllerSetpoint good = positionSetpoint({1.0, 0.0, 0.0});
  ASSERT_TRUE(controller.setSetpoint(good, 0.0));

  ControllerSetpoint bad = good;
  bad.position.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(controller.setSetpoint(bad, 0.1));
  bad = good;
  bad.velocity.y() = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(controller.setSetpoint(bad, 0.1));
  bad = good;
  bad.yaw = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(controller.setSetpoint(bad, 0.1));

  // The previous setpoint stays in use (and times out on its own)
  const ControllerCommand cmd = controller.update(ControllerState(), 0.2);
  EXPECT_FALSE(cmd.timed_out);
  EXPECT_TRUE(cmd.velocity.allFinite());
  EXPECT_TRUE(controller.update(ControllerState(), 0.5).timed_out);
}

// --- Yaw ---------------------------------------------------------------------------

TEST(TrajectoryController, YawProportionalToTheError)
{
  TrajectoryControllerParams p;
  p.kp_yaw = 1.0;
  p.max_yaw_rate = 5.0;
  TrajectoryController controller(p);
  ControllerSetpoint sp = positionSetpoint({0.0, 0.0, 0.0});
  sp.yaw = 0.8;
  controller.setSetpoint(sp, 0.0);
  ControllerState state;
  state.yaw = 0.3;
  EXPECT_NEAR(controller.update(state, 0.0).yaw_rate, 0.5, 1e-9);
}

TEST(TrajectoryController, YawRateIsLimited)
{
  TrajectoryController controller;  // kp_yaw 1.5, limit 1.5 rad/s
  ControllerSetpoint sp = positionSetpoint({0.0, 0.0, 0.0});
  sp.yaw = 2.0;
  controller.setSetpoint(sp, 0.0);
  EXPECT_NEAR(controller.update(ControllerState(), 0.0).yaw_rate, 1.5, 1e-12);
  sp.yaw = -2.0;
  controller.setSetpoint(sp, 0.1);
  EXPECT_NEAR(controller.update(ControllerState(), 0.1).yaw_rate, -1.5, 1e-12);
}

TEST(TrajectoryController, YawTakesTheShortWayAroundPi)
{
  TrajectoryController controller;
  ControllerSetpoint sp = positionSetpoint({0.0, 0.0, 0.0});
  sp.yaw = -kPi + 0.1;
  controller.setSetpoint(sp, 0.0);
  ControllerState state;
  state.yaw = kPi - 0.1;  // 0.2 rad away through +-pi, not 6.08 rad the other way
  const ControllerCommand cmd = controller.update(state, 0.0);
  EXPECT_NEAR(cmd.yaw_rate, 1.5 * 0.2, 1e-9);  // turning positive

  sp.yaw = kPi - 0.1;
  state.yaw = -kPi + 0.1;
  controller.setSetpoint(sp, 0.1);
  EXPECT_NEAR(controller.update(state, 0.1).yaw_rate, -1.5 * 0.2, 1e-9);
}

TEST(TrajectoryController, YawRateFeedForwardIsAddedAndStillLimited)
{
  TrajectoryController controller;
  ControllerSetpoint sp = positionSetpoint({0.0, 0.0, 0.0});
  sp.yaw = 0.0;
  sp.yaw_rate = 0.4;
  controller.setSetpoint(sp, 0.0);
  EXPECT_NEAR(controller.update(ControllerState(), 0.0).yaw_rate, 0.4, 1e-12);

  sp.yaw_rate = 3.0;
  controller.setSetpoint(sp, 0.1);
  EXPECT_NEAR(controller.update(ControllerState(), 0.1).yaw_rate, 1.5, 1e-12);
}

TEST(TrajectoryController, YawConvergesWithoutOvershootAcrossPi)
{
  TrajectoryController controller;
  LagPlant plant;
  plant.state.yaw = 3.0;
  ControllerSetpoint sp = positionSetpoint({0.0, 0.0, 0.0});
  sp.yaw = -3.0;  // 0.283 rad through pi
  double worst = 0.0;
  for (int k = 0; k < 500; ++k) {
    controller.setSetpoint(sp, k * kPeriod);
    plant.step(controller.update(plant.state, k * kPeriod), kPeriod);
    worst = std::max(worst, std::abs(std::remainder(plant.state.yaw - 3.0, 2.0 * kPi)));
  }
  EXPECT_NEAR(std::remainder(plant.state.yaw - sp.yaw, 2.0 * kPi), 0.0, 1e-3);
  EXPECT_LT(worst, 0.30);  // never swung the long way round
}

// --- Closed loop on the 6-DOF drone model -------------------------------------------

// The real plant of this project: flight controller + rotors + aerodynamics, 1 kHz,
// with the trajectory controller at 50 Hz commanding world-frame velocities.
struct DroneLoop
{
  static constexpr double kDt = 0.001;

  flight::QuadrotorParams params = flight::makeX500Params();
  flight::QuadrotorModel model{params};
  flight::FlightController autopilot{params};
  flight::WindModel wind;
  flight::RigidBodyState state;
  TrajectoryController controller;
  flight::VelocityCommand command;
  double time = 0.0;
  double max_error_after_settling = 0.0;  // distance to the target after settle_time [m]
  double max_overshoot = 0.0;             // beyond the target along the trip [m]

  DroneLoop()
  {
    state.position = Eigen::Vector3d(0.0, 0.0, 5.0);
    const double w = std::sqrt(
      params.mass * flight::kGravity / (flight::kNumRotors * params.thrust_coefficient));
    model.setRotorSpeeds(flight::RotorArray{w, w, w, w});
    autopilot.reset(state);
    flight::VelocityCommand climb;
    climb.velocity.z() = 0.2;
    autopilot.update(state, climb, kDt);  // leaves the landed state: already flying
  }

  // Flies for `seconds`. The setpoint is delivered at 50 Hz unless `sp` is null.
  void run(
    double seconds, const ControllerSetpoint * sp, const Eigen::Vector3d & target,
    double settle_time = 0.0)
  {
    const int steps = static_cast<int>(std::round(seconds / kDt));
    const Eigen::Vector3d direction = (target - state.position).normalized();
    for (int k = 0; k < steps; ++k) {
      if (k % 20 == 0) {
        ControllerState s;
        s.position = state.position;
        s.velocity = state.velocity;
        s.yaw = flight::yawFromQuaternion(state.orientation);
        if (sp != nullptr) {
          controller.setSetpoint(*sp, time);
        }
        const ControllerCommand c = controller.update(s, time);
        command.velocity = c.velocity;
        command.yaw_rate = c.yaw_rate;
      }
      model.setRotorSpeedCommands(autopilot.update(state, command, kDt));
      model.updateMotors(kDt);
      const flight::Wrench wrench = model.computeWrench(state, wind.update(kDt));
      flight::integrateRigidBody(state, wrench, params.mass, params.inertia, kDt);
      time += kDt;
      max_overshoot = std::max(max_overshoot, (state.position - target).dot(direction));
      if (time > settle_time) {
        max_error_after_settling =
          std::max(max_error_after_settling, (state.position - target).norm());
      }
    }
  }
};

TEST(TrajectoryControllerClosedLoop, FlyingToAFixedPointAndHoldingIt)
{
  DroneLoop loop;
  const Eigen::Vector3d start = loop.state.position;
  const Eigen::Vector3d target(12.0, -8.0, 9.0);
  const ControllerSetpoint sp = positionSetpoint(target);
  loop.run(45.0, &sp, target, 25.0);
  // Reaches the point and holds it within 0.2 m (the P2.1 acceptance criterion)
  EXPECT_LT(loop.max_error_after_settling, 0.2);
  EXPECT_LT((loop.state.position - target).norm(), 0.1);
  // Overshoot stays below 5 % of the trip
  EXPECT_LT(loop.max_overshoot, 0.05 * (target - start).norm());
  EXPECT_LT(loop.state.velocity.norm(), 0.1);
}

TEST(TrajectoryControllerClosedLoop, HoldsThePointInSteadyWind)
{
  DroneLoop loop;
  loop.wind.setMean(Eigen::Vector3d(3.0, -2.0, 0.0));
  const Eigen::Vector3d target(0.0, 0.0, 8.0);
  const ControllerSetpoint sp = positionSetpoint(target);
  loop.run(60.0, &sp, target, 40.0);
  EXPECT_LT(loop.max_error_after_settling, 0.2);
}

TEST(TrajectoryControllerClosedLoop, TurnsToTheCommandedHeading)
{
  DroneLoop loop;
  ControllerSetpoint sp = positionSetpoint({0.0, 0.0, 5.0});
  sp.yaw = 2.0;
  loop.run(10.0, &sp, Eigen::Vector3d(0.0, 0.0, 5.0));
  EXPECT_NEAR(
    std::remainder(flight::yawFromQuaternion(loop.state.orientation) - 2.0, 2.0 * kPi), 0.0, 0.05);
}

TEST(TrajectoryControllerClosedLoop, BrakesToAHoverWhenTheSetpointStopsArriving)
{
  DroneLoop loop;
  const Eigen::Vector3d target(30.0, 0.0, 5.0);
  const ControllerSetpoint sp = positionSetpoint(target);
  loop.run(3.0, &sp, target);  // flying towards the point at speed
  ASSERT_GT(loop.state.velocity.x(), 1.0);

  // The setpoint stream dies: the controller outputs zero velocity after 0.3 s and the
  // drone brakes to a hover well short of the target.
  loop.run(8.0, nullptr, target);
  EXPECT_LT(loop.state.velocity.norm(), 0.1);
  EXPECT_LT(loop.state.position.x(), target.x() - 2.0);
}

}  // namespace
}  // namespace control
}  // namespace interceptor
