#include <gtest/gtest.h>
#include <Eigen/Dense>

#include <cmath>
#include <vector>

#include "flight/flight_controller.hpp"
#include "flight/quadrotor_model.hpp"
#include "flight/wind_model.hpp"

namespace interceptor
{
namespace flight
{
namespace
{

constexpr double kDt = 0.001;

double hoverSpeed(const QuadrotorParams & p)
{
  return std::sqrt(p.mass * kGravity / (kNumRotors * p.thrust_coefficient));
}

RigidBodyState stateAt(double z)
{
  RigidBodyState s;
  s.position = Eigen::Vector3d(0.0, 0.0, z);
  return s;
}

// Closed loop: flight controller + motors + aerodynamics + rigid body, at 1 kHz.
struct ClosedLoopSim
{
  QuadrotorParams params = makeX500Params();
  QuadrotorModel model{params};
  FlightController controller{params};
  WindModel wind;
  RigidBodyState state = stateAt(5.0);

  // Starts hovering in the air (already flying, past the take-off)
  ClosedLoopSim()
  {
    const double w = hoverSpeed(params);
    model.setRotorSpeeds(RotorArray{w, w, w, w});
    controller.reset(state);
    VelocityCommand climb;
    climb.velocity.z() = 0.2;
    controller.update(state, climb, kDt);  // leaves the landed state
  }

  void run(double seconds, const VelocityCommand & cmd)
  {
    const int steps = static_cast<int>(std::round(seconds / kDt));
    for (int k = 0; k < steps; ++k) {
      model.setRotorSpeedCommands(controller.update(state, cmd, kDt));
      model.updateMotors(kDt);
      const Wrench wrench = model.computeWrench(state, wind.update(kDt));
      integrateRigidBody(state, wrench, params.mass, params.inertia, kDt);
    }
  }
};

// --- Plant --------------------------------------------------------------------

TEST(QuadrotorModel, HoverSpeedBalancesWeight)
{
  QuadrotorModel model;
  const double w = hoverSpeed(model.params());
  model.setRotorSpeeds(RotorArray{w, w, w, w});
  const Wrench wrench = model.computeWrench(stateAt(50.0), Eigen::Vector3d::Zero());
  EXPECT_NEAR(wrench.force.z(), model.params().mass * kGravity, 1e-3);
  EXPECT_NEAR(wrench.force.head<2>().norm(), 0.0, 1e-9);
  // CW and CCW reaction torques cancel, the symmetric layout gives no roll/pitch torque
  EXPECT_NEAR(wrench.torque.norm(), 0.0, 1e-9);
  // Realistic X500 hover point: roughly 6500-7000 RPM
  EXPECT_GT(w, 600.0);
  EXPECT_LT(w, 800.0);
}

TEST(QuadrotorModel, ThrustGrowsWithSquareOfRotorSpeed)
{
  QuadrotorModel model;
  const double t1 = model.rotorThrust(300.0, 100.0);
  const double t2 = model.rotorThrust(600.0, 100.0);
  EXPECT_NEAR(t2 / t1, 4.0, 1e-3);
}

TEST(QuadrotorModel, ThinnerAirGivesLessThrust)
{
  QuadrotorParams p = makeX500Params();
  p.air_density = 0.5 * kSeaLevelDensity;
  QuadrotorModel thin(p);
  QuadrotorModel sea_level;
  EXPECT_NEAR(thin.rotorThrust(500.0, 100.0) / sea_level.rotorThrust(500.0, 100.0), 0.5, 1e-3);
}

TEST(QuadrotorModel, GroundEffectBoostsThrustNearGround)
{
  QuadrotorModel model;
  const double r = model.params().rotor_radius;
  EXPECT_NEAR(model.groundEffectFactor(20.0), 1.0, 1e-4);
  EXPECT_GT(model.groundEffectFactor(r), model.groundEffectFactor(2.0 * r));
  EXPECT_GT(model.groundEffectFactor(2.0 * r), 1.0);
  // Capped below h = R/2
  EXPECT_NEAR(model.groundEffectFactor(0.0), 4.0 / 3.0, 1e-9);

  QuadrotorParams p = makeX500Params();
  p.enable_ground_effect = false;
  EXPECT_DOUBLE_EQ(QuadrotorModel(p).groundEffectFactor(0.0), 1.0);
}

TEST(QuadrotorModel, DifferentialThrustGivesExpectedTorqueSigns)
{
  QuadrotorModel model;
  const double w = 500.0;
  // Rotor order: FR, FL, RL, RR
  model.setRotorSpeeds(RotorArray{w + 50, w, w, w + 50});  // right side faster
  EXPECT_LT(model.computeWrench(stateAt(50.0), Eigen::Vector3d::Zero()).torque.x(), 0.0);

  model.setRotorSpeeds(RotorArray{w + 50, w + 50, w, w});  // front faster: nose up
  EXPECT_LT(model.computeWrench(stateAt(50.0), Eigen::Vector3d::Zero()).torque.y(), 0.0);

  model.setRotorSpeeds(RotorArray{w, w + 50, w, w + 50});  // CW rotors faster: yaw left
  EXPECT_GT(model.computeWrench(stateAt(50.0), Eigen::Vector3d::Zero()).torque.z(), 0.0);
}

TEST(QuadrotorModel, DragPushesDroneDownwind)
{
  QuadrotorModel model;
  const double w = hoverSpeed(model.params());
  model.setRotorSpeeds(RotorArray{w, w, w, w});
  AeroBreakdown terms;
  const Eigen::Vector3d wind(6.0, 0.0, 0.0);
  model.computeWrench(stateAt(50.0), wind, &terms);
  EXPECT_GT(terms.body_drag_force.x(), 0.0);
  EXPECT_GT(terms.rotor_drag_force.x(), 0.0);
  // Quadratic body drag: 0.5 * rho * CdA * v^2
  EXPECT_NEAR(terms.body_drag_force.x(), 0.5 * kSeaLevelDensity * 0.04 * 36.0, 1e-9);

  // Flying at the wind speed through still air is the same as hovering in that wind
  RigidBodyState moving = stateAt(50.0);
  moving.velocity = -wind;
  AeroBreakdown moving_terms;
  model.computeWrench(moving, Eigen::Vector3d::Zero(), &moving_terms);
  EXPECT_TRUE(moving_terms.body_drag_force.isApprox(terms.body_drag_force, 1e-9));
}

TEST(QuadrotorModel, MotorFollowsFirstOrderLag)
{
  QuadrotorModel model;
  model.setRotorSpeedCommands(RotorArray{800, 800, 800, 800});
  const double tau = model.params().motor_time_constant_up;
  const int steps = 10;
  for (int k = 0; k < steps; ++k) {
    model.updateMotors(kDt);
  }
  EXPECT_NEAR(model.rotorSpeeds()[0] / 800.0, 1.0 - std::exp(-steps * kDt / tau), 1e-9);
  // Spinning down is slower than spinning up
  model.setRotorSpeedCommands(RotorArray{0, 0, 0, 0});
  const double before = model.rotorSpeeds()[0];
  model.updateMotors(kDt);
  EXPECT_NEAR(
    model.rotorSpeeds()[0] / before, std::exp(-kDt / model.params().motor_time_constant_down),
    1e-9);
  // Commands are clamped to the motor limit
  model.setRotorSpeedCommands(RotorArray{5000, 0, 0, 0});
  EXPECT_DOUBLE_EQ(model.rotorSpeedCommands()[0], model.params().max_rotor_speed);
}

TEST(QuadrotorModel, StoppedRotorsFallWithQuadraticDrag)
{
  QuadrotorModel model;
  const QuadrotorParams & p = model.params();
  RigidBodyState s = stateAt(100.0);
  const double t = 1.5;
  for (int k = 0; k < static_cast<int>(t / kDt); ++k) {
    const Wrench wrench = model.computeWrench(s, Eigen::Vector3d::Zero());
    integrateRigidBody(s, wrench, p.mass, p.inertia, kDt);
  }
  // Analytic fall with quadratic drag: v(t) = -v_t tanh(g t / v_t)
  const double v_terminal =
    std::sqrt(2.0 * p.mass * kGravity / (p.air_density * p.drag_area.z()));
  EXPECT_NEAR(s.velocity.z(), -v_terminal * std::tanh(kGravity * t / v_terminal), 0.02);
  EXPECT_GT(s.velocity.z(), -kGravity * t);
}

// --- Wind ---------------------------------------------------------------------

TEST(WindModel, SteadyWindWithoutGusts)
{
  WindParams p;
  p.mean = Eigen::Vector3d(3.0, -1.0, 0.0);
  WindModel wind(p);
  EXPECT_TRUE(wind.update(0.01).isApprox(p.mean));
}

TEST(WindModel, GustsHaveRequestedIntensity)
{
  WindParams p;
  p.gust_stddev = 1.5;
  p.gust_time_constant = 0.5;
  WindModel wind(p);
  const int n = 400000;
  double sum = 0.0;
  double sum_sq = 0.0;
  for (int k = 0; k < n; ++k) {
    const double g = wind.update(0.01).x();
    sum += g;
    sum_sq += g * g;
  }
  const double mean = sum / n;
  const double stddev = std::sqrt(sum_sq / n - mean * mean);
  EXPECT_NEAR(mean, 0.0, 0.1);
  EXPECT_NEAR(stddev, 1.5, 0.1);
}

TEST(WindModel, ResetIsDeterministic)
{
  WindParams p;
  p.gust_stddev = 1.0;
  WindModel wind(p);
  std::vector<double> first;
  for (int k = 0; k < 50; ++k) {
    first.push_back(wind.update(0.01).y());
  }
  wind.reset();
  for (int k = 0; k < 50; ++k) {
    EXPECT_DOUBLE_EQ(wind.update(0.01).y(), first[k]);
  }
}

// --- Controller -----------------------------------------------------------------

TEST(FlightController, HeadingFrameRotation)
{
  const Eigen::Vector3d v = headingToWorld(Eigen::Vector3d(1.0, 0.0, 0.5), M_PI / 2.0);
  EXPECT_TRUE(v.isApprox(Eigen::Vector3d(0.0, 1.0, 0.5), 1e-12));
  const Eigen::Quaterniond q(Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()));
  EXPECT_NEAR(yawFromQuaternion(q), 0.7, 1e-12);
}

TEST(FlightController, AllocationRoundTrip)
{
  const QuadrotorParams p = makeX500Params();
  FlightController controller(p);
  QuadrotorModel model(p);
  const double thrust = 20.0;
  const Eigen::Vector3d torque(0.3, -0.2, 0.05);
  model.setRotorSpeeds(controller.allocate(thrust, torque));
  const Wrench w = model.computeWrench(stateAt(50.0), Eigen::Vector3d::Zero());
  EXPECT_NEAR(w.force.z(), thrust, 1e-3);
  EXPECT_TRUE(w.torque.isApprox(torque, 1e-3));
}

TEST(FlightController, AllocationKeepsRollPitchWhenSaturated)
{
  const QuadrotorParams p = makeX500Params();
  FlightController controller(p);
  QuadrotorModel model(p);
  const Eigen::Vector3d torque(0.5, 0.0, 5.0);  // impossible yaw demand
  model.setRotorSpeeds(controller.allocate(17.0, torque));
  const Wrench w = model.computeWrench(stateAt(50.0), Eigen::Vector3d::Zero());
  EXPECT_NEAR(w.torque.x(), 0.5, 1e-3);
  EXPECT_NEAR(w.torque.y(), 0.0, 1e-3);
  EXPECT_GT(w.torque.z(), 0.0);
  EXPECT_LT(w.torque.z(), 5.0);
}

TEST(FlightControllerClosedLoop, HoldsHover)
{
  ClosedLoopSim sim;
  sim.run(10.0, VelocityCommand());
  EXPECT_NEAR(sim.state.position.z(), 5.0, 0.05);
  EXPECT_LT(sim.state.position.head<2>().norm(), 0.05);
  EXPECT_LT(sim.state.velocity.norm(), 0.01);
}

TEST(FlightControllerClosedLoop, TracksForwardVelocity)
{
  ClosedLoopSim sim;
  VelocityCommand cmd;
  cmd.velocity = Eigen::Vector3d(4.0, 0.0, 0.0);
  sim.run(8.0, cmd);
  EXPECT_NEAR(sim.state.velocity.x(), 4.0, 0.05);
  EXPECT_NEAR(sim.state.velocity.y(), 0.0, 0.05);
  EXPECT_NEAR(sim.state.position.z(), 5.0, 0.2);
  // Flying forward against drag needs the nose down: thrust axis leans forward
  const Eigen::Vector3d body_z = sim.state.orientation * Eigen::Vector3d::UnitZ();
  EXPECT_GT(body_z.x(), 0.01);
}

TEST(FlightControllerClosedLoop, ClimbsAndDescends)
{
  ClosedLoopSim sim;
  VelocityCommand cmd;
  cmd.velocity.z() = 2.0;
  sim.run(5.0, cmd);
  EXPECT_NEAR(sim.state.velocity.z(), 2.0, 0.05);
  EXPECT_GT(sim.state.position.z(), 12.0);
  cmd.velocity.z() = -1.0;
  sim.run(5.0, cmd);
  EXPECT_NEAR(sim.state.velocity.z(), -1.0, 0.05);
}

TEST(FlightControllerClosedLoop, YawRate)
{
  ClosedLoopSim sim;
  VelocityCommand cmd;
  cmd.yaw_rate = 0.5;
  sim.run(2.0, cmd);
  EXPECT_NEAR(yawFromQuaternion(sim.state.orientation), 1.0, 0.1);
  EXPECT_NEAR(sim.state.position.z(), 5.0, 0.05);
}

TEST(FlightControllerClosedLoop, RejectsWindAndGusts)
{
  ClosedLoopSim sim;
  WindParams wp;
  wp.mean = Eigen::Vector3d(6.0, 0.0, 0.0);
  wp.gust_stddev = 1.0;
  sim.wind = WindModel(wp);
  sim.run(30.0, VelocityCommand());
  // Drifts while the integrator builds up, then holds against the wind
  EXPECT_LT(sim.state.velocity.norm(), 0.3);
  EXPECT_LT(sim.state.position.head<2>().norm(), 3.0);
  EXPECT_NEAR(sim.state.position.z(), 5.0, 0.3);
  // Tilted into the wind: thrust axis leans upwind (-x)
  const Eigen::Vector3d body_z = sim.state.orientation * Eigen::Vector3d::UnitZ();
  EXPECT_LT(body_z.x(), -0.02);
}

TEST(FlightControllerClosedLoop, IdlesOnGroundUntilClimbCommanded)
{
  ClosedLoopSim sim;
  sim.state = stateAt(0.0);
  sim.model.setRotorSpeeds(RotorArray{});
  sim.controller.reset(sim.state);

  // Armed on the ground: horizontal commands alone don't take off
  VelocityCommand cmd;
  cmd.velocity.x() = 2.0;
  sim.run(2.0, cmd);
  EXPECT_TRUE(sim.controller.landed());
  EXPECT_NEAR(sim.state.position.norm(), 0.0, 1e-9);
  const double idle = sim.controller.params().idle_rotor_speed;
  EXPECT_NEAR(sim.model.rotorSpeeds()[0], idle, 1.0);

  // Climb command: take off
  cmd.velocity = Eigen::Vector3d(0.0, 0.0, 1.0);
  sim.run(4.0, cmd);
  EXPECT_FALSE(sim.controller.landed());
  EXPECT_GT(sim.state.position.z(), 2.5);
}

TEST(FlightControllerClosedLoop, DetectsTouchdown)
{
  ClosedLoopSim sim;
  sim.state = stateAt(1.0);
  VelocityCommand cmd;
  cmd.velocity.z() = -1.0;
  sim.run(1.5, cmd);
  EXPECT_FALSE(sim.controller.landed());  // still descending
  sim.run(3.0, cmd);
  EXPECT_TRUE(sim.controller.landed());
  EXPECT_NEAR(sim.state.position.z(), 0.0, 1e-9);
  EXPECT_NEAR(sim.model.rotorSpeeds()[0], sim.controller.params().idle_rotor_speed, 1.0);
}

TEST(FlightControllerClosedLoop, NoFalseTouchdownInFlight)
{
  ClosedLoopSim sim;
  VelocityCommand cmd;
  cmd.velocity.z() = -2.0;  // fast descent start: low thrust but not landed
  sim.run(1.0, cmd);
  cmd.velocity.z() = 0.0;   // then brake and hover
  sim.run(5.0, cmd);
  EXPECT_FALSE(sim.controller.landed());
  EXPECT_GT(sim.state.position.z(), 2.0);
}

}  // namespace
}  // namespace flight
}  // namespace interceptor
