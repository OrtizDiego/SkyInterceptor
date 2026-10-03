#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <array>

namespace interceptor
{
namespace flight
{

// Frames (REP-103): world is ENU with z up, the body frame is FLU
// (x forward, y left, z up). Rotor thrust acts along body +z.

constexpr int kNumRotors = 4;
constexpr double kGravity = 9.80665;      // m/s^2
constexpr double kSeaLevelDensity = 1.225;  // kg/m^3, ISA sea level

using RotorArray = std::array<double, kNumRotors>;

struct RotorGeometry
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();  // hub position in the body frame [m]
  int direction = 1;  // +1: spins CCW seen from above (+z), -1: spins CW
};

struct QuadrotorParams
{
  // Rigid body (Gazebo uses the URDF values, these are used by the controller and tests)
  double mass = 1.75;  // kg
  Eigen::Matrix3d inertia = Eigen::Vector3d(0.040, 0.044, 0.080).asDiagonal();  // kg m^2
  std::array<RotorGeometry, kNumRotors> rotors{};

  // Rotor aerodynamics. Defaults are the PX4 x500 SITL values (2216 920KV motor, 10-11" prop).
  double thrust_coefficient = 8.54858e-6;      // k_f: T = k_f * w^2 [N/(rad/s)^2] at sea level
  double moment_coefficient = 0.016;           // k_m: reaction torque Q = k_m * T [m]
  double rotor_drag_coefficient = 8.06428e-5;  // blade-flapping / induced drag [N/(m/s)/(rad/s)]
  double rotor_radius = 0.1397;                // m (11" propeller)
  double max_rotor_speed = 1000.0;             // rad/s
  double motor_time_constant_up = 0.0125;      // s, first-order spin-up
  double motor_time_constant_down = 0.025;     // s, first-order spin-down

  // Airframe aerodynamics
  double air_density = kSeaLevelDensity;  // kg/m^3
  // Drag coefficient times reference area per body axis, C_d * A [m^2]
  Eigen::Vector3d drag_area = Eigen::Vector3d(0.04, 0.04, 0.08);
  double angular_drag = 2.0e-3;  // rotational damping [N m s/rad]

  // Ground effect (Cheeseman-Bennett). Height of the ground plane in world z.
  bool enable_ground_effect = true;
  double ground_height = 0.0;
};

// Holybro X500 V2 geometry used by the URDF: motors at (+-0.174, +-0.174, 0.06) m.
// PX4 quad-X numbering: FR and RL spin CCW, FL and RR spin CW.
QuadrotorParams makeX500Params();

struct RigidBodyState
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();         // world [m]
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();  // body -> world
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();         // world [m/s]
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();  // body [rad/s]
};

// Force and torque in the body frame. The force acts at the body origin and the torque
// is taken about the body origin. Gravity is not included: the physics engine adds it.
struct Wrench
{
  Eigen::Vector3d force = Eigen::Vector3d::Zero();
  Eigen::Vector3d torque = Eigen::Vector3d::Zero();
};

// Per-term breakdown, useful for telemetry and tests (all body frame).
struct AeroBreakdown
{
  RotorArray rotor_thrust{};  // N
  Eigen::Vector3d thrust_force = Eigen::Vector3d::Zero();
  Eigen::Vector3d rotor_drag_force = Eigen::Vector3d::Zero();
  Eigen::Vector3d body_drag_force = Eigen::Vector3d::Zero();
};

// Quadrotor plant: motor dynamics plus the rotor and airframe aerodynamics that turn
// rotor speeds into a body wrench.
class QuadrotorModel
{
public:
  explicit QuadrotorModel(const QuadrotorParams & params = makeX500Params());

  const QuadrotorParams & params() const {return params_;}

  // Rotor speed commands [rad/s], clamped to [0, max_rotor_speed].
  void setRotorSpeedCommands(const RotorArray & commands);
  // Bypass motor dynamics (tests, resets).
  void setRotorSpeeds(const RotorArray & speeds);
  // Advance the first-order motor model by dt seconds.
  void updateMotors(double dt);

  const RotorArray & rotorSpeeds() const {return speeds_;}
  const RotorArray & rotorSpeedCommands() const {return commands_;}

  // Thrust of one rotor spinning at `speed` [rad/s] with its hub `height` above ground.
  double rotorThrust(double speed, double height) const;

  // Cheeseman-Bennett in-ground-effect thrust ratio T_IGE / T_OGE (>= 1).
  double groundEffectFactor(double height) const;

  // Wrench produced by the current rotor speeds. `wind` is the air velocity in world frame.
  Wrench computeWrench(
    const RigidBodyState & state, const Eigen::Vector3d & wind,
    AeroBreakdown * breakdown = nullptr) const;

private:
  QuadrotorParams params_;
  RotorArray commands_{};
  RotorArray speeds_{};
};

// Semi-implicit Euler step of the 6-DOF rigid-body equations with gravity and a flat
// ground at `ground_height` (no bounce, no sliding). In Gazebo the physics engine does
// this; it's used for unit tests and offline tuning. The body origin is the CoG.
void integrateRigidBody(
  RigidBodyState & state, const Wrench & wrench, double mass,
  const Eigen::Matrix3d & inertia, double dt, double ground_height = 0.0);

}  // namespace flight
}  // namespace interceptor
