#include "flight/quadrotor_model.hpp"

#include <algorithm>
#include <cmath>

namespace interceptor
{
namespace flight
{

QuadrotorParams makeX500Params()
{
  QuadrotorParams p;
  constexpr double kArm = 0.174;
  constexpr double kRotorZ = 0.06;
  p.rotors[0] = {Eigen::Vector3d(kArm, -kArm, kRotorZ), 1};    // front right, CCW
  p.rotors[1] = {Eigen::Vector3d(kArm, kArm, kRotorZ), -1};    // front left, CW
  p.rotors[2] = {Eigen::Vector3d(-kArm, kArm, kRotorZ), 1};    // rear left, CCW
  p.rotors[3] = {Eigen::Vector3d(-kArm, -kArm, kRotorZ), -1};  // rear right, CW
  return p;
}

QuadrotorModel::QuadrotorModel(const QuadrotorParams & params)
: params_(params)
{
}

void QuadrotorModel::setRotorSpeedCommands(const RotorArray & commands)
{
  for (int i = 0; i < kNumRotors; ++i) {
    commands_[i] = std::clamp(commands[i], 0.0, params_.max_rotor_speed);
  }
}

void QuadrotorModel::setRotorSpeeds(const RotorArray & speeds)
{
  for (int i = 0; i < kNumRotors; ++i) {
    speeds_[i] = std::clamp(speeds[i], 0.0, params_.max_rotor_speed);
    commands_[i] = speeds_[i];
  }
}

void QuadrotorModel::updateMotors(double dt)
{
  if (dt <= 0.0) {
    return;
  }
  for (int i = 0; i < kNumRotors; ++i) {
    const double tau = commands_[i] > speeds_[i] ?
      params_.motor_time_constant_up : params_.motor_time_constant_down;
    const double alpha = tau > 0.0 ? 1.0 - std::exp(-dt / tau) : 1.0;
    speeds_[i] += alpha * (commands_[i] - speeds_[i]);
  }
}

double QuadrotorModel::groundEffectFactor(double height) const
{
  if (!params_.enable_ground_effect) {
    return 1.0;
  }
  // The model diverges at h = R/4; below h = R/2 the ratio is held at 4/3.
  const double h = std::max(height, 0.5 * params_.rotor_radius);
  const double ratio = params_.rotor_radius / (4.0 * h);
  return 1.0 / (1.0 - ratio * ratio);
}

double QuadrotorModel::rotorThrust(double speed, double height) const
{
  const double density_ratio = params_.air_density / kSeaLevelDensity;
  return params_.thrust_coefficient * density_ratio * speed * speed *
         groundEffectFactor(height);
}

Wrench QuadrotorModel::computeWrench(
  const RigidBodyState & state, const Eigen::Vector3d & wind,
  AeroBreakdown * breakdown) const
{
  const Eigen::Matrix3d rot = state.orientation.toRotationMatrix();
  const double density_ratio = params_.air_density / kSeaLevelDensity;
  // Velocity of the airframe relative to the air, in the body frame
  const Eigen::Vector3d air_velocity = rot.transpose() * (state.velocity - wind);

  Wrench wrench;
  AeroBreakdown terms;

  for (int i = 0; i < kNumRotors; ++i) {
    const RotorGeometry & rotor = params_.rotors[i];
    const double speed = speeds_[i];

    // Thrust along the rotor axis (body z)
    const double hub_height = (state.position + rot * rotor.position).z() -
      params_.ground_height;
    const double thrust = rotorThrust(speed, hub_height);
    const Eigen::Vector3d thrust_force(0.0, 0.0, thrust);

    // Rotor drag (blade flapping and induced drag): opposes the in-plane airflow at
    // the hub and grows with rotor speed.
    Eigen::Vector3d hub_air_velocity = air_velocity +
      state.angular_velocity.cross(rotor.position);
    hub_air_velocity.z() = 0.0;
    const Eigen::Vector3d rotor_drag = -std::abs(speed) * params_.rotor_drag_coefficient *
      density_ratio * hub_air_velocity;

    const Eigen::Vector3d rotor_force = thrust_force + rotor_drag;
    wrench.force += rotor_force;
    wrench.torque += rotor.position.cross(rotor_force);
    // Aerodynamic reaction torque opposes the propeller's spin
    wrench.torque.z() -= rotor.direction * params_.moment_coefficient * thrust;

    terms.rotor_thrust[i] = thrust;
    terms.thrust_force += thrust_force;
    terms.rotor_drag_force += rotor_drag;
  }

  // Parasitic drag of the airframe, quadratic in the airspeed along each body axis
  const Eigen::Vector3d body_drag = -0.5 * params_.air_density *
    params_.drag_area.cwiseProduct(air_velocity.cwiseAbs()).cwiseProduct(air_velocity);
  wrench.force += body_drag;
  terms.body_drag_force = body_drag;

  // Rotational damping
  wrench.torque -= params_.angular_drag * state.angular_velocity;

  if (breakdown != nullptr) {
    *breakdown = terms;
  }
  return wrench;
}

void integrateRigidBody(
  RigidBodyState & state, const Wrench & wrench, double mass,
  const Eigen::Matrix3d & inertia, double dt, double ground_height)
{
  const Eigen::Matrix3d rot = state.orientation.toRotationMatrix();
  const Eigen::Vector3d accel = rot * wrench.force / mass - Eigen::Vector3d(0.0, 0.0, kGravity);
  state.velocity += accel * dt;
  state.position += state.velocity * dt;

  const Eigen::Vector3d & w = state.angular_velocity;
  const Eigen::Vector3d angular_accel = inertia.ldlt().solve(
    wrench.torque - w.cross(inertia * w));
  state.angular_velocity += angular_accel * dt;

  const Eigen::Vector3d rotation = state.angular_velocity * dt;
  const double angle = rotation.norm();
  if (angle > 1e-12) {
    state.orientation =
      state.orientation * Eigen::Quaterniond(Eigen::AngleAxisd(angle, rotation / angle));
    state.orientation.normalize();
  }

  if (state.position.z() < ground_height) {
    state.position.z() = ground_height;
    state.velocity.setZero();
    state.angular_velocity.setZero();
  }
}

}  // namespace flight
}  // namespace interceptor
