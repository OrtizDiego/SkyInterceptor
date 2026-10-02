#include "flight/flight_controller.hpp"

#include <algorithm>
#include <cmath>

namespace interceptor
{
namespace flight
{

namespace
{

double wrapAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

// vee map of a skew-symmetric matrix
Eigen::Vector3d vee(const Eigen::Matrix3d & m)
{
  return Eigen::Vector3d(m(2, 1), m(0, 2), m(1, 0));
}

Eigen::Vector3d clampVector(const Eigen::Vector3d & v, const Eigen::Vector3d & limit)
{
  return v.cwiseMax(-limit).cwiseMin(limit);
}

}  // namespace

Eigen::Vector3d headingToWorld(const Eigen::Vector3d & velocity, double yaw)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return Eigen::Vector3d(
    c * velocity.x() - s * velocity.y(),
    s * velocity.x() + c * velocity.y(),
    velocity.z());
}

double yawFromQuaternion(const Eigen::Quaterniond & q)
{
  return std::atan2(
    2.0 * (q.w() * q.z() + q.x() * q.y()),
    1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

FlightController::FlightController(
  const QuadrotorParams & vehicle, const FlightControllerParams & params)
: vehicle_(vehicle), params_(params)
{
  buildAllocation();
}

void FlightController::buildAllocation()
{
  // [T, tau_x, tau_y, tau_z]^T = A * [T_1 .. T_4]^T
  Eigen::Matrix4d a;
  for (int i = 0; i < kNumRotors; ++i) {
    const RotorGeometry & rotor = vehicle_.rotors[i];
    a(0, i) = 1.0;
    a(1, i) = rotor.position.y();
    a(2, i) = -rotor.position.x();
    a(3, i) = -rotor.direction * vehicle_.moment_coefficient;
  }
  allocation_inverse_ = a.inverse();
}

void FlightController::reset(const RigidBodyState & state)
{
  velocity_integral_.setZero();
  rate_integral_.setZero();
  yaw_setpoint_ = yawFromQuaternion(state.orientation);
  desired_attitude_ = state.orientation.toRotationMatrix();
  collective_thrust_ = 0.0;
  landed_ = true;
  land_timer_ = 0.0;
}

RotorArray FlightController::allocate(double thrust, const Eigen::Vector3d & torque) const
{
  const double kf = vehicle_.thrust_coefficient * vehicle_.air_density / kSeaLevelDensity;
  const double t_max = kf * vehicle_.max_rotor_speed * vehicle_.max_rotor_speed;
  const double t_min = kf * params_.idle_rotor_speed * params_.idle_rotor_speed;

  Eigen::Vector4d t_rp = allocation_inverse_ * Eigen::Vector4d(thrust, torque.x(), torque.y(), 0);
  const Eigen::Vector4d t_yaw = allocation_inverse_ * Eigen::Vector4d(0, 0, 0, torque.z());

  // 1. Roll/pitch differential must fit in the thrust range: scale it down if not.
  double hi = t_rp.maxCoeff();
  double lo = t_rp.minCoeff();
  const double mean = t_rp.mean();
  if (hi - lo > t_max - t_min) {
    const double scale = (t_max - t_min) / (hi - lo);
    t_rp = (t_rp.array() - mean) * scale + mean;
    hi = t_rp.maxCoeff();
    lo = t_rp.minCoeff();
  }
  // 2. Shift the collective so roll/pitch fit (attitude has priority over thrust).
  if (hi > t_max) {
    t_rp.array() -= hi - t_max;
  } else if (lo < t_min) {
    t_rp.array() += t_min - lo;
  }
  // 3. Add as much yaw torque as the remaining headroom allows.
  double yaw_scale = 1.0;
  for (int i = 0; i < kNumRotors; ++i) {
    if (t_yaw[i] > 1e-12) {
      yaw_scale = std::min(yaw_scale, (t_max - t_rp[i]) / t_yaw[i]);
    } else if (t_yaw[i] < -1e-12) {
      yaw_scale = std::min(yaw_scale, (t_min - t_rp[i]) / t_yaw[i]);
    }
  }
  yaw_scale = std::max(yaw_scale, 0.0);

  RotorArray speeds{};
  for (int i = 0; i < kNumRotors; ++i) {
    const double t = std::clamp(t_rp[i] + yaw_scale * t_yaw[i], t_min, t_max);
    speeds[i] = std::sqrt(t / kf);
  }
  return speeds;
}

RotorArray FlightController::update(
  const RigidBodyState & state, const VelocityCommand & command, double dt)
{
  const Eigen::Matrix3d rot = state.orientation.toRotationMatrix();
  const double yaw = yawFromQuaternion(state.orientation);

  // Command limits
  Eigen::Vector3d v_cmd = command.velocity;
  const double h_speed = v_cmd.head<2>().norm();
  if (h_speed > params_.max_horizontal_speed) {
    v_cmd.head<2>() *= params_.max_horizontal_speed / h_speed;
  }
  v_cmd.z() = std::clamp(v_cmd.z(), -params_.max_vertical_speed, params_.max_vertical_speed);
  const double yaw_rate = std::clamp(
    command.yaw_rate, -params_.max_yaw_rate, params_.max_yaw_rate);

  // On the ground: idle until a climb is commanded
  if (landed_) {
    if (v_cmd.z() < params_.takeoff_climb_speed) {
      reset(state);
      return allocate(0.0, Eigen::Vector3d::Zero());
    }
    landed_ = false;
  }

  // Heading setpoint: integrate the yaw-rate command, never far ahead of the vehicle
  yaw_setpoint_ = wrapAngle(yaw_setpoint_ + yaw_rate * dt);
  const double yaw_error = wrapAngle(yaw_setpoint_ - yaw);
  if (std::abs(yaw_error) > params_.max_yaw_error) {
    yaw_setpoint_ = wrapAngle(yaw + std::copysign(params_.max_yaw_error, yaw_error));
  }

  // Velocity loop -> desired acceleration -> thrust vector
  const Eigen::Vector3d v_error = v_cmd - state.velocity;
  Eigen::Vector3d accel = params_.velocity_p.cwiseProduct(v_error) + velocity_integral_;
  const bool z_high = accel.z() > params_.max_vertical_accel_up;
  const bool z_low = accel.z() < -params_.max_vertical_accel_down;
  accel.z() = std::clamp(
    accel.z(), -params_.max_vertical_accel_down, params_.max_vertical_accel_up);

  Eigen::Vector3d force = vehicle_.mass * (accel + Eigen::Vector3d(0.0, 0.0, kGravity));
  const double max_horizontal = force.z() * std::tan(params_.max_tilt);
  const double horizontal = force.head<2>().norm();
  const bool tilt_saturated = horizontal > max_horizontal;
  if (tilt_saturated) {
    force.head<2>() *= max_horizontal / horizontal;
  }

  // Integrate with anti-windup: stop integrating an axis that is saturated
  const Eigen::Vector3d increment = params_.velocity_i.cwiseProduct(v_error) * dt;
  if (!tilt_saturated) {
    velocity_integral_.head<2>() += increment.head<2>();
  }
  if (!(z_high && v_error.z() > 0.0) && !(z_low && v_error.z() < 0.0)) {
    velocity_integral_.z() += increment.z();
  }
  velocity_integral_ = clampVector(velocity_integral_, params_.max_velocity_integral);

  // Desired attitude: body z along the thrust vector, body x towards the yaw setpoint
  const Eigen::Vector3d b3 = force.normalized();
  const Eigen::Vector3d heading(std::cos(yaw_setpoint_), std::sin(yaw_setpoint_), 0.0);
  const Eigen::Vector3d b2 = b3.cross(heading).normalized();
  const Eigen::Vector3d b1 = b2.cross(b3);
  desired_attitude_.col(0) = b1;
  desired_attitude_.col(1) = b2;
  desired_attitude_.col(2) = b3;

  // Collective thrust: the part of the desired force along the current body z axis
  collective_thrust_ = std::max(force.dot(rot.col(2)), 0.0);

  // Touchdown: asked to descend (or hold) but barely moving on low thrust
  const bool touchdown = v_cmd.z() <= 0.0 &&
    state.velocity.norm() < params_.land_max_speed &&
    collective_thrust_ < params_.land_thrust_ratio * vehicle_.mass * kGravity;
  land_timer_ = touchdown ? land_timer_ + dt : 0.0;
  if (land_timer_ >= params_.land_detect_time) {
    reset(state);
    return allocate(0.0, Eigen::Vector3d::Zero());
  }

  // Attitude loop (geometric error on SO(3)) -> body-rate setpoint
  const Eigen::Vector3d attitude_error =
    0.5 * vee(desired_attitude_.transpose() * rot - rot.transpose() * desired_attitude_);
  Eigen::Vector3d rate_cmd = -params_.attitude_p.cwiseProduct(attitude_error);
  rate_cmd += rot.transpose() * Eigen::Vector3d(0.0, 0.0, yaw_rate);
  rate_cmd = clampVector(rate_cmd, params_.max_body_rate);

  // Body-rate loop -> torque
  const Eigen::Vector3d & w = state.angular_velocity;
  const Eigen::Vector3d rate_error = rate_cmd - w;
  rate_integral_ = clampVector(
    rate_integral_ + params_.rate_i.cwiseProduct(rate_error) * dt, params_.max_rate_integral);
  const Eigen::Vector3d angular_accel = params_.rate_p.cwiseProduct(rate_error) + rate_integral_;
  const Eigen::Vector3d torque = vehicle_.inertia * angular_accel + w.cross(vehicle_.inertia * w);

  return allocate(collective_thrust_, torque);
}

}  // namespace flight
}  // namespace interceptor
