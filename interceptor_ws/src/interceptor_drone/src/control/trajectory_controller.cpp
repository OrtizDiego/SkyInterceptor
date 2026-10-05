#include "control/trajectory_controller.hpp"

#include <algorithm>
#include <cmath>

#include "common/math_utils.hpp"

namespace interceptor
{
namespace control
{

namespace
{

// Longest step the integrator is allowed to take; a longer gap (paused clock, first
// call after a restart) integrates nothing instead of a huge step.
constexpr double kMaxIntegrationStep = 0.5;

}  // namespace

TrajectoryController::TrajectoryController(const TrajectoryControllerParams & params)
: params_(params)
{
}

bool TrajectoryController::setSetpoint(const ControllerSetpoint & setpoint, double time_s)
{
  if (!setpoint.position.allFinite() || !setpoint.velocity.allFinite() ||
    !std::isfinite(setpoint.yaw) || !std::isfinite(setpoint.yaw_rate) ||
    !std::isfinite(time_s))
  {
    return false;
  }
  if (setpoint_ &&
    (setpoint_->source != setpoint.source || setpoint_->position_valid != setpoint.position_valid))
  {
    resetIntegrator();
  }
  setpoint_ = setpoint;
  setpoint_time_ = time_s;
  return true;
}

ControllerCommand TrajectoryController::update(const ControllerState & state, double time_s)
{
  // A backward clock jump (simulation restart) invalidates everything we know
  if (has_last_update_ && time_s < last_update_time_) {
    reset();
  }
  double dt = has_last_update_ ? time_s - last_update_time_ : 0.0;
  if (dt > kMaxIntegrationStep) {
    dt = 0.0;
  }
  last_update_time_ = time_s;
  has_last_update_ = true;

  if (!setpoint_ || time_s - setpoint_time_ > params_.setpoint_timeout) {
    ControllerCommand stop = hold();
    stop.timed_out = true;
    return stop;
  }
  const ControllerSetpoint & sp = *setpoint_;

  ControllerCommand command;
  if (sp.position_valid) {
    const Eigen::Vector3d error = sp.position - state.position;
    const Eigen::Vector3d unsaturated = sp.velocity + params_.kp_pos * error + integral_ +
      params_.kd_pos * (sp.velocity - state.velocity);
    command.velocity = saturate(unsaturated);

    // Integrator: accumulate only close to the setpoint (a long approach would build up
    // a large integral and cause overshoot), unwind by back-calculation while
    // saturated, clamp
    Eigen::Vector3d next = integral_;
    if (error.norm() < params_.integral_zone) {
      next += dt * params_.ki_pos * error;
    }
    const Eigen::Vector3d back = dt * params_.kb_antiwindup * (command.velocity - unsaturated);
    for (int i = 0; i < 3; ++i) {
      // Only unwind: a correction pointing away from zero (saturation on the opposite
      // side of the integral term) would wind the integrator up instead.
      if (back[i] * next[i] < 0.0) {
        next[i] = std::abs(back[i]) >= std::abs(next[i]) ? 0.0 : next[i] + back[i];
      }
    }
    const double limit = params_.max_integral_velocity;
    const double horizontal = next.head<2>().norm();
    if (horizontal > limit) {
      next.head<2>() *= limit / horizontal;
    }
    next.z() = std::clamp(next.z(), -limit, limit);
    integral_ = next;
  } else {
    integral_.setZero();
    command.velocity = saturate(sp.velocity);
  }

  command.yaw_rate = std::clamp(
    sp.yaw_rate + params_.kp_yaw * wrapAngle(sp.yaw - state.yaw),
    -params_.max_yaw_rate, params_.max_yaw_rate);
  return command;
}

ControllerCommand TrajectoryController::hold()
{
  integral_.setZero();
  return ControllerCommand();
}

void TrajectoryController::resetIntegrator()
{
  integral_.setZero();
}

void TrajectoryController::reset()
{
  setpoint_.reset();
  has_last_update_ = false;
  integral_.setZero();
}

Eigen::Vector3d TrajectoryController::saturate(const Eigen::Vector3d & velocity) const
{
  Eigen::Vector3d out = velocity;
  const double horizontal = out.head<2>().norm();
  if (horizontal > params_.max_horizontal_velocity) {
    out.head<2>() *= params_.max_horizontal_velocity / horizontal;
  }
  out.z() = std::clamp(out.z(), -params_.max_vertical_velocity, params_.max_vertical_velocity);
  return out;
}

}  // namespace control
}  // namespace interceptor
