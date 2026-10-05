#pragma once

#include <Eigen/Dense>

#include <cstdint>
#include <optional>

namespace interceptor
{
namespace control
{

struct TrajectoryControllerParams
{
  // Position loop: velocity = v_ff + kp * e + I + kd * (v_ff - v)
  double kp_pos = 1.0;
  double ki_pos = 0.05;                // 1/s^2: I accumulates ki * e [m/s]
  double kd_pos = 0.2;                 // damping on the velocity error (derivative of e)
  double kb_antiwindup = 2.0;          // 1/s, back-calculation gain on saturation
  double max_integral_velocity = 1.0;  // m/s, clamp on the integral term
  double integral_zone = 2.0;          // m, the integral only accumulates inside this error

  // Yaw loop: yaw_rate = yaw_rate_ff + kp * wrap(yaw_sp - yaw), then rate limited
  double kp_yaw = 1.5;

  // Output limits
  double max_horizontal_velocity = 30.0;  // m/s
  double max_vertical_velocity = 5.0;     // m/s
  double max_yaw_rate = 1.5;              // rad/s

  // No fresh setpoint for this long: command zero velocity
  double setpoint_timeout = 0.3;  // s
};

// What the trajectory controller tracks. World frame (ENU), like FlightSetpoint.
struct ControllerSetpoint
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();  // m, used when position_valid
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();  // m/s, feed-forward or velocity target
  double yaw = 0.0;                                    // rad
  double yaw_rate = 0.0;                               // rad/s, feed-forward
  bool position_valid = false;
  uint8_t source = 0;  // FlightSetpoint source; a change resets the integrator
};

// Vehicle state from odometry. World frame.
struct ControllerState
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();  // m
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();  // m/s
  double yaw = 0.0;                                    // rad
};

// World-frame velocity and yaw rate: the /cmd_vel content.
struct ControllerCommand
{
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  double yaw_rate = 0.0;
  bool timed_out = false;  // true when no setpoint is active: the command is zero
};

// Position PID -> velocity command, yaw P with a rate limit. Plain C++ and Eigen, no ROS.
//
// The onboard flight controller of the drone already closes a velocity loop, so this
// controller stops at the velocity command: the setpoint velocity is passed through as
// feed-forward and the setpoint acceleration is not used.
//
// With position_valid = false the setpoint velocity is tracked directly (no position
// feedback, integrator held at zero).
//
// Anti-windup: the integral term only accumulates within integral_zone of the setpoint,
// is clamped, and while the output is saturated back-calculation unwinds it (saturation
// never winds it up further).
class TrajectoryController
{
public:
  explicit TrajectoryController(const TrajectoryControllerParams & params = {});

  const TrajectoryControllerParams & params() const {return params_;}

  // Stores a new setpoint received at time_s (seconds on the clock passed to update).
  // Returns false and keeps the previous setpoint when it has non-finite values. The
  // integrator is reset when the source or the position_valid flag changes.
  bool setSetpoint(const ControllerSetpoint & setpoint, double time_s);

  // Computes the command for the current state. Zero command (timed_out = true) before
  // the first setpoint and when the latest one is older than setpoint_timeout.
  ControllerCommand update(const ControllerState & state, double time_s);

  // Zero command; also clears the integrator. For stale odometry.
  ControllerCommand hold();

  // Clears the integrator (call on a mission mode change)
  void resetIntegrator();

  // Forgets the setpoint and the integrator
  void reset();

  const Eigen::Vector3d & integral() const {return integral_;}
  bool hasSetpoint() const {return setpoint_.has_value();}

private:
  Eigen::Vector3d saturate(const Eigen::Vector3d & velocity) const;

  TrajectoryControllerParams params_;
  std::optional<ControllerSetpoint> setpoint_;
  double setpoint_time_ = 0.0;
  double last_update_time_ = 0.0;
  bool has_last_update_ = false;
  Eigen::Vector3d integral_ = Eigen::Vector3d::Zero();  // m/s
};

}  // namespace control
}  // namespace interceptor
