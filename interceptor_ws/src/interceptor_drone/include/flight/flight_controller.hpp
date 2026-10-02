#pragma once

#include <Eigen/Dense>

#include "flight/quadrotor_model.hpp"

namespace interceptor
{
namespace flight
{

struct FlightControllerParams
{
  // Command limits
  double max_horizontal_speed = 8.0;  // m/s
  double max_vertical_speed = 3.0;    // m/s
  double max_yaw_rate = 1.5;          // rad/s

  // Velocity loop (output: acceleration [m/s^2])
  Eigen::Vector3d velocity_p = Eigen::Vector3d(2.0, 2.0, 3.0);
  Eigen::Vector3d velocity_i = Eigen::Vector3d(0.6, 0.6, 1.0);
  Eigen::Vector3d max_velocity_integral = Eigen::Vector3d(3.0, 3.0, 4.0);  // m/s^2
  double max_tilt = 0.61;  // rad (35 deg)
  double max_vertical_accel_up = 6.0;    // m/s^2
  double max_vertical_accel_down = 5.0;  // m/s^2

  // Attitude loop (output: body rates [rad/s])
  Eigen::Vector3d attitude_p = Eigen::Vector3d(6.0, 6.0, 3.0);
  Eigen::Vector3d max_body_rate = Eigen::Vector3d(3.5, 3.5, 2.0);
  double max_yaw_error = 0.5;  // rad, yaw setpoint is kept within this of the heading

  // Rate loop (output: angular acceleration [rad/s^2], scaled by the inertia)
  Eigen::Vector3d rate_p = Eigen::Vector3d(18.0, 18.0, 8.0);
  Eigen::Vector3d rate_i = Eigen::Vector3d(4.0, 4.0, 1.0);
  Eigen::Vector3d max_rate_integral = Eigen::Vector3d(5.0, 5.0, 2.0);  // rad/s^2

  // Rotor speed while armed with no thrust demand
  double idle_rotor_speed = 100.0;  // rad/s

  // Landed state: after arming the rotors idle until a climb is commanded, and
  // touchdown is detected when descending with low thrust and no motion.
  double takeoff_climb_speed = 0.1;  // m/s, climb command that starts the take-off
  double land_thrust_ratio = 0.75;   // thrust below this fraction of the weight ...
  double land_max_speed = 0.2;       // m/s ... while moving slower than this ...
  double land_detect_time = 1.0;     // s ... for this long means landed
};

// Velocity command: linear velocity in the world frame and yaw rate.
struct VelocityCommand
{
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  double yaw_rate = 0.0;
};

// Rotates a heading-frame velocity (x forward, y left, z up, level) into the world frame.
Eigen::Vector3d headingToWorld(const Eigen::Vector3d & velocity, double yaw);

// Yaw of a body -> world rotation (ZYX convention).
double yawFromQuaternion(const Eigen::Quaterniond & q);

// Onboard autopilot, PX4-style cascade:
//   velocity PI -> thrust vector -> attitude P (on SO(3)) -> body-rate PI -> mixer.
// Its output is the rotor speed command for each motor. While landed (after a reset)
// the rotors idle until a climb is commanded.
class FlightController
{
public:
  FlightController(
    const QuadrotorParams & vehicle = makeX500Params(),
    const FlightControllerParams & params = FlightControllerParams());

  // Zero the integrators, hold the current heading and enter the landed state.
  // Call on arming.
  void reset(const RigidBodyState & state);

  RotorArray update(const RigidBodyState & state, const VelocityCommand & command, double dt);

  // Control allocation: collective thrust [N] and body torque [N m] -> rotor speeds.
  // On saturation roll/pitch keep priority over collective thrust, and yaw comes last.
  RotorArray allocate(double thrust, const Eigen::Vector3d & torque) const;

  const FlightControllerParams & params() const {return params_;}

  // Telemetry from the last update
  const Eigen::Matrix3d & desiredAttitude() const {return desired_attitude_;}
  double collectiveThrust() const {return collective_thrust_;}
  double yawSetpoint() const {return yaw_setpoint_;}
  bool landed() const {return landed_;}

private:
  void buildAllocation();

  QuadrotorParams vehicle_;
  FlightControllerParams params_;
  Eigen::Matrix4d allocation_inverse_ = Eigen::Matrix4d::Identity();

  Eigen::Vector3d velocity_integral_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d rate_integral_ = Eigen::Vector3d::Zero();
  double yaw_setpoint_ = 0.0;
  Eigen::Matrix3d desired_attitude_ = Eigen::Matrix3d::Identity();
  double collective_thrust_ = 0.0;
  bool landed_ = true;
  double land_timer_ = 0.0;
};

}  // namespace flight
}  // namespace interceptor
