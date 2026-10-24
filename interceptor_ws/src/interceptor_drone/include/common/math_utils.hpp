#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace interceptor
{

// Skew symmetric matrix
inline Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d & v)
{
  Eigen::Matrix3d m;
  m << 0, -v.z(), v.y(),
    v.z(), 0, -v.x(),
    -v.y(), v.x(), 0;
  return m;
}

// Quaternion to rotation matrix
inline Eigen::Matrix3d quatToRot(const Eigen::Vector4d & q)
{
  double w = q(0), x = q(1), y = q(2), z = q(3);
  Eigen::Matrix3d R;
  R << 1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
    2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
    2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y);
  return R;
}

// Rotation matrix to quaternion
inline Eigen::Vector4d rotToQuat(const Eigen::Matrix3d & R)
{
  double trace = R.trace();
  Eigen::Vector4d q;

  if (trace > 0) {
    double s = 0.5 / std::sqrt(trace + 1.0);
    q(0) = 0.25 / s;
    q(1) = (R(2, 1) - R(1, 2)) * s;
    q(2) = (R(0, 2) - R(2, 0)) * s;
    q(3) = (R(1, 0) - R(0, 1)) * s;
  } else {
    // Handle other cases
    if (R(0, 0) > R(1, 1) && R(0, 0) > R(2, 2)) {
      double s = 2.0 * std::sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2));
      q(0) = (R(2, 1) - R(1, 2)) / s;
      q(1) = 0.25 * s;
      q(2) = (R(0, 1) + R(1, 0)) / s;
      q(3) = (R(0, 2) + R(2, 0)) / s;
    } else if (R(1, 1) > R(2, 2)) {
      double s = 2.0 * std::sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2));
      q(0) = (R(0, 2) - R(2, 0)) / s;
      q(1) = (R(0, 1) + R(1, 0)) / s;
      q(2) = 0.25 * s;
      q(3) = (R(1, 2) + R(2, 1)) / s;
    } else {
      double s = 2.0 * std::sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1));
      q(0) = (R(1, 0) - R(0, 1)) / s;
      q(1) = (R(0, 2) + R(2, 0)) / s;
      q(2) = (R(1, 2) + R(2, 1)) / s;
      q(3) = 0.25 * s;
    }
  }

  return q.normalized();
}

// Wraps an angle to (-pi, pi]
inline double wrapAngle(double angle)
{
  constexpr double kPi = 3.14159265358979323846;
  angle = std::fmod(angle + kPi, 2.0 * kPi);
  if (angle <= 0.0) {
    angle += 2.0 * kPi;
  }
  return angle - kPi;
}

// The rate at which the horizontal bearing from `from` to `to` turns [rad/s], from
// both positions and velocities: d/dt atan2(r_y, r_x) = (r x v_rel)_z / |r|^2, with
// r = to - from and v_rel = to_velocity - from_velocity. It is the yaw-rate
// feed-forward of a heading that follows a target: the heading loop then only has
// to close what is left, so its gain can stay low. Zero closer than min_range,
// where the bearing is ill-defined and its rate unbounded.
inline double bearingRate(
  const Eigen::Vector3d & from, const Eigen::Vector3d & from_velocity,
  const Eigen::Vector3d & to, const Eigen::Vector3d & to_velocity, double min_range)
{
  const Eigen::Vector2d r = (to - from).head<2>();
  const double r2 = r.squaredNorm();
  if (r2 < min_range * min_range) {
    return 0.0;
  }
  const Eigen::Vector2d v = (to_velocity - from_velocity).head<2>();
  return (r.x() * v.y() - r.y() * v.x()) / r2;
}

// Saturate vector magnitude
inline Eigen::Vector3d saturateVector(const Eigen::Vector3d & v, double max_norm)
{
  double norm = v.norm();
  if (norm > max_norm && norm > 1e-6) {
    return v * (max_norm / norm);
  }
  return v;
}

}  // namespace interceptor
