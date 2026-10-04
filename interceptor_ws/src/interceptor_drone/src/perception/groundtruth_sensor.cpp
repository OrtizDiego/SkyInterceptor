#include "perception/groundtruth_sensor.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace interceptor
{

std::vector<OcclusionCylinder> parseOcclusionCylinders(const std::vector<double> & flat)
{
  if (flat.size() % 4 != 0) {
    throw std::invalid_argument(
            "occlusion cylinders must be [x, y, radius, height] quadruples, got " +
            std::to_string(flat.size()) + " values");
  }
  std::vector<OcclusionCylinder> cylinders;
  cylinders.reserve(flat.size() / 4);
  for (std::size_t i = 0; i < flat.size(); i += 4) {
    OcclusionCylinder cylinder;
    cylinder.center = Eigen::Vector2d(flat[i], flat[i + 1]);
    cylinder.radius = flat[i + 2];
    cylinder.height = flat[i + 3];
    if (!(cylinder.radius > 0.0) || !(cylinder.height > 0.0)) {
      throw std::invalid_argument(
              "occlusion cylinder " + std::to_string(i / 4) +
              " needs a positive radius and height");
    }
    cylinders.push_back(cylinder);
  }
  return cylinders;
}

bool segmentIntersectsCylinder(
  const Eigen::Vector3d & from, const Eigen::Vector3d & to, const OcclusionCylinder & cylinder)
{
  // Points on the segment are from + t * d, t in [0, 1]. First find the t interval whose
  // xy projection lies inside the circle, then check the heights over that interval.
  const Eigen::Vector3d d = to - from;
  const Eigen::Vector2d f = from.head<2>() - cylinder.center;
  const Eigen::Vector2d d_xy = d.head<2>();
  const double a = d_xy.squaredNorm();
  const double b = 2.0 * f.dot(d_xy);
  const double c = f.squaredNorm() - cylinder.radius * cylinder.radius;

  double t_lo = 0.0;
  double t_hi = 1.0;
  if (a < 1e-12) {
    // Vertical segment (or a point): inside the circle everywhere or nowhere
    if (c > 0.0) {
      return false;
    }
  } else {
    const double discriminant = b * b - 4.0 * a * c;
    if (discriminant < 0.0) {
      return false;
    }
    const double root = std::sqrt(discriminant);
    t_lo = std::max(t_lo, (-b - root) / (2.0 * a));
    t_hi = std::min(t_hi, (-b + root) / (2.0 * a));
    if (t_lo > t_hi) {
      return false;
    }
  }

  // z is linear in t, so its extremes over [t_lo, t_hi] are at the interval ends
  const double z_lo = from.z() + d.z() * t_lo;
  const double z_hi = from.z() + d.z() * t_hi;
  return std::max(z_lo, z_hi) >= 0.0 && std::min(z_lo, z_hi) <= cylinder.height;
}

bool isLineOfSightBlocked(
  const Eigen::Vector3d & observer, const Eigen::Vector3d & target,
  const std::vector<OcclusionCylinder> & cylinders)
{
  return std::any_of(
    cylinders.begin(), cylinders.end(), [&](const OcclusionCylinder & cylinder) {
      return segmentIntersectsCylinder(observer, target, cylinder);
    });
}

namespace
{

const GroundTruthSensorConfig & validated(const GroundTruthSensorConfig & config)
{
  if (!(config.pos_noise_std >= 0.0)) {
    throw std::invalid_argument("pos_noise_std must be >= 0");
  }
  if (!(config.dropout_prob >= 0.0 && config.dropout_prob <= 1.0)) {
    throw std::invalid_argument("dropout_prob must be in [0, 1]");
  }
  return config;
}

}  // namespace

GroundTruthSensor::GroundTruthSensor(const GroundTruthSensorConfig & config, uint64_t seed)
: config_(validated(config)),
  rng_(seed),
  dropout_(config.dropout_prob)
{
}

std::optional<Eigen::Vector3d> GroundTruthSensor::measure(const Eigen::Vector3d & true_position)
{
  if (dropout_(rng_)) {
    return std::nullopt;
  }
  const Eigen::Vector3d unit_noise(noise_(rng_), noise_(rng_), noise_(rng_));
  return true_position + config_.pos_noise_std * unit_noise;
}

}  // namespace interceptor
