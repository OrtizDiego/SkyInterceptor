#pragma once

#include <Eigen/Dense>

#include <cstdint>
#include <optional>
#include <random>
#include <vector>

namespace interceptor
{

// Vertical cylinder standing on the ground (z = 0), e.g. a tree canopy that hides targets
struct OcclusionCylinder
{
  Eigen::Vector2d center = Eigen::Vector2d::Zero();  // m, world xy
  double radius = 0.0;  // m
  double height = 0.0;  // m, top of the cylinder above ground
};

// Parses the flat [x, y, radius, height, x, y, radius, height, ...] list used by the
// occlusion.cylinders parameter (ROS parameters cannot nest arrays). Throws
// std::invalid_argument if the length isn't a multiple of 4 or a radius/height is <= 0.
std::vector<OcclusionCylinder> parseOcclusionCylinders(const std::vector<double> & flat);

// True if the segment from -> to touches the cylinder (surface included)
bool segmentIntersectsCylinder(
  const Eigen::Vector3d & from, const Eigen::Vector3d & to, const OcclusionCylinder & cylinder);

// True if any cylinder blocks the straight line of sight from the observer to the target
bool isLineOfSightBlocked(
  const Eigen::Vector3d & observer, const Eigen::Vector3d & target,
  const std::vector<OcclusionCylinder> & cylinders);

struct GroundTruthSensorConfig
{
  double pos_noise_std = 0.2;  // m, per axis
  double dropout_prob = 0.0;   // probability that a detection is dropped
};

// Turns ground-truth positions into detector-like measurements: random dropout, then
// zero-mean Gaussian noise with the same standard deviation on every axis.
class GroundTruthSensor
{
public:
  // Throws std::invalid_argument if pos_noise_std < 0 or dropout_prob is outside [0, 1]
  GroundTruthSensor(const GroundTruthSensorConfig & config, uint64_t seed);

  // Noisy measurement of true_position, or nullopt if this detection is dropped
  std::optional<Eigen::Vector3d> measure(const Eigen::Vector3d & true_position);

  const GroundTruthSensorConfig & config() const {return config_;}

private:
  GroundTruthSensorConfig config_;
  std::mt19937_64 rng_;
  std::normal_distribution<double> noise_;  // standard normal, scaled by pos_noise_std
  std::bernoulli_distribution dropout_;
};

}  // namespace interceptor
