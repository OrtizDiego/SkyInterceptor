#pragma once

#include <Eigen/Dense>

#include <cmath>
#include <random>
#include <vector>

namespace interceptor
{
namespace testing
{

struct TruthSample
{
  double t;
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
};

// A piece of horizontal motion: acceleration along the velocity and turn rate
// of the velocity vector, held for duration seconds. Vertical velocity is constant.
struct Segment
{
  double duration;
  double tangential_accel = 0.0;  // m/s^2
  double turn_rate = 0.0;         // rad/s
};

// Samples a piecewise trajectory at rate_hz, starting at t = 0. Integrated with
// 1 ms sub-steps, so it is exact to well below the measurement noise.
inline std::vector<TruthSample> generateTrajectory(
  const Eigen::Vector3d & p0, const Eigen::Vector3d & v0, const std::vector<Segment> & segments,
  double rate_hz)
{
  constexpr double kSubStep = 1e-3;
  std::vector<TruthSample> out;
  Eigen::Vector3d p = p0;
  Eigen::Vector3d v = v0;
  double t = 0.0;
  double next_sample = 0.0;
  const double period = 1.0 / rate_hz;

  for (const auto & segment : segments) {
    const int steps = static_cast<int>(std::round(segment.duration / kSubStep));
    for (int k = 0; k < steps; ++k) {
      if (t >= next_sample - 1e-9) {
        out.push_back({next_sample, p, v});
        next_sample += period;
      }
      const double speed = v.head<2>().norm();
      const Eigen::Vector2d dir = speed > 1e-9 ? Eigen::Vector2d(v.head<2>() / speed) :
        Eigen::Vector2d(1.0, 0.0);
      const Eigen::Vector2d accel =
        segment.tangential_accel * dir +
        segment.turn_rate * speed * Eigen::Vector2d(-dir.y(), dir.x());
      // Semi-implicit midpoint step
      const Eigen::Vector2d v_next = v.head<2>() + accel * kSubStep;
      p.head<2>() += 0.5 * (v.head<2>() + v_next) * kSubStep;
      p.z() += v.z() * kSubStep;
      v.head<2>() = v_next;
      t += kSubStep;
    }
  }
  return out;
}

class MeasurementNoise
{
public:
  MeasurementNoise(double stddev, unsigned seed)
  : stddev_(stddev), rng_(seed), normal_(0.0, 1.0) {}

  Eigen::Vector3d sample(const Eigen::Vector3d & truth)
  {
    return truth + stddev_ * Eigen::Vector3d(normal_(rng_), normal_(rng_), normal_(rng_));
  }

  Eigen::Matrix3d covariance() const {return Eigen::Matrix3d::Identity() * stddev_ * stddev_;}

private:
  double stddev_;
  std::mt19937 rng_;
  std::normal_distribution<double> normal_;
};

}  // namespace testing
}  // namespace interceptor
