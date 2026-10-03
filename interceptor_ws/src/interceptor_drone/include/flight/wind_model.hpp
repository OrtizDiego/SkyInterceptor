#pragma once

#include <Eigen/Dense>

#include <random>

namespace interceptor
{
namespace flight
{

struct WindParams
{
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();  // steady wind, world frame [m/s]
  double gust_stddev = 0.0;          // horizontal turbulence intensity [m/s]
  double vertical_gust_ratio = 0.5;  // vertical intensity relative to horizontal
  double gust_time_constant = 2.0;   // correlation time [s]
  unsigned int seed = 42;
};

// Steady wind plus turbulence. Each gust component is a first-order Gauss-Markov
// process (a simplified Dryden model) discretised exactly, so its stationary standard
// deviation is gust_stddev for any time step.
class WindModel
{
public:
  explicit WindModel(const WindParams & params = WindParams());

  void setMean(const Eigen::Vector3d & mean) {params_.mean = mean;}
  void setGustStddev(double stddev) {params_.gust_stddev = stddev < 0.0 ? 0.0 : stddev;}
  const WindParams & params() const {return params_;}

  // Advance the turbulence state by dt seconds and return the wind velocity.
  Eigen::Vector3d update(double dt);
  Eigen::Vector3d velocity() const {return params_.mean + gust_;}
  const Eigen::Vector3d & gust() const {return gust_;}

  void reset();

private:
  WindParams params_;
  Eigen::Vector3d gust_ = Eigen::Vector3d::Zero();
  std::mt19937 rng_;
  std::normal_distribution<double> normal_{0.0, 1.0};
};

}  // namespace flight
}  // namespace interceptor
