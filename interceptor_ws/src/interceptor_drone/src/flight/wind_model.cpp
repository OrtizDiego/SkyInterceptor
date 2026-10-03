#include "flight/wind_model.hpp"

#include <cmath>

namespace interceptor
{
namespace flight
{

WindModel::WindModel(const WindParams & params)
: params_(params), rng_(params.seed)
{
}

Eigen::Vector3d WindModel::update(double dt)
{
  if (dt <= 0.0 || params_.gust_stddev <= 0.0 || params_.gust_time_constant <= 0.0) {
    if (params_.gust_stddev <= 0.0) {
      gust_.setZero();
    }
    return velocity();
  }
  // Exact discretisation of dg = -g/tau dt + sigma sqrt(2/tau) dW
  const double a = std::exp(-dt / params_.gust_time_constant);
  const double b = std::sqrt(1.0 - a * a);
  const Eigen::Vector3d sigma(
    params_.gust_stddev, params_.gust_stddev,
    params_.gust_stddev * params_.vertical_gust_ratio);
  for (int k = 0; k < 3; ++k) {
    gust_[k] = a * gust_[k] + b * sigma[k] * normal_(rng_);
  }
  return velocity();
}

void WindModel::reset()
{
  gust_.setZero();
  rng_.seed(params_.seed);
  normal_.reset();
}

}  // namespace flight
}  // namespace interceptor
