#include "estimation/imm_filter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace interceptor
{
namespace estimation
{
namespace
{

constexpr double kProbabilityTolerance = 1e-6;
constexpr double kLog2Pi = 1.8378770664093453;  // log(2 pi)

void symmetrize(StateMatrix & P)
{
  P = 0.5 * (P + P.transpose()).eval();
}

ImmParams twoModelParams(ModelType second)
{
  ImmParams params;
  params.models = {ModelType::CV, second};
  params.transition.resize(2, 2);
  // Per second: CV holds ~10 s on average, a maneuver ~4.5 s
  params.transition << 0.90, 0.10,
    0.20, 0.80;
  params.transition_interval = 1.0;
  return params;
}

}  // namespace

ImmParams defaultGroundImmParams()
{
  return twoModelParams(ModelType::CA);
}

ImmParams defaultAerialImmParams()
{
  ImmParams params = twoModelParams(ModelType::CT);
  params.prior.velocity = 10.0;  // m/s, drones are fast: a tighter prior loses new tracks
  return params;
}

ImmParams makeImmParams(
  const std::vector<std::string> & model_names, const std::vector<double> & transition_row_major,
  const ProcessNoise & noise, const InitialUncertainty & prior)
{
  ImmParams params;
  params.noise = noise;
  params.prior = prior;
  for (const auto & name : model_names) {
    const auto type = modelTypeFromString(name);
    if (!type) {
      throw std::invalid_argument("unknown IMM motion model '" + name + "' (cv, ca, ct)");
    }
    params.models.push_back(*type);
  }
  const auto n = static_cast<Eigen::Index>(params.models.size());
  if (static_cast<Eigen::Index>(transition_row_major.size()) != n * n) {
    throw std::invalid_argument(
            "IMM transition matrix has " + std::to_string(transition_row_major.size()) +
            " entries, expected " + std::to_string(n * n) + " for " + std::to_string(n) +
            " models");
  }
  params.transition.resize(n, n);
  for (Eigen::Index i = 0; i < n; ++i) {
    for (Eigen::Index j = 0; j < n; ++j) {
      params.transition(i, j) = transition_row_major[static_cast<size_t>(i * n + j)];
    }
  }
  validate(params);
  return params;
}

void validate(const ImmParams & params)
{
  const auto n = static_cast<Eigen::Index>(params.models.size());
  if (n == 0) {
    throw std::invalid_argument("IMM needs at least one motion model");
  }
  if (params.transition.rows() != n || params.transition.cols() != n) {
    throw std::invalid_argument("IMM transition matrix must be n x n for n models");
  }
  for (Eigen::Index i = 0; i < n; ++i) {
    if ((params.transition.row(i).array() < 0.0).any() ||
      std::abs(params.transition.row(i).sum() - 1.0) > kProbabilityTolerance)
    {
      throw std::invalid_argument(
              "IMM transition matrix row " + std::to_string(i) +
              " must be non-negative and sum to 1");
    }
  }
  if (params.initial_probabilities.size() != 0) {
    if (params.initial_probabilities.size() != n ||
      (params.initial_probabilities.array() < 0.0).any() ||
      std::abs(params.initial_probabilities.sum() - 1.0) > kProbabilityTolerance)
    {
      throw std::invalid_argument(
              "IMM initial probabilities must have one non-negative entry per model and sum to 1");
    }
  }
  if (params.transition_interval < 0.0) {
    throw std::invalid_argument("IMM transition interval must be non-negative");
  }
  if (params.noise.acc < 0.0 || params.noise.jerk < 0.0 || params.noise.turn_rate < 0.0) {
    throw std::invalid_argument("IMM process noise must be non-negative");
  }
  if (params.prior.velocity < 0.0 || params.prior.acceleration < 0.0 ||
    params.prior.turn_rate < 0.0)
  {
    throw std::invalid_argument("IMM prior standard deviations must be non-negative");
  }
}

ImmFilter::ImmFilter(const ImmParams & params)
: params_(params)
{
  validate(params_);
  for (const auto type : params_.models) {
    models_.push_back(makeMotionModel(type, params_.noise));
  }
  const auto n = static_cast<Eigen::Index>(models_.size());
  x_.assign(models_.size(), StateVector::Zero());
  P_.assign(models_.size(), StateMatrix::Zero());
  probabilities_ = params_.initial_probabilities.size() == n ?
    params_.initial_probabilities :
    Eigen::VectorXd::Constant(n, 1.0 / static_cast<double>(n));
  estimate_.model_probabilities = probabilities_;
}

void ImmFilter::initialize(const Eigen::Vector3d & z, const Eigen::Matrix3d & R)
{
  for (size_t j = 0; j < models_.size(); ++j) {
    models_[j]->initialize(z, R, params_.prior, x_[j], P_[j]);
  }
  const auto n = static_cast<Eigen::Index>(models_.size());
  probabilities_ = params_.initial_probabilities.size() == n ?
    params_.initial_probabilities :
    Eigen::VectorXd::Constant(n, 1.0 / static_cast<double>(n));
  estimate_ = combine(x_, P_);
  initialized_ = true;
}

void ImmFilter::predict(double dt)
{
  if (!initialized_) {
    return;
  }
  const size_t n = models_.size();
  const Eigen::MatrixXd pi = transitionMatrix(dt);

  // Predicted model probabilities c_j = sum_i p_ij mu_i
  const Eigen::VectorXd c = pi.transpose() * probabilities_;

  // Interaction: model j starts from the mixture of all model estimates,
  // weighted by mu_i|j = p_ij mu_i / c_j.
  //
  // Unequal state dimensions: a source model i that does not use a component
  // model j needs (e.g. omega for CV -> CT) is augmented with model j's own
  // estimate of it, mean and variance, instead of its zero padding. Mixing in
  // the padded zeros would pull CT's turn rate and CA's acceleration towards
  // zero by mu_CV|j every cycle, which at 30 Hz biases them badly (Granstrom,
  // Willett and Bar-Shalom, "Systematic approach to IMM mixing for unequal
  // dimension states", IEEE TAES 2015).
  std::vector<StateVector> x0(n);
  std::vector<StateMatrix> P0(n);
  std::vector<StateVector> xi(n);
  std::vector<StateMatrix> Pi(n);
  for (size_t j = 0; j < n; ++j) {
    const auto jj = static_cast<Eigen::Index>(j);
    if (c(jj) <= std::numeric_limits<double>::min()) {
      // Model unreachable this cycle: nothing to mix into it
      x0[j] = x_[j];
      P0[j] = P_[j];
      continue;
    }
    const StateMask active_j = models_[j]->activeStates();
    for (size_t i = 0; i < n; ++i) {
      xi[i] = x_[i];
      Pi[i] = P_[i];
      if (i == j) {
        continue;
      }
      const StateMask active_i = models_[i]->activeStates();
      for (int k = 0; k < kStateDim; ++k) {
        if (active_j[k] && !active_i[k]) {
          xi[i](k) = x_[j](k);
          for (int l = 0; l < kStateDim; ++l) {
            // Covariance of the borrowed components among themselves; no
            // correlation with the source model's own components
            const bool borrowed_l = active_j[l] && !active_i[l];
            Pi[i](k, l) = borrowed_l ? P_[j](k, l) : 0.0;
            Pi[i](l, k) = Pi[i](k, l);
          }
        }
      }
    }
    x0[j].setZero();
    for (size_t i = 0; i < n; ++i) {
      const auto ii = static_cast<Eigen::Index>(i);
      x0[j] += (pi(ii, jj) * probabilities_(ii) / c(jj)) * xi[i];
    }
    P0[j].setZero();
    for (size_t i = 0; i < n; ++i) {
      const auto ii = static_cast<Eigen::Index>(i);
      const StateVector d = xi[i] - x0[j];
      P0[j] += (pi(ii, jj) * probabilities_(ii) / c(jj)) * (Pi[i] + d * d.transpose());
    }
    symmetrize(P0[j]);
  }

  // Model-conditioned prediction (re-applies each model's padding)
  for (size_t j = 0; j < n; ++j) {
    models_[j]->predict(x0[j], P0[j], dt);
    x_[j] = x0[j];
    P_[j] = P0[j];
  }
  probabilities_ = c / c.sum();
  estimate_ = combine(x_, P_);
}

Eigen::MatrixXd ImmFilter::transitionMatrix(double dt) const
{
  const Eigen::MatrixXd & base = params_.transition;
  if (params_.transition_interval <= 0.0) {
    return base;
  }
  const double steps = std::max(0.0, dt) / params_.transition_interval;
  Eigen::MatrixXd pi = base;
  for (Eigen::Index i = 0; i < base.rows(); ++i) {
    const double p_stay = base(i, i);
    if (p_stay >= 1.0) {
      continue;
    }
    const double stay = std::pow(p_stay, steps);
    pi.row(i) *= (1.0 - stay) / (1.0 - p_stay);
    pi(i, i) = stay;
  }
  return pi;
}

bool ImmFilter::update(const Eigen::Vector3d & z, const Eigen::Matrix3d & R)
{
  if (!initialized_) {
    return false;
  }
  const size_t n = models_.size();
  std::vector<StateVector> x = x_;
  std::vector<StateMatrix> P = P_;

  // log(mu_j L_j), normalised with log-sum-exp so tiny likelihoods don't underflow
  Eigen::VectorXd log_weight(static_cast<Eigen::Index>(n));
  double max_log_weight = -std::numeric_limits<double>::infinity();
  for (size_t j = 0; j < n; ++j) {
    const auto jj = static_cast<Eigen::Index>(j);
    const UpdateResult result = positionUpdate(x[j], P[j], z, R);
    const double prior = probabilities_(jj);
    log_weight(jj) = (result.ok && prior > 0.0) ?
      std::log(prior) + result.log_likelihood :
      -std::numeric_limits<double>::infinity();
    if (!result.ok) {
      // Keep the predicted model state
      x[j] = x_[j];
      P[j] = P_[j];
    }
    max_log_weight = std::max(max_log_weight, log_weight(jj));
  }
  if (!std::isfinite(max_log_weight)) {
    return false;
  }

  const Eigen::VectorXd weight = (log_weight.array() - max_log_weight).exp().matrix();
  probabilities_ = weight / weight.sum();
  x_ = std::move(x);
  P_ = std::move(P);
  estimate_ = combine(x_, P_);
  return true;
}

ImmEstimate ImmFilter::extrapolate(double dt) const
{
  if (!initialized_ || dt <= 0.0) {
    return estimate_;
  }
  std::vector<StateVector> x = x_;
  std::vector<StateMatrix> P = P_;
  for (size_t j = 0; j < models_.size(); ++j) {
    models_[j]->predict(x[j], P[j], dt);
  }
  return combine(x, P);
}

double ImmFilter::mahalanobis2(const Eigen::Vector3d & z, const Eigen::Matrix3d & R) const
{
  double m2 = std::numeric_limits<double>::infinity();
  double normalized = 0.0;
  gatingDistance(z, R, m2, normalized);
  return m2;
}

bool ImmFilter::gatingDistance(
  const Eigen::Vector3d & z, const Eigen::Matrix3d & R, double & mahalanobis2,
  double & normalized_distance) const
{
  const Eigen::Vector3d y = z - estimate_.position();
  const Eigen::Matrix3d S = estimate_.positionCovariance() + R;
  double log_likelihood = 0.0;
  if (!innovationStatistics(y, S, mahalanobis2, log_likelihood)) {
    mahalanobis2 = std::numeric_limits<double>::infinity();
    normalized_distance = std::numeric_limits<double>::infinity();
    return false;
  }
  // -2 log N(y; 0, S) - 3 log(2 pi) = d^2 + log det S
  normalized_distance = -2.0 * log_likelihood - 3.0 * kLog2Pi;
  return true;
}

ImmEstimate ImmFilter::combine(
  const std::vector<StateVector> & x, const std::vector<StateMatrix> & P) const
{
  ImmEstimate out;
  out.model_probabilities = probabilities_;
  for (size_t j = 0; j < x.size(); ++j) {
    const double mu = probabilities_(static_cast<Eigen::Index>(j));
    out.state += mu * x[j];
    out.acceleration += mu * models_[j]->acceleration(x[j]);
  }
  for (size_t j = 0; j < x.size(); ++j) {
    const double mu = probabilities_(static_cast<Eigen::Index>(j));
    const StateVector d = x[j] - out.state;
    out.covariance += mu * (P[j] + d * d.transpose());
  }
  symmetrize(out.covariance);
  return out;
}

}  // namespace estimation
}  // namespace interceptor
