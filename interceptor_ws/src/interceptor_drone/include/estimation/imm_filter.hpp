#pragma once

#include <Eigen/Dense>

#include <memory>
#include <string>
#include <vector>

#include "estimation/ekf_models.hpp"

namespace interceptor
{
namespace estimation
{

struct ImmParams
{
  std::vector<ModelType> models;
  // Markov model transition matrix, transition(i, j) = P(model j | model i)
  // over transition_interval seconds. Rows sum to 1.
  Eigen::MatrixXd transition;
  // Time the transition matrix refers to [s]. predict(dt) rescales it to dt:
  // the probability of staying in model i becomes p_ii^(dt / interval) (a
  // constant switching rate) and the off-diagonal entries keep their ratios.
  // This keeps the mode dynamics independent of the sensor rate and makes a
  // long gap mix more than a single frame. 0 applies the matrix as is, once
  // per predict() call.
  double transition_interval = 1.0;
  // Model probabilities of a new track. Empty: uniform.
  Eigen::VectorXd initial_probabilities;
  ProcessNoise noise;
  InitialUncertainty prior;
};

// FOLLOW / ground targets: CV + CA
ImmParams defaultGroundImmParams();
// INTERCEPT / aerial targets: CV + CT
ImmParams defaultAerialImmParams();

// Builds the model set and transition matrix from parameter values, e.g.
// models {"cv", "ca"} and transition {0.95, 0.05, 0.10, 0.90} (row-major).
// Throws std::invalid_argument on unknown model names or a bad matrix.
ImmParams makeImmParams(
  const std::vector<std::string> & model_names, const std::vector<double> & transition_row_major,
  const ProcessNoise & noise = ProcessNoise(),
  const InitialUncertainty & prior = InitialUncertainty());

// Throws std::invalid_argument if the parameters are inconsistent
void validate(const ImmParams & params);

// Combined (moment-matched) IMM estimate
struct ImmEstimate
{
  StateVector state = StateVector::Zero();
  StateMatrix covariance = StateMatrix::Zero();
  // Probability-weighted acceleration implied by each model (CA: a, CT: omega x v, CV: 0).
  // This is not state.segment<3>(kAcc), which only holds the CA contribution.
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
  Eigen::VectorXd model_probabilities;

  Eigen::Vector3d position() const {return state.segment<3>(kPos);}
  Eigen::Vector3d velocity() const {return state.segment<3>(kVel);}
  double turnRate() const {return state(kOmega);}
  Eigen::Matrix3d positionCovariance() const {return covariance.block<3, 3>(kPos, kPos);}
  Eigen::Matrix3d velocityCovariance() const {return covariance.block<3, 3>(kVel, kVel);}
};

// Interacting Multiple Model filter (Blom & Bar-Shalom) over EKF motion models
// that share the padded state layout of ekf_models.hpp. One filter cycle is
//   predict(dt): interaction (mixing) + model-conditioned prediction
//   update(z, R): model-conditioned update + mode probability update + combination
// Between measurements, extrapolate() predicts the combined estimate without
// changing the filter, e.g. to publish at a higher rate than the sensor.
class ImmFilter
{
public:
  explicit ImmFilter(const ImmParams & params);

  // Starts every model at the position fix z (covariance R) with the prior
  // velocity/acceleration/turn-rate uncertainty and the initial model probabilities
  void initialize(const Eigen::Vector3d & z, const Eigen::Matrix3d & R);

  // Mixing, then each model predicts by dt [s]. Model probabilities become the
  // predicted ones, c_j = sum_i p_ij mu_i.
  void predict(double dt);

  // Measurement update with a 3D position z and covariance R. Returns false
  // (and leaves the predicted estimate) if no model could process it.
  bool update(const Eigen::Vector3d & z, const Eigen::Matrix3d & R);

  // Combined estimate after the last predict()/update()
  const ImmEstimate & estimate() const {return estimate_;}

  // Combined estimate predicted dt [s] ahead, at the current model probabilities
  ImmEstimate extrapolate(double dt) const;

  // Squared Mahalanobis distance of z from the combined estimate, S = P_pos + R
  double mahalanobis2(const Eigen::Vector3d & z, const Eigen::Matrix3d & R) const;
  // Same, plus log det(S): the normalised distance used for nearest-neighbour
  // association, which penalises uncertain tracks
  bool gatingDistance(
    const Eigen::Vector3d & z, const Eigen::Matrix3d & R, double & mahalanobis2,
    double & normalized_distance) const;

  // Transition matrix over dt seconds (see ImmParams::transition_interval)
  Eigen::MatrixXd transitionMatrix(double dt) const;

  bool initialized() const {return initialized_;}
  size_t numModels() const {return models_.size();}
  ModelType modelType(size_t i) const {return models_[i]->type();}
  const Eigen::VectorXd & modelProbabilities() const {return probabilities_;}
  const StateVector & modelState(size_t i) const {return x_[i];}
  const StateMatrix & modelCovariance(size_t i) const {return P_[i];}

private:
  ImmEstimate combine(
    const std::vector<StateVector> & x, const std::vector<StateMatrix> & P) const;

  ImmParams params_;
  // Models are immutable, so copies of a filter share them
  std::vector<std::shared_ptr<const MotionModel>> models_;
  std::vector<StateVector> x_;
  std::vector<StateMatrix> P_;
  Eigen::VectorXd probabilities_;
  ImmEstimate estimate_;
  bool initialized_ = false;
};

}  // namespace estimation
}  // namespace interceptor
