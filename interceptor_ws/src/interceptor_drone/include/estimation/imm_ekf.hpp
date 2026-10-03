#pragma once

#include <memory>
#include <vector>

#include "estimation/ekf_models.hpp"

namespace interceptor
{

/**
 * @brief Interacting Multiple Model (IMM) Filter
 * Manages multiple motion models and blends their estimates
 */
class IMMFilter
{
public:
  IMMFilter()
  {
    // Initialize transition probability matrix (example values)
    // [ CV->CV  CV->CA ]
    // [ CA->CV  CA->CA ]
    transition_matrix_ << 0.95, 0.05,
      0.10, 0.90;

    // Initial model probabilities
    model_probs_ = {0.8, 0.2};
  }

  void addModel(std::shared_ptr<MotionModel> model)
  {
    models_.push_back(model);
  }

  void initialize(const Eigen::Vector3d & pos)
  {
    // Initialize CV model state (6D)
    x_cv_ = Eigen::VectorXd::Zero(6);
    x_cv_.segment<3>(0) = pos;
    P_cv_ = Eigen::MatrixXd::Identity(6, 6) * 1.0;

    // Initialize CA model state (9D)
    x_ca_ = Eigen::VectorXd::Zero(9);
    x_ca_.segment<3>(0) = pos;
    P_ca_ = Eigen::MatrixXd::Identity(9, 9) * 1.0;

    // Initialize combined state
    combineEstimates();

    initialized_ = true;
  }

  void predict(double dt)
  {
    if (!initialized_) {return;}

    // 1. Interaction (Mixing)
    mixStates();

    // 2. Model-specific prediction
    models_[0]->predict(x_cv_, P_cv_, dt);
    models_[1]->predict(x_ca_, P_ca_, dt);
  }

  void update(const Eigen::Vector3d & z, double R_noise)
  {
    if (!initialized_) {return;}

    // 1. Model-specific update
    models_[0]->update(x_cv_, P_cv_, z, R_noise);
    models_[1]->update(x_ca_, P_ca_, z, R_noise);

    // 2. Model probability update
    updateModelProbabilities(z, R_noise);

    // 3. Combined estimate
    combineEstimates();
  }

  bool isInitialized() const {return initialized_;}

  Eigen::Vector3d getPosition() const {return x_combined_.segment<3>(0);}
  Eigen::Vector3d getVelocity() const {return x_combined_.segment<3>(3);}
  Eigen::Vector3d getAcceleration() const {return x_combined_.segment<3>(6);}
  Eigen::Matrix3d getPositionCov() const {return P_combined_.block<3, 3>(0, 0);}
  Eigen::Matrix3d getVelocityCov() const {return P_combined_.block<3, 3>(3, 3);}

private:
  void mixStates()
  {
    // Mixing for 2 models (CV and CA)
    // c_j = sum_i(p_ij * mu_i)
    double c1 =
      transition_matrix_(0, 0) * model_probs_[0] + transition_matrix_(1, 0) * model_probs_[1];
    double c2 =
      transition_matrix_(0, 1) * model_probs_[0] + transition_matrix_(1, 1) * model_probs_[1];

    // mu_i|j = (p_ij * mu_i) / c_j
    double mu11 = (transition_matrix_(0, 0) * model_probs_[0]) / c1;
    double mu21 = (transition_matrix_(1, 0) * model_probs_[1]) / c1;
    double mu12 = (transition_matrix_(0, 1) * model_probs_[0]) / c2;
    double mu22 = (transition_matrix_(1, 1) * model_probs_[1]) / c2;

    // Mix states (simplified: CA state contains CV state)
    Eigen::VectorXd x_mixed_cv = x_cv_ * mu11 + x_ca_.segment<6>(0) * mu21;
    Eigen::VectorXd x_mixed_ca = x_ca_;
    x_mixed_ca.segment<6>(0) = x_cv_.segment<6>(0) * mu12 + x_ca_.segment<6>(0) * mu22;

    x_cv_ = x_mixed_cv;
    x_ca_ = x_mixed_ca;

    // Covariance mixing (omitted for brevity/simplicity in this prototype,
    // but normally required for full IMM)
  }

  void updateModelProbabilities(const Eigen::Vector3d & z, double R_noise)
  {
    double likelihood_cv = models_[0]->calculateLikelihood(x_cv_, P_cv_, z, R_noise);
    double likelihood_ca = models_[1]->calculateLikelihood(x_ca_, P_ca_, z, R_noise);

    // c_j = sum_i(p_ij * mu_i)
    double c1 =
      transition_matrix_(0, 0) * model_probs_[0] + transition_matrix_(1, 0) * model_probs_[1];
    double c2 =
      transition_matrix_(0, 1) * model_probs_[0] + transition_matrix_(1, 1) * model_probs_[1];

    double norm = likelihood_cv * c1 + likelihood_ca * c2;
    if (norm < 1e-9) {norm = 1e-9;}

    model_probs_[0] = (likelihood_cv * c1) / norm;
    model_probs_[1] = (likelihood_ca * c2) / norm;
  }

  void combineEstimates()
  {
    // Combine 6D CV and 9D CA into 9D combined
    x_combined_ = Eigen::VectorXd::Zero(9);
    x_combined_.segment<6>(0) = x_cv_ * model_probs_[0] + x_ca_.segment<6>(0) * model_probs_[1];
    x_combined_.segment<3>(6) = x_ca_.segment<3>(6) * model_probs_[1];

    P_combined_ = Eigen::MatrixXd::Zero(9, 9);
    P_combined_.block<6, 6>(0, 0) += P_cv_ * model_probs_[0];
    P_combined_.block<9, 9>(0, 0) += P_ca_ * model_probs_[1];
  }

  std::vector<std::shared_ptr<MotionModel>> models_;
  std::vector<double> model_probs_;
  Eigen::Matrix2d transition_matrix_;

  Eigen::VectorXd x_cv_, x_ca_, x_combined_;
  Eigen::MatrixXd P_cv_, P_ca_, P_combined_;

  bool initialized_ = false;
};

}  // namespace interceptor
