#pragma once

#include <Eigen/Dense>

namespace interceptor
{

/**
 * @brief Base class for EKF motion models
 */
class MotionModel
{
public:
  virtual ~MotionModel() = default;
  virtual void predict(Eigen::VectorXd & x, Eigen::MatrixXd & P, double dt) = 0;
  virtual void update(
    Eigen::VectorXd & x, Eigen::MatrixXd & P, const Eigen::Vector3d & z,
    double R_noise) = 0;
  virtual double calculateLikelihood(
    const Eigen::VectorXd & x, const Eigen::MatrixXd & P,
    const Eigen::Vector3d & z, double R_noise) = 0;
};

/**
 * @brief Constant Velocity (CV) Model
 * State: [px, py, pz, vx, vy, vz]^T (6D)
 */
class CVModel : public MotionModel
{
public:
  CVModel(double q_pos, double q_vel)
  : q_pos_(q_pos), q_vel_(q_vel) {}

  void predict(Eigen::VectorXd & x, Eigen::MatrixXd & P, double dt) override
  {
    // State transition matrix F
    Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
    F(0, 3) = F(1, 4) = F(2, 5) = dt;

    // Process noise matrix Q
    Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();
    double dt2 = dt * dt;
    double dt3 = dt2 * dt / 3.0;
    double dt4 = dt2 * dt2 / 4.0;

    // Piecewise white noise model for Q
    Q.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * (q_pos_ * dt4);
    Q.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * (q_pos_ * dt3);
    Q.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity() * (q_pos_ * dt3);
    Q.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * (q_vel_ * dt2);

    x = F * x;
    P = F * P * F.transpose() + Q;
  }

  void update(
    Eigen::VectorXd & x, Eigen::MatrixXd & P, const Eigen::Vector3d & z,
    double R_noise) override
  {
    // Measurement matrix H (we only measure position)
    Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero();
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * R_noise;
    Eigen::Vector3d y = z - H * x;     // Innovation
    Eigen::Matrix3d S = H * P * H.transpose() + R;     // Innovation covariance
    Eigen::Matrix<double, 6, 3> K = P * H.transpose() * S.inverse();     // Kalman gain

    x = x + K * y;
    P = (Eigen::Matrix<double, 6, 6>::Identity() - K * H) * P;
  }

  double calculateLikelihood(
    const Eigen::VectorXd & x, const Eigen::MatrixXd & P,
    const Eigen::Vector3d & z, double R_noise) override
  {
    Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero();
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

    Eigen::Vector3d y = z - H * x;
    Eigen::Matrix3d S = H * P * H.transpose() + Eigen::Matrix3d::Identity() * R_noise;

    double detS = S.determinant();
    if (detS < 1e-9) {detS = 1e-9;}

    double exponent = -0.5 * y.transpose() * S.inverse() * y;
    return (1.0 / std::sqrt(std::pow(2 * M_PI, 3) * detS)) * std::exp(exponent);
  }

private:
  double q_pos_, q_vel_;
};

/**
 * @brief Constant Acceleration (CA) Model
 * State: [px, py, pz, vx, vy, vz, ax, ay, az]^T (9D)
 */
class CAModel : public MotionModel
{
public:
  CAModel(double q_pos, double q_vel, double q_acc)
  : q_pos_(q_pos), q_vel_(q_vel), q_acc_(q_acc) {}

  void predict(Eigen::VectorXd & x, Eigen::MatrixXd & P, double dt) override
  {
    // State transition matrix F
    Eigen::Matrix<double, 9, 9> F = Eigen::Matrix<double, 9, 9>::Identity();
    double dt2 = 0.5 * dt * dt;
    F(0, 3) = F(1, 4) = F(2, 5) = dt;
    F(0, 6) = F(1, 7) = F(2, 8) = dt2;
    F(3, 6) = F(4, 7) = F(5, 8) = dt;

    // Simplified process noise Q
    Eigen::Matrix<double, 9, 9> Q = Eigen::Matrix<double, 9, 9>::Identity();
    Q.block<3, 3>(0, 0) *= q_pos_;
    Q.block<3, 3>(3, 3) *= q_vel_;
    Q.block<3, 3>(6, 6) *= q_acc_;
    Q *= dt;

    x = F * x;
    P = F * P * F.transpose() + Q;
  }

  void update(
    Eigen::VectorXd & x, Eigen::MatrixXd & P, const Eigen::Vector3d & z,
    double R_noise) override
  {
    Eigen::Matrix<double, 3, 9> H = Eigen::Matrix<double, 3, 9>::Zero();
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * R_noise;
    Eigen::Vector3d y = z - H * x;
    Eigen::Matrix3d S = H * P * H.transpose() + R;
    Eigen::Matrix<double, 9, 3> K = P * H.transpose() * S.inverse();

    x = x + K * y;
    P = (Eigen::Matrix<double, 9, 9>::Identity() - K * H) * P;
  }

  double calculateLikelihood(
    const Eigen::VectorXd & x, const Eigen::MatrixXd & P,
    const Eigen::Vector3d & z, double R_noise) override
  {
    Eigen::Matrix<double, 3, 9> H = Eigen::Matrix<double, 3, 9>::Zero();
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

    Eigen::Vector3d y = z - H * x;
    Eigen::Matrix3d S = H * P * H.transpose() + Eigen::Matrix3d::Identity() * R_noise;

    double detS = S.determinant();
    if (detS < 1e-9) {detS = 1e-9;}

    double exponent = -0.5 * y.transpose() * S.inverse() * y;
    return (1.0 / std::sqrt(std::pow(2 * M_PI, 3) * detS)) * std::exp(exponent);
  }

private:
  double q_pos_, q_vel_, q_acc_;
};

}  // namespace interceptor
