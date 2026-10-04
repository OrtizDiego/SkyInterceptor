#include "estimation/ekf_models.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <initializer_list>
#include <stdexcept>

namespace interceptor
{
namespace estimation
{
namespace
{

constexpr double kLog2Pi = 1.8378770664093453;  // log(2 pi)

// Below this |omega * dt| the CT model uses series expansions instead of
// dividing by omega
constexpr double kSmallTurnAngle = 1e-3;

void symmetrize(StateMatrix & P)
{
  P = 0.5 * (P + P.transpose()).eval();
}

// Per-axis CV block of Q for white acceleration q on (p, v) of every axis
void addWhiteAccelerationNoise(StateMatrix & Q, double q, double dt)
{
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  for (int i = 0; i < 3; ++i) {
    Q(kPos + i, kPos + i) += q * dt3 / 3.0;
    Q(kPos + i, kVel + i) += q * dt2 / 2.0;
    Q(kVel + i, kPos + i) += q * dt2 / 2.0;
    Q(kVel + i, kVel + i) += q * dt;
  }
}

StateMask mask(std::initializer_list<int> blocks_of_three, bool omega)
{
  StateMask m{};
  m.fill(false);
  for (const int start : blocks_of_three) {
    for (int i = 0; i < 3; ++i) {
      m[start + i] = true;
    }
  }
  m[kOmega] = omega;
  return m;
}

}  // namespace

std::optional<ModelType> modelTypeFromString(const std::string & name)
{
  std::string lower(name);
  std::transform(
    lower.begin(), lower.end(), lower.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  if (lower == "cv") {
    return ModelType::CV;
  }
  if (lower == "ca") {
    return ModelType::CA;
  }
  if (lower == "ct") {
    return ModelType::CT;
  }
  return std::nullopt;
}

std::string toString(ModelType type)
{
  switch (type) {
    case ModelType::CV:
      return "cv";
    case ModelType::CA:
      return "ca";
    case ModelType::CT:
      return "ct";
  }
  return "unknown";
}

bool innovationStatistics(
  const Eigen::Vector3d & innovation, const Eigen::Matrix3d & innovation_cov,
  double & mahalanobis2, double & log_likelihood)
{
  const Eigen::LLT<Eigen::Matrix3d> llt(innovation_cov);
  if (llt.info() != Eigen::Success) {
    return false;
  }
  const Eigen::Vector3d w = llt.matrixL().solve(innovation);
  mahalanobis2 = w.squaredNorm();
  const double log_det = 2.0 * llt.matrixL().toDenseMatrix().diagonal().array().log().sum();
  log_likelihood = -0.5 * (mahalanobis2 + log_det + 3.0 * kLog2Pi);
  return true;
}

UpdateResult positionUpdate(
  StateVector & x, StateMatrix & P, const Eigen::Vector3d & z, const Eigen::Matrix3d & R)
{
  UpdateResult result;
  result.innovation = z - x.segment<3>(kPos);
  result.innovation_cov = P.block<3, 3>(kPos, kPos) + R;
  if (!innovationStatistics(
      result.innovation, result.innovation_cov, result.mahalanobis2, result.log_likelihood))
  {
    return result;
  }

  // K = P H' S^-1 with H = [I3 0]
  const Eigen::Matrix<double, kStateDim, 3> PHt = P.middleCols<3>(kPos);
  const Eigen::Matrix<double, kStateDim, 3> K =
    result.innovation_cov.llt().solve(PHt.transpose()).transpose();

  x += K * result.innovation;

  // Joseph form keeps P symmetric positive semi-definite
  StateMatrix I_KH = StateMatrix::Identity();
  I_KH.middleCols<3>(kPos) -= K;
  P = I_KH * P * I_KH.transpose() + K * R * K.transpose();
  symmetrize(P);

  result.ok = true;
  return result;
}

// --- MotionModel ---------------------------------------------------------------

void MotionModel::predict(StateVector & x, StateMatrix & P, double dt) const
{
  if (dt <= 0.0) {
    applyPadding(x, P);
    return;
  }
  StateMatrix F;
  const StateMatrix Q = processNoise(x, dt);
  x = transition(x, dt, F);
  P = F * P * F.transpose() + Q;
  applyPadding(x, P);
  symmetrize(P);
}

void MotionModel::applyPadding(StateVector & x, StateMatrix & P) const
{
  const StateMask active = activeStates();
  for (int i = 0; i < kStateDim; ++i) {
    if (!active[i]) {
      x(i) = 0.0;
      P.row(i).setZero();
      P.col(i).setZero();
    }
  }
}

void MotionModel::initialize(
  const Eigen::Vector3d & z, const Eigen::Matrix3d & R, const InitialUncertainty & prior,
  StateVector & x, StateMatrix & P) const
{
  x.setZero();
  x.segment<3>(kPos) = z;
  P.setZero();
  P.block<3, 3>(kPos, kPos) = R;
  P.block<3, 3>(kVel, kVel).diagonal().setConstant(prior.velocity * prior.velocity);
  P.block<3, 3>(kAcc, kAcc).diagonal().setConstant(prior.acceleration * prior.acceleration);
  P(kOmega, kOmega) = prior.turn_rate * prior.turn_rate;
  applyPadding(x, P);
}

// --- CV ----------------------------------------------------------------------------

StateMask CVModel::activeStates() const
{
  return mask({kPos, kVel}, false);
}

StateVector CVModel::transition(const StateVector & x, double dt, StateMatrix & F) const
{
  F.setIdentity();
  for (int i = 0; i < 3; ++i) {
    F(kPos + i, kVel + i) = dt;
  }
  return F * x;
}

StateMatrix CVModel::processNoise(const StateVector & /*x*/, double dt) const
{
  StateMatrix Q = StateMatrix::Zero();
  addWhiteAccelerationNoise(Q, noise_.acc, dt);
  return Q;
}

Eigen::Vector3d CVModel::acceleration(const StateVector & /*x*/) const
{
  return Eigen::Vector3d::Zero();
}

// --- CA ----------------------------------------------------------------------------

StateMask CAModel::activeStates() const
{
  return mask({kPos, kVel, kAcc}, false);
}

StateVector CAModel::transition(const StateVector & x, double dt, StateMatrix & F) const
{
  F.setIdentity();
  for (int i = 0; i < 3; ++i) {
    F(kPos + i, kVel + i) = dt;
    F(kPos + i, kAcc + i) = 0.5 * dt * dt;
    F(kVel + i, kAcc + i) = dt;
  }
  return F * x;
}

StateMatrix CAModel::processNoise(const StateVector & /*x*/, double dt) const
{
  // White jerk, per axis on (p, v, a)
  const double q = noise_.jerk;
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt3 * dt;
  const double dt5 = dt4 * dt;
  StateMatrix Q = StateMatrix::Zero();
  for (int i = 0; i < 3; ++i) {
    const int p = kPos + i;
    const int v = kVel + i;
    const int a = kAcc + i;
    Q(p, p) = q * dt5 / 20.0;
    Q(p, v) = Q(v, p) = q * dt4 / 8.0;
    Q(p, a) = Q(a, p) = q * dt3 / 6.0;
    Q(v, v) = q * dt3 / 3.0;
    Q(v, a) = Q(a, v) = q * dt2 / 2.0;
    Q(a, a) = q * dt;
  }
  return Q;
}

Eigen::Vector3d CAModel::acceleration(const StateVector & x) const
{
  return x.segment<3>(kAcc);
}

// --- CT ----------------------------------------------------------------------------

StateMask CTModel::activeStates() const
{
  return mask({kPos, kVel}, true);
}

StateVector CTModel::transition(const StateVector & x, double dt, StateMatrix & F) const
{
  const double w = x(kOmega);
  const double vx = x(kVel);
  const double vy = x(kVel + 1);
  const double a = w * dt;
  const double s = std::sin(a);
  const double c = std::cos(a);

  // A = sin(w dt) / w, B = (1 - cos(w dt)) / w and their derivatives w.r.t. w
  double A, B, dA, dB;
  if (std::abs(a) < kSmallTurnAngle) {
    const double a2 = a * a;
    A = dt * (1.0 - a2 / 6.0);
    B = dt * (a / 2.0 - a * a2 / 24.0);
    dA = dt * dt * (-a / 3.0 + a * a2 / 30.0);
    dB = dt * dt * (0.5 - a2 / 8.0);
  } else {
    A = s / w;
    B = (1.0 - c) / w;
    dA = (dt * c * w - s) / (w * w);
    dB = (dt * s * w - (1.0 - c)) / (w * w);
  }

  StateVector xn = x;
  xn(kPos) = x(kPos) + A * vx - B * vy;
  xn(kPos + 1) = x(kPos + 1) + B * vx + A * vy;
  xn(kPos + 2) = x(kPos + 2) + dt * x(kVel + 2);
  xn(kVel) = c * vx - s * vy;
  xn(kVel + 1) = s * vx + c * vy;

  F.setIdentity();
  F(kPos, kVel) = A;
  F(kPos, kVel + 1) = -B;
  F(kPos, kOmega) = dA * vx - dB * vy;
  F(kPos + 1, kVel) = B;
  F(kPos + 1, kVel + 1) = A;
  F(kPos + 1, kOmega) = dB * vx + dA * vy;
  F(kPos + 2, kVel + 2) = dt;
  F(kVel, kVel) = c;
  F(kVel, kVel + 1) = -s;
  F(kVel, kOmega) = -dt * (s * vx + c * vy);
  F(kVel + 1, kVel) = s;
  F(kVel + 1, kVel + 1) = c;
  F(kVel + 1, kOmega) = dt * (c * vx - s * vy);
  return xn;
}

StateMatrix CTModel::processNoise(const StateVector & /*x*/, double dt) const
{
  StateMatrix Q = StateMatrix::Zero();
  addWhiteAccelerationNoise(Q, noise_.acc, dt);
  Q(kOmega, kOmega) = noise_.turn_rate * dt;
  return Q;
}

Eigen::Vector3d CTModel::acceleration(const StateVector & x) const
{
  const double w = x(kOmega);
  return Eigen::Vector3d(-w * x(kVel + 1), w * x(kVel), 0.0);
}

std::shared_ptr<const MotionModel> makeMotionModel(ModelType type, const ProcessNoise & noise)
{
  switch (type) {
    case ModelType::CV:
      return std::make_shared<const CVModel>(noise);
    case ModelType::CA:
      return std::make_shared<const CAModel>(noise);
    case ModelType::CT:
      return std::make_shared<const CTModel>(noise);
  }
  throw std::invalid_argument("unknown motion model");
}

}  // namespace estimation
}  // namespace interceptor
