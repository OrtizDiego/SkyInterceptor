#include <gtest/gtest.h>
#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "estimation/ekf_models.hpp"
#include "estimation/imm_filter.hpp"
#include "synthetic_trajectory.hpp"

namespace interceptor
{
namespace estimation
{
namespace
{

constexpr double kRate = 30.0;  // Hz, ground-truth detection rate
constexpr double kDt = 1.0 / kRate;
constexpr double kSigma = 0.3;  // m, measurement noise

StateVector stateOf(
  const Eigen::Vector3d & p, const Eigen::Vector3d & v,
  const Eigen::Vector3d & a = Eigen::Vector3d::Zero(), double omega = 0.0)
{
  StateVector x = StateVector::Zero();
  x.segment<3>(kPos) = p;
  x.segment<3>(kVel) = v;
  x.segment<3>(kAcc) = a;
  x(kOmega) = omega;
  return x;
}

bool isPositiveSemiDefinite(const Eigen::MatrixXd & P)
{
  if (!P.isApprox(P.transpose(), 1e-9)) {
    return false;
  }
  const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(P);
  return eig.eigenvalues().minCoeff() > -1e-9;
}

// Central-difference Jacobian of the model transition
StateMatrix numericJacobian(const MotionModel & model, const StateVector & x, double dt)
{
  StateMatrix J;
  StateMatrix unused;
  const double h = 1e-6;
  for (int i = 0; i < kStateDim; ++i) {
    StateVector xp = x;
    StateVector xm = x;
    xp(i) += h;
    xm(i) -= h;
    J.col(i) = (model.transition(xp, dt, unused) - model.transition(xm, dt, unused)) / (2.0 * h);
  }
  return J;
}

// --- Models -------------------------------------------------------------------

TEST(MotionModels, ParseNames)
{
  EXPECT_EQ(modelTypeFromString("cv"), ModelType::CV);
  EXPECT_EQ(modelTypeFromString("CA"), ModelType::CA);
  EXPECT_EQ(modelTypeFromString("Ct"), ModelType::CT);
  EXPECT_FALSE(modelTypeFromString("singer").has_value());
  EXPECT_EQ(toString(ModelType::CT), "ct");
}

TEST(MotionModels, CVPredictsStraightLineAndPadsUnusedStates)
{
  const CVModel cv;
  StateVector x = stateOf({1.0, 2.0, 3.0}, {2.0, -1.0, 0.5}, {9.0, 9.0, 9.0}, 0.7);
  StateMatrix P = StateMatrix::Identity();
  cv.predict(x, P, 2.0);
  EXPECT_TRUE(x.segment<3>(kPos).isApprox(Eigen::Vector3d(5.0, 0.0, 4.0)));
  EXPECT_TRUE(x.segment<3>(kVel).isApprox(Eigen::Vector3d(2.0, -1.0, 0.5)));
  // CV hypothesis: no acceleration, no turn rate, and no uncertainty about it
  EXPECT_TRUE(x.segment<3>(kAcc).isZero());
  EXPECT_DOUBLE_EQ(x(kOmega), 0.0);
  EXPECT_TRUE((P.block<4, kStateDim>(kAcc, 0).isZero()));
  EXPECT_TRUE((P.block<kStateDim, 4>(0, kAcc).isZero()));
  EXPECT_TRUE(isPositiveSemiDefinite(P));
}

TEST(MotionModels, CAPredictsConstantAcceleration)
{
  const CAModel ca;
  StateVector x = stateOf({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, -1.0}, 0.5);
  StateMatrix P = StateMatrix::Identity();
  ca.predict(x, P, 3.0);
  EXPECT_TRUE(x.segment<3>(kPos).isApprox(Eigen::Vector3d(12.0, 0.0, -4.5)));
  EXPECT_TRUE(x.segment<3>(kVel).isApprox(Eigen::Vector3d(7.0, 0.0, -3.0)));
  EXPECT_TRUE(x.segment<3>(kAcc).isApprox(Eigen::Vector3d(2.0, 0.0, -1.0)));
  EXPECT_DOUBLE_EQ(x(kOmega), 0.0);
  EXPECT_TRUE(isPositiveSemiDefinite(P));
}

TEST(MotionModels, CTFollowsCircleAtConstantSpeed)
{
  // Quarter turn to the left at 10 m/s and 0.5 rad/s: radius 20 m
  const CTModel ct;
  const double w = 0.5;
  const double t = (M_PI / 2.0) / w;
  StateVector x = stateOf({0.0, 0.0, 10.0}, {10.0, 0.0, 1.0}, Eigen::Vector3d::Zero(), w);
  StateMatrix P = StateMatrix::Identity() * 0.1;
  ct.predict(x, P, t);
  EXPECT_NEAR(x(kPos), 20.0, 1e-9);
  EXPECT_NEAR(x(kPos + 1), 20.0, 1e-9);
  EXPECT_NEAR(x(kPos + 2), 10.0 + t, 1e-9);
  EXPECT_NEAR(x(kVel), 0.0, 1e-9);
  EXPECT_NEAR(x(kVel + 1), 10.0, 1e-9);
  EXPECT_NEAR(x(kOmega), w, 1e-12);
  // Centripetal acceleration omega x v, pointing to the centre (0, 20)
  EXPECT_TRUE(ct.acceleration(x).isApprox(Eigen::Vector3d(-5.0, 0.0, 0.0), 1e-9));
  EXPECT_TRUE(isPositiveSemiDefinite(P));
}

TEST(MotionModels, CTWithZeroTurnRateMatchesCV)
{
  const CTModel ct;
  const CVModel cv;
  StateMatrix F_ct;
  StateMatrix F_cv;
  const StateVector x = stateOf({1.0, 2.0, 3.0}, {4.0, -3.0, 0.2});
  const StateVector x_ct = ct.transition(x, 0.5, F_ct);
  const StateVector x_cv = cv.transition(x, 0.5, F_cv);
  EXPECT_TRUE(x_ct.isApprox(x_cv, 1e-12));
  // Same Jacobian, except the CT sensitivity of the position to omega (v dt^2 / 2)
  EXPECT_NEAR(F_ct(kPos, kOmega), 3.0 * 0.125, 1e-12);
  EXPECT_NEAR(F_ct(kPos + 1, kOmega), 4.0 * 0.125, 1e-12);
}

TEST(MotionModels, CTJacobianMatchesFiniteDifferences)
{
  const CTModel ct;
  // Regular branch, series branch and both sides of the switch-over
  for (const double w : {0.8, -0.3, 1e-6, 0.0, 0.99e-3 / 0.1, 1.01e-3 / 0.1}) {
    const StateVector x = stateOf({3.0, -2.0, 15.0}, {8.0, 5.0, -1.0}, Eigen::Vector3d::Zero(), w);
    StateMatrix F;
    ct.transition(x, 0.1, F);
    const StateMatrix J = numericJacobian(ct, x, 0.1);
    // Only the active block matters (padding removes the rest)
    for (int r : {0, 1, 2, 3, 4, 5, 9}) {
      for (int c : {0, 1, 2, 3, 4, 5, 9}) {
        EXPECT_NEAR(F(r, c), J(r, c), 1e-6) << "omega " << w << " F(" << r << ", " << c << ")";
      }
    }
  }
}

TEST(MotionModels, PositionUpdateShrinksCovarianceAndStaysPsd)
{
  const CAModel ca;
  StateVector x;
  StateMatrix P;
  const Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * 0.09;
  ca.initialize(Eigen::Vector3d(1.0, 1.0, 1.0), R, InitialUncertainty(), x, P);
  ca.predict(x, P, 0.1);
  const double before = P.trace();
  const UpdateResult result = positionUpdate(x, P, Eigen::Vector3d(1.2, 0.9, 1.0), R);
  ASSERT_TRUE(result.ok);
  EXPECT_LT(P.trace(), before);
  EXPECT_TRUE(isPositiveSemiDefinite(P));
  EXPECT_TRUE(std::isfinite(result.log_likelihood));
  // Padded turn rate stays untouched by the update
  EXPECT_DOUBLE_EQ(x(kOmega), 0.0);
  EXPECT_DOUBLE_EQ(P(kOmega, kOmega), 0.0);
}

// --- IMM parameters -------------------------------------------------------------

TEST(ImmParams, ParsesModelNamesAndRowMajorMatrix)
{
  const ImmParams p = makeImmParams({"cv", "ct"}, {0.9, 0.1, 0.2, 0.8});
  ASSERT_EQ(p.models.size(), 2u);
  EXPECT_EQ(p.models[1], ModelType::CT);
  EXPECT_DOUBLE_EQ(p.transition(0, 1), 0.1);
  EXPECT_DOUBLE_EQ(p.transition(1, 0), 0.2);
}

TEST(ImmParams, RejectsInvalidConfigurations)
{
  EXPECT_THROW(makeImmParams({"cv", "jerk"}, {0.9, 0.1, 0.2, 0.8}), std::invalid_argument);
  EXPECT_THROW(makeImmParams({"cv", "ca"}, {0.9, 0.1, 0.2}), std::invalid_argument);
  EXPECT_THROW(makeImmParams({"cv", "ca"}, {0.9, 0.2, 0.2, 0.8}), std::invalid_argument);
  EXPECT_THROW(makeImmParams({"cv", "ca"}, {1.1, -0.1, 0.2, 0.8}), std::invalid_argument);
  EXPECT_THROW(makeImmParams({}, {}), std::invalid_argument);
  ImmParams p = defaultGroundImmParams();
  p.initial_probabilities = Eigen::Vector2d(0.5, 0.6);
  EXPECT_THROW(ImmFilter{p}, std::invalid_argument);
}

// --- IMM filter -------------------------------------------------------------------

struct ImmRun
{
  double rmse;
  std::vector<Eigen::VectorXd> probabilities;  // per frame, after the update
};

// Runs a single IMM filter on a noisy trajectory and returns the position RMSE
// after warmup seconds
ImmRun runImm(
  const ImmParams & params, const std::vector<testing::TruthSample> & truth, unsigned seed,
  double warmup = 1.0)
{
  testing::MeasurementNoise noise(kSigma, seed);
  ImmFilter imm(params);
  ImmRun run;
  double sum2 = 0.0;
  int n = 0;
  for (size_t k = 0; k < truth.size(); ++k) {
    const Eigen::Vector3d z = noise.sample(truth[k].position);
    if (k == 0) {
      imm.initialize(z, noise.covariance());
    } else {
      imm.predict(truth[k].t - truth[k - 1].t);
      EXPECT_TRUE(imm.update(z, noise.covariance()));
    }
    EXPECT_NEAR(imm.modelProbabilities().sum(), 1.0, 1e-9);
    run.probabilities.push_back(imm.modelProbabilities());
    if (truth[k].t >= warmup) {
      sum2 += (imm.estimate().position() - truth[k].position).squaredNorm();
      ++n;
    }
  }
  EXPECT_TRUE(isPositiveSemiDefinite(imm.estimate().covariance));
  run.rmse = std::sqrt(sum2 / std::max(n, 1));
  return run;
}

TEST(ImmFilter, InitializesAtTheFixWithInitialProbabilities)
{
  ImmParams params = defaultAerialImmParams();
  params.initial_probabilities = Eigen::Vector2d(0.7, 0.3);
  ImmFilter imm(params);
  EXPECT_FALSE(imm.initialized());
  const Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * 0.09;
  imm.initialize(Eigen::Vector3d(1.0, 2.0, 30.0), R);
  EXPECT_TRUE(imm.initialized());
  EXPECT_TRUE(imm.estimate().position().isApprox(Eigen::Vector3d(1.0, 2.0, 30.0)));
  EXPECT_TRUE(imm.estimate().velocity().isZero());
  EXPECT_TRUE(imm.estimate().positionCovariance().isApprox(R));
  EXPECT_NEAR(imm.modelProbabilities()(0), 0.7, 1e-12);
}

TEST(ImmFilter, TransitionMatrixScalesWithTime)
{
  ImmParams params = defaultGroundImmParams();  // [0.9 0.1; 0.2 0.8] per second
  ImmFilter imm(params);
  EXPECT_TRUE(imm.transitionMatrix(1.0).isApprox(params.transition));
  EXPECT_TRUE(imm.transitionMatrix(0.0).isApprox(Eigen::Matrix2d::Identity()));
  // Two seconds: staying twice in a row
  EXPECT_NEAR(imm.transitionMatrix(2.0)(0, 0), 0.81, 1e-12);
  const Eigen::MatrixXd frame = imm.transitionMatrix(1.0 / 30.0);
  EXPECT_NEAR(frame(0, 0), std::pow(0.9, 1.0 / 30.0), 1e-12);
  EXPECT_NEAR(frame(1, 1), std::pow(0.8, 1.0 / 30.0), 1e-12);
  EXPECT_TRUE(frame.rowwise().sum().isApprox(Eigen::Vector2d::Ones()));

  // Interval 0: the matrix applies as is, once per predict()
  params.transition_interval = 0.0;
  EXPECT_TRUE(ImmFilter(params).transitionMatrix(1.0 / 30.0).isApprox(params.transition));
}

TEST(ImmFilter, PredictAppliesMarkovChainAndExtrapolateIsConst)
{
  ImmParams params = defaultGroundImmParams();
  params.initial_probabilities = Eigen::Vector2d(1.0, 0.0);
  ImmFilter imm(params);
  imm.initialize(Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity() * 0.09);

  const ImmEstimate ahead = imm.extrapolate(1.0);
  EXPECT_NEAR(imm.modelProbabilities()(0), 1.0, 1e-12);  // unchanged
  EXPECT_GT(ahead.positionCovariance().trace(), imm.estimate().positionCovariance().trace());

  imm.predict(1.0);
  // c = Pi(1 s)' mu = [0.9, 0.1]
  EXPECT_NEAR(imm.modelProbabilities()(0), 0.9, 1e-12);
  EXPECT_NEAR(imm.modelProbabilities()(1), 0.1, 1e-12);
}

TEST(ImmFilter, MahalanobisDistanceOfCombinedEstimate)
{
  ImmFilter imm(defaultGroundImmParams());
  imm.initialize(Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity() * 0.5);
  // S = P_pos + R = I: d^2 = |z|^2
  EXPECT_NEAR(
    imm.mahalanobis2(Eigen::Vector3d(1.0, 2.0, 2.0), Eigen::Matrix3d::Identity() * 0.5), 9.0,
    1e-9);
}

TEST(ImmFilter, StraightLineFavoursCVAndBeatsMeasurementNoise)
{
  const auto truth = testing::generateTrajectory(
    {0.0, 0.0, 0.9}, {1.4, 0.3, 0.0}, {{20.0}}, kRate);
  const ImmRun run = runImm(defaultGroundImmParams(), truth, 1);
  EXPECT_LT(run.rmse, 0.3);
  EXPECT_LT(run.rmse, kSigma * std::sqrt(3.0) / 2.0);  // well below the raw 3D error
  EXPECT_GT(run.probabilities.back()(0), 0.8);
}

TEST(ImmFilter, AccelerationSwitchesToCA)
{
  // Car: cruise, hard acceleration, cruise
  const auto truth = testing::generateTrajectory(
    {0.0, 0.0, 0.8}, {5.0, 0.0, 0.0}, {{6.0}, {5.0, 3.0}, {6.0}}, kRate);
  const ImmRun run = runImm(defaultGroundImmParams(), truth, 2);
  EXPECT_LT(run.rmse, 0.3);
  const auto prob_ca_at = [&](double t) {
      return run.probabilities[static_cast<size_t>(t * kRate)](1);
    };
  EXPECT_LT(prob_ca_at(5.5), 0.5);   // cruising
  EXPECT_GT(prob_ca_at(10.5), 0.5);  // accelerating
  EXPECT_LT(prob_ca_at(16.5), 0.5);  // cruising again
}

TEST(ImmFilter, TurnSwitchesToCT)
{
  // Drone at 10 m/s: straight, 0.5 rad/s turn, straight
  const auto truth = testing::generateTrajectory(
    {0.0, 0.0, 20.0}, {10.0, 0.0, 0.0}, {{6.0}, {6.0, 0.0, 0.5}, {6.0}}, kRate);
  const ImmRun run = runImm(defaultAerialImmParams(), truth, 3);
  EXPECT_LT(run.rmse, 0.3);
  const auto prob_ct_at = [&](double t) {
      return run.probabilities[static_cast<size_t>(t * kRate)](1);
    };
  EXPECT_LT(prob_ct_at(5.5), 0.5);
  EXPECT_GT(prob_ct_at(11.5), 0.5);
  EXPECT_LT(prob_ct_at(17.5), 0.5);
}

TEST(ImmFilter, CombinedAccelerationFollowsTheTurn)
{
  // Steady 0.5 rad/s turn at 10 m/s: 5 m/s^2 centripetal acceleration. Over the
  // last 5 s the turn rate and acceleration must be unbiased and not too noisy.
  const double w = 0.5;
  const auto truth = testing::generateTrajectory(
    {0.0, 0.0, 20.0}, {10.0, 0.0, 0.0}, {{12.0, 0.0, w}}, kRate);
  testing::MeasurementNoise noise(kSigma, 4);
  ImmFilter imm(defaultAerialImmParams());
  imm.initialize(noise.sample(truth[0].position), noise.covariance());
  double w_sum = 0.0;
  double w_sum2 = 0.0;
  double a_sum2 = 0.0;
  int n = 0;
  for (size_t k = 1; k < truth.size(); ++k) {
    imm.predict(kDt);
    imm.update(noise.sample(truth[k].position), noise.covariance());
    if (truth[k].t < 7.0) {
      continue;
    }
    const Eigen::Vector3d v = truth[k].velocity;
    const Eigen::Vector3d a_true(-w * v.y(), w * v.x(), 0.0);
    const double w_err = imm.estimate().turnRate() - w;
    w_sum += w_err;
    w_sum2 += w_err * w_err;
    a_sum2 += (imm.estimate().acceleration - a_true).squaredNorm();
    ++n;
  }
  EXPECT_LT(std::abs(w_sum / n), 0.05);       // rad/s bias
  EXPECT_LT(std::sqrt(w_sum2 / n), 0.2);      // rad/s RMS
  EXPECT_LT(std::sqrt(a_sum2 / n), 2.0);      // m/s^2 RMS, of 5 m/s^2
  EXPECT_GT(imm.modelProbabilities()(1), 0.8);  // CT
}

}  // namespace
}  // namespace estimation
}  // namespace interceptor
