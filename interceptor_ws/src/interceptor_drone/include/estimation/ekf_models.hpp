#pragma once

#include <Eigen/Dense>

#include <array>
#include <memory>
#include <optional>
#include <string>

namespace interceptor
{
namespace estimation
{

// Common state layout shared by every motion model, so the IMM can mix model
// states with plain weighted sums:
//
//   x = [px py pz | vx vy vz | ax ay az | omega]   (10 states, ENU world frame)
//
// omega is the horizontal turn rate (yaw rate of the velocity vector, rad/s).
// Each model uses a subset and zero-pads the rest:
//
//   CV (6D) : p, v          a = 0, omega = 0
//   CA (9D) : p, v, a       omega = 0
//   CT (7D) : p, v, omega   a = 0 (the turn acceleration is omega x v, see acceleration())
//
// Zero-padding is the model's own hypothesis (a CV target has no acceleration
// and no turn rate), so the padded components carry zero mean and zero
// covariance. predict() re-applies the padding after every step, so whatever the
// IMM mixes into an unused component is dropped before it can affect the model.
// When mixing into a model, the IMM does not mix these padded zeros in; it
// borrows the receiving model's own estimate instead (see ImmFilter::predict).
// The measurement is always the 3D position, so H = [I3 0] for every model.
constexpr int kStateDim = 10;
constexpr int kPos = 0;
constexpr int kVel = 3;
constexpr int kAcc = 6;
constexpr int kOmega = 9;

using StateVector = Eigen::Matrix<double, kStateDim, 1>;
using StateMatrix = Eigen::Matrix<double, kStateDim, kStateDim>;
using StateMask = std::array<bool, kStateDim>;

enum class ModelType
{
  CV,  // Constant velocity
  CA,  // Constant acceleration
  CT,  // Coordinated turn (horizontal), constant velocity in z
};

// "cv" | "ca" | "ct" (case-insensitive); nullopt for anything else
std::optional<ModelType> modelTypeFromString(const std::string & name);
std::string toString(ModelType type);

// Continuous-time white-noise spectral densities of the process noise
// Defaults tuned on the synthetic trajectories of the unit tests (30 Hz, 0.3 m
// noise): CV stays stiff and leaves maneuvers to CA / CT, as an IMM should.
struct ProcessNoise
{
  double acc = 0.3;        // CV and CT: white acceleration, (m/s^2)^2 / Hz
  double jerk = 1.0;       // CA: white jerk, (m/s^3)^2 / Hz
  double turn_rate = 0.2;  // CT: random-walk turn rate, (rad/s^2)^2 / Hz
};

// Prior standard deviations for a track started from a single position fix
struct InitialUncertainty
{
  double velocity = 5.0;      // m/s
  double acceleration = 2.0;  // m/s^2 (CA)
  double turn_rate = 0.5;     // rad/s (CT)
};

// Result of a Kalman measurement update
struct UpdateResult
{
  Eigen::Vector3d innovation = Eigen::Vector3d::Zero();
  Eigen::Matrix3d innovation_cov = Eigen::Matrix3d::Identity();
  double mahalanobis2 = 0.0;    // innovation' S^-1 innovation
  double log_likelihood = 0.0;  // log N(innovation; 0, S)
  bool ok = false;              // false if S was not positive definite (state untouched)
};

// Squared Mahalanobis distance and Gaussian log-likelihood of a 3D innovation.
// Returns false if S is not positive definite.
bool innovationStatistics(
  const Eigen::Vector3d & innovation, const Eigen::Matrix3d & innovation_cov,
  double & mahalanobis2, double & log_likelihood);

// Position measurement update (H = [I3 0]) in Joseph form. Shared by all models:
// the measurement is linear in the common state layout.
UpdateResult positionUpdate(
  StateVector & x, StateMatrix & P, const Eigen::Vector3d & z, const Eigen::Matrix3d & R);

// Discrete EKF motion model on the common state layout
class MotionModel
{
public:
  virtual ~MotionModel() = default;

  virtual ModelType type() const = 0;

  // State components this model uses; the others are held at zero
  virtual StateMask activeStates() const = 0;

  // x <- f(x, dt), P <- F P F' + Q with F = df/dx. Applies the zero-padding.
  void predict(StateVector & x, StateMatrix & P, double dt) const;

  // f(x, dt) and its Jacobian F, without padding or noise
  virtual StateVector transition(const StateVector & x, double dt, StateMatrix & F) const = 0;

  // Discrete process noise Q(dt), zero on padded components
  virtual StateMatrix processNoise(const StateVector & x, double dt) const = 0;

  // Acceleration the model implies for a state (zero for CV, omega x v for CT)
  virtual Eigen::Vector3d acceleration(const StateVector & x) const = 0;

  // Zeros the padded components of x and the matching rows/columns of P
  void applyPadding(StateVector & x, StateMatrix & P) const;

  // Initial state from a position fix z with covariance R
  void initialize(
    const Eigen::Vector3d & z, const Eigen::Matrix3d & R, const InitialUncertainty & prior,
    StateVector & x, StateMatrix & P) const;
};

class CVModel : public MotionModel
{
public:
  explicit CVModel(const ProcessNoise & noise = ProcessNoise())
  : noise_(noise) {}

  ModelType type() const override {return ModelType::CV;}
  StateMask activeStates() const override;
  StateVector transition(const StateVector & x, double dt, StateMatrix & F) const override;
  StateMatrix processNoise(const StateVector & x, double dt) const override;
  Eigen::Vector3d acceleration(const StateVector & x) const override;

private:
  ProcessNoise noise_;
};

class CAModel : public MotionModel
{
public:
  explicit CAModel(const ProcessNoise & noise = ProcessNoise())
  : noise_(noise) {}

  ModelType type() const override {return ModelType::CA;}
  StateMask activeStates() const override;
  StateVector transition(const StateVector & x, double dt, StateMatrix & F) const override;
  StateMatrix processNoise(const StateVector & x, double dt) const override;
  Eigen::Vector3d acceleration(const StateVector & x) const override;

private:
  ProcessNoise noise_;
};

// Coordinated turn in the horizontal plane with constant speed and turn rate,
// constant velocity in z. Nonlinear in omega: the EKF uses the analytic Jacobian,
// with series expansions for |omega| -> 0 (where it reduces to CV).
class CTModel : public MotionModel
{
public:
  explicit CTModel(const ProcessNoise & noise = ProcessNoise())
  : noise_(noise) {}

  ModelType type() const override {return ModelType::CT;}
  StateMask activeStates() const override;
  StateVector transition(const StateVector & x, double dt, StateMatrix & F) const override;
  StateMatrix processNoise(const StateVector & x, double dt) const override;
  Eigen::Vector3d acceleration(const StateVector & x) const override;

private:
  ProcessNoise noise_;
};

std::shared_ptr<const MotionModel> makeMotionModel(ModelType type, const ProcessNoise & noise);

}  // namespace estimation
}  // namespace interceptor
