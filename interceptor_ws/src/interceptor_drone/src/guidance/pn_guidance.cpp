/**
 * @file pn_guidance.cpp
 * @brief Proportional Navigation guidance law. Plain C++ and Eigen, no ROS.
 */

#include "guidance/pn_guidance.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>

namespace interceptor
{
namespace guidance
{

namespace
{
// Below this range the LOS direction is meaningless: the guidance is frozen.
constexpr double kMinRange = 1e-3;  // m
// Below this closing speed the target is not approaching: the time to go is not
// defined and the lead point would run away.
constexpr double kMinClosingSpeed = 0.5;  // m/s
}  // namespace

std::optional<GuidanceLaw> parseGuidanceLaw(const std::string & name)
{
  std::string lowered;
  lowered.reserve(name.size());
  for (const char c : name) {
    lowered.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
  }
  if (lowered == "pn") {
    return GuidanceLaw::PURE_PN;
  }
  if (lowered == "apn") {
    return GuidanceLaw::AUGMENTED_PN;
  }
  return std::nullopt;
}

std::string toString(GuidanceLaw law)
{
  switch (law) {
    case GuidanceLaw::PURE_PN:
      return "pn";
    case GuidanceLaw::AUGMENTED_PN:
      return "apn";
  }
  return "unknown";
}

LosKinematics computeLosKinematics(const EngagementState & state)
{
  LosKinematics los;
  los.relative_position = state.target_position - state.interceptor_position;
  los.relative_velocity = state.target_velocity - state.interceptor_velocity;
  los.range = los.relative_position.norm();

  if (los.range < kMinRange) {
    // Degenerate geometry: keep the unit vector along x and report no rotation.
    los.los_unit = Eigen::Vector3d::UnitX();
    los.closing_velocity = 0.0;
    los.los_rate = Eigen::Vector3d::Zero();
    los.time_to_go = 0.0;
    return los;
  }

  los.los_unit = los.relative_position / los.range;

  // Closing speed along the LOS: positive while the range shrinks.
  los.closing_velocity = -los.los_unit.dot(los.relative_velocity);

  // LOS rate vector: the relative velocity perpendicular to the LOS, divided by the
  // range. This is the classic lambda_dot = (v_r - (u . v_r) u) / |r|.
  const Eigen::Vector3d perpendicular_velocity =
    los.relative_velocity - los.los_unit * los.los_unit.dot(los.relative_velocity);
  los.los_rate = perpendicular_velocity / los.range;

  // Time to go: the range divided by the closing speed. A diverging or hovering
  // target has no finite time to go, which is reported as 0.
  if (los.closing_velocity > kMinClosingSpeed) {
    los.time_to_go = los.range / los.closing_velocity;
  } else {
    los.time_to_go = 0.0;
  }

  return los;
}

Eigen::Vector3d purePn(const LosKinematics & los, double nav_constant)
{
  return los.los_rate * (nav_constant * los.closing_velocity);
}

Eigen::Vector3d augmentedPn(
  const LosKinematics & los, const Eigen::Vector3d & target_acceleration, double nav_constant,
  double target_accel_gain, bool project_perpendicular)
{
  Eigen::Vector3d accel = purePn(los, nav_constant);

  Eigen::Vector3d target_term = target_acceleration;
  if (project_perpendicular) {
    // Only the part of the target acceleration that bends the LOS away from the
    // interceptor is worth compensating; the along-LOS part is handled by the
    // closing-speed term of the pure PN law.
    target_term -= los.los_unit * los.los_unit.dot(target_acceleration);
  }
  accel += target_term * (nav_constant * target_accel_gain);

  return accel;
}

double zeroEffortMiss(const LosKinematics & los)
{
  // If neither vehicle accelerates, the relative position evolves linearly along the
  // relative velocity. The miss distance is the distance from the origin to that line.
  const double range = los.range;
  const double speed = los.relative_velocity.norm();
  if (speed < 1e-6) {
    return range;
  }
  const double sin_angle = los.relative_position.cross(los.relative_velocity).norm() /
    (range * speed);
  return range * std::min(1.0, sin_angle);
}

Eigen::Vector3d saturateAcceleration(const Eigen::Vector3d & accel, double max_accel)
{
  const double norm = accel.norm();
  if (max_accel > 0.0 && norm > max_accel && norm > 1e-9) {
    return accel * (max_accel / norm);
  }
  return accel;
}

Eigen::Vector3d pnVelocitySetpoint(
  const LosKinematics & los, const Eigen::Vector3d & accel, double accel_to_velocity_gain,
  double max_speed)
{
  // Along the LOS: the closing speed the interceptor sustains. It is capped so the
  // vehicle never asks for more than it can deliver.
  const double closing_speed = std::clamp(los.closing_velocity, 0.0, max_speed);
  Eigen::Vector3d velocity = los.los_unit * closing_speed;

  // Perpendicular to the LOS: the guidance acceleration, which is the only lateral
  // authority. It is scaled by the effective time constant of the velocity loop, so
  // the acceleration is realised over that time.
  velocity += accel * accel_to_velocity_gain;

  return velocity;
}

PnGuidance::PnGuidance(const GuidanceParams & params)
: params_(params)
{
}

GuidanceResult PnGuidance::update(const EngagementState & state) const
{
  GuidanceResult result;
  result.law = params_.law;

  const LosKinematics los = computeLosKinematics(state);
  result.los = los;
  result.zero_effort_miss = zeroEffortMiss(los);

  Eigen::Vector3d accel = Eigen::Vector3d::Zero();
  switch (params_.law) {
    case GuidanceLaw::PURE_PN:
      accel = purePn(los, params_.nav_constant);
      break;
    case GuidanceLaw::AUGMENTED_PN:
      accel = augmentedPn(
        los, state.target_acceleration, params_.nav_constant, params_.target_accel_gain,
        params_.project_target_accel);
      break;
  }

  result.acceleration_unsaturated = accel;
  result.saturated = accel.norm() > params_.max_accel;
  result.acceleration = saturateAcceleration(accel, params_.max_accel);

  return result;
}

}  // namespace guidance
}  // namespace interceptor
