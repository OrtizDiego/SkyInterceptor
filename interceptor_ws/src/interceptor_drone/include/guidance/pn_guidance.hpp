/**
 * @file pn_guidance.hpp
 * @brief Proportional Navigation guidance (pure PN and augmented PN). Plain C++ and
 *        Eigen, no ROS, so the guidance law can be unit tested on synthetic geometry.
 *
 * Frames and units: every vector is in the world frame (ENU, metres, seconds), the
 * same frame as /odom and /target/state.
 *
 * The law
 * -------
 * Pure PN (Shneydor, "Missile Guidance and Pursuit"):
 *
 *     a_M = N * V_c * lambda_dot
 *
 * where V_c = -d(r)/dt is the closing speed along the line of sight (positive while
 * the range shrinks) and lambda_dot is the LOS rate vector. With
 * r = p_T - p_M, u = r / |r| and v_r = v_T - v_M, the LOS rate vector is
 *
 *     lambda_dot = (v_r - (u . v_r) u) / |r|
 *
 * which is the component of the relative velocity perpendicular to the LOS, divided
 * by the range. The commanded acceleration is therefore perpendicular to the LOS and
 * rotates the LOS onto the target.
 *
 * Augmented PN adds the target's acceleration, projected perpendicular to the LOS:
 *
 *     a_M = N * V_c * lambda_dot + (N / 2) * a_T_perp
 *
 * The (N/2) factor is the textbook value (plan section 4.4). The perpendicular
 * projection keeps the commanded acceleration perpendicular to the LOS, so the
 * along-LOS component of the target's acceleration does not corrupt the guidance
 * geometry; only the part of a_T that bends the LOS away is worth compensating.
 * A simulation sweep over random engagements confirmed k = N/2 with the projection
 * intercepts the large majority of the cases where pure PN misses.
 *
 * The perpendicular projection is applied to the target acceleration, not to the
 * PN term: the PN term is already perpendicular to the LOS by construction.
 */

#pragma once

#include <Eigen/Dense>

#include <optional>
#include <string>

namespace interceptor
{
namespace guidance
{

enum class GuidanceLaw
{
  PURE_PN,       // n_c = N * V_c * lambda_dot
  AUGMENTED_PN,  // pure PN + target acceleration term (from the tracker's estimate)
};

/// "pn" | "apn" (case-insensitive) -> GuidanceLaw, std::nullopt otherwise
std::optional<GuidanceLaw> parseGuidanceLaw(const std::string & name);
std::string toString(GuidanceLaw law);

struct GuidanceParams
{
  GuidanceLaw law = GuidanceLaw::AUGMENTED_PN;
  double nav_constant = 4.0;  // N (effective navigation ratio N'), typically 3..5
  double max_accel = 15.0;    // m/s^2, norm limit on the commanded acceleration
  // Multiplier on the target acceleration in augmented PN. 0.5 gives the textbook
  // (N/2) * a_T_perp term; set to 0 for pure PN behaviour with the same law.
  double target_accel_gain = 0.5;
  // Whether the target acceleration is projected perpendicular to the LOS before it
  // is added. True keeps the command perpendicular to the LOS (textbook form).
  bool project_target_accel = true;
};

/// Inputs to one guidance update. World frame (ENU).
struct EngagementState
{
  Eigen::Vector3d interceptor_position = Eigen::Vector3d::Zero();  // m       (/odom)
  Eigen::Vector3d interceptor_velocity = Eigen::Vector3d::Zero();  // m/s     (/odom, world)
  Eigen::Vector3d target_position = Eigen::Vector3d::Zero();       // m       (/target/state)
  Eigen::Vector3d target_velocity = Eigen::Vector3d::Zero();       // m/s     (/target/state)
  Eigen::Vector3d target_acceleration = Eigen::Vector3d::Zero();   // m/s^2   (/target/state, IMM)
};

/// Line-of-sight (LOS) kinematics of the engagement. World frame (ENU).
struct LosKinematics
{
  // r = p_target - p_interceptor [m]
  Eigen::Vector3d relative_position = Eigen::Vector3d::Zero();
  // v_r = v_target - v_interceptor [m/s]
  Eigen::Vector3d relative_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d los_unit = Eigen::Vector3d::UnitX();  // r / |r| (unit vector)
  double range = 0.0;                                   // |r| [m]
  // lambda_dot: LOS angular-rate vector [rad/s]
  Eigen::Vector3d los_rate = Eigen::Vector3d::Zero();
  double closing_velocity = 0.0;  // V_c [m/s], positive while the range is shrinking
  double time_to_go = 0.0;        // t_go [s], estimated time to intercept (>= 0)
};

/// Output of one guidance update
struct GuidanceResult
{
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();              // m/s^2, saturated
  Eigen::Vector3d acceleration_unsaturated = Eigen::Vector3d::Zero();  // m/s^2, raw law output
  bool saturated = false;          // |acceleration_unsaturated| > max_accel
  LosKinematics los;               // geometry used for this command
  double zero_effort_miss = 0.0;   // m, predicted miss if neither vehicle manoeuvres
  GuidanceLaw law = GuidanceLaw::AUGMENTED_PN;
};

// ---------------------------------------------------------------------------
// PN law
// ---------------------------------------------------------------------------

/**
 * @brief LOS geometry of the engagement.
 * @param state  interceptor and target position / velocity (world frame)
 * @return every field of LosKinematics: relative position/velocity, LOS unit vector,
 *         range, LOS rate vector lambda_dot [rad/s], closing velocity V_c [m/s]
 *         (> 0 when closing) and time to go t_go [s].
 */
LosKinematics computeLosKinematics(const EngagementState & state);

/**
 * @brief Pure proportional navigation: n_c = N * V_c * lambda_dot.
 * @param los           output of computeLosKinematics()
 * @param nav_constant  N [-]
 * @return commanded interceptor acceleration [m/s^2], world frame, unsaturated
 *         (perpendicular to the LOS).
 */
Eigen::Vector3d purePn(const LosKinematics & los, double nav_constant);

/**
 * @brief Augmented proportional navigation: pure PN plus a term for the target's
 *        acceleration (the tracker's IMM estimate on /target/state).
 * @param los                  output of computeLosKinematics()
 * @param target_acceleration  estimated target acceleration a_T [m/s^2], world frame
 * @param nav_constant         N [-]
 * @param target_accel_gain    multiplier on a_T (0.5 gives the textbook (N/2) term)
 * @param project_perpendicular  project a_T perpendicular to the LOS before adding it
 * @return commanded interceptor acceleration [m/s^2], world frame, unsaturated.
 */
Eigen::Vector3d augmentedPn(
  const LosKinematics & los, const Eigen::Vector3d & target_acceleration, double nav_constant,
  double target_accel_gain = 0.5, bool project_perpendicular = true);

/**
 * @brief Zero-effort miss: miss distance if neither vehicle accelerates from now on.
 * @param los  output of computeLosKinematics()
 * @return miss distance [m] (>= 0). Telemetry only.
 */
double zeroEffortMiss(const LosKinematics & los);

// ---------------------------------------------------------------------------

/// Limits the norm of an acceleration command to max_accel (direction kept)
Eigen::Vector3d saturateAcceleration(const Eigen::Vector3d & accel, double max_accel);

/**
 * @brief Velocity setpoint for a vehicle that flies a velocity loop.
 *
 * The interceptor flies a velocity loop (the trajectory controller commands a velocity
 * and the onboard autopilot closes it), so the acceleration law has to be turned into a
 * velocity setpoint. The setpoint has two parts:
 *
 *   - along the line of sight: the closing speed the interceptor sustains, so the range
 *     shrinks at the rate the guidance geometry assumes;
 *   - perpendicular to the line of sight: the guidance acceleration, scaled by
 *     `accel_to_velocity_gain` (the effective time constant of the velocity loop).
 *
 * The guidance acceleration is the only lateral authority: it is what rotates the LOS
 * onto the target, which is the whole point of proportional navigation. Adding the
 * target's velocity to the setpoint instead would track the target directly and hide
 * what the guidance law contributes.
 *
 * @param los    output of computeLosKinematics()
 * @param accel  the guidance acceleration [m/s^2]
 * @param accel_to_velocity_gain  s, how much the acceleration bends the velocity
 * @param max_speed  m/s, cap on the closing speed along the LOS
 * @return velocity setpoint [m/s], world frame
 */
Eigen::Vector3d pnVelocitySetpoint(
  const LosKinematics & los, const Eigen::Vector3d & accel, double accel_to_velocity_gain,
  double max_speed);

/**
 * PN guidance law: LOS kinematics -> selected law -> saturation.
 * Stateless apart from its parameters; the phase logic lives in the node.
 */
class PnGuidance
{
public:
  explicit PnGuidance(const GuidanceParams & params = GuidanceParams());

  GuidanceResult update(const EngagementState & state) const;

  const GuidanceParams & params() const {return params_;}

private:
  GuidanceParams params_;
};

}  // namespace guidance
}  // namespace interceptor
