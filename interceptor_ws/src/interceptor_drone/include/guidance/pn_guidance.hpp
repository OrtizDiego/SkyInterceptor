/**
 * @file pn_guidance.hpp
 * @brief Proportional Navigation guidance (pure PN and augmented PN) and the terminal
 *        capture law that turns the pursuit into a capture. Plain C++ and Eigen, no
 *        ROS, so the guidance law can be unit tested on synthetic geometry.
 *
 * Frames and units: every vector is in the world frame (ENU, metres, seconds), the
 * same frame as /odom and /target/state.
 *
 * The pursuit law
 * ----------------
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
 *
 * The terminal capture law
 * -----------------------
 * PN is a pursuit law: it rotates the LOS onto the target but it never bleeds off the
 * closing speed, so the interceptor flies *through* the target. Measured in the Gazebo
 * demo before this law existed: the interceptor reached the capture envelope (1.5 m)
 * at 10-12 m/s relative speed, against a limit of 5 m/s, and the capture was detected
 * but never flown.
 *
 * The law below fixes that by commanding the *relative* velocity instead of guessing
 * it from the measured closing speed:
 *
 *     v_M = v_T + u * v_close(r) + a_pn * tau
 *
 * with three parts:
 *
 *   - v_T, the target's velocity, is a feedforward. It says "go where the target
 *     goes", so the quantity the law actually shapes is the relative velocity
 *     v_r = v_T - v_M, which is
 *
 *         v_r = -u * v_close(r) - a_pn * tau
 *
 *     i.e. the interceptor flies *with* the target and closes on it, instead of
 *     flying at it. The relative speed at the capture is then the closing speed
 *     (plus the small lateral term), which the closure profile governs.
 *   - u * v_close(r) is the governed closure along the line of sight, see below.
 *   - a_pn * tau is the PN lateral term, the only lateral authority. It is the
 *     acceleration of the pursuit law (pure PN or augmented PN) turned into a
 *     velocity offset by the effective time constant of the vehicle's velocity loop.
 *     Keeping it is what the earlier "relative velocity match" experiment in the
 *     capture phase got wrong: dropping it loses the lateral correction and the
 *     range oscillates instead of shrinking.
 *
 * Why the old along-LOS term was wrong
 * ------------------------------------
 * The setpoint used before this law commanded, along the LOS, the *measured* closing
 * speed V_c. That term has no integral action: in the steady state the interceptor's
 * along-LOS velocity converges to the target's, so the range stops shrinking and the
 * interceptor simply matches the target's speed. A sweep over the target's speed and
 * manoeuvre measured that against a target flying away at a constant 2, 4 or 6 m/s the
 * range never changed at all (20 m at the start, 20 m after 40 s): the old law only
 * closed the gap in the demo because the intruder accelerated from a standstill.
 *
 * The closure profile
 * -------------------
 * The commanded closing speed is the fastest speed from which the vehicle can still
 * slow down to `final_closing_speed` at the hold radius under a constant
 * deceleration `approach_decel`:
 *
 *     v_profile(r) = sqrt( v_final^2 + 2 * a * (r - r_hold) )   for r > r_hold
 *     v_profile(r) = 0                                           for r <= r_hold
 *
 * This is the braking curve of a pursuer: the deceleration it demands is exactly `a`
 * everywhere, so it is a profile the vehicle can actually track, and the closing
 * speed at the hold radius is `v_final` by construction. The hold radius is *inside*
 * the capture radius on purpose: the range converges to it, so aiming at the
 * envelope boundary would approach it asymptotically and never enter the envelope.
 *
 * The profile on its own is not enough, because it is an open-loop schedule on the
 * range: the range closes at the speed the vehicle actually flies, so the profile
 * falls behind, the vehicle carries more speed into the envelope than the profile
 * asks for, and it arrives at the capture too fast. The measured closing speed is
 * therefore fed back against the profile:
 *
 *     v_close = v_profile - k * (V_c - v_profile)
 *
 * whose steady state is the profile itself, but which slows the vehicle down on the
 * way in as soon as it is closing faster than the profile wants. Measured on the
 * sweep below: without the feedback the interceptor arrives at 3.9 m/s and
 * overshoots the target by a metre, with it at 2.0-3.3 m/s and it does not.
 *
 * The gain has to stay well below 1. The feedback goes through the lag of the
 * vehicle's velocity loop, and a gain of 1 makes the loop ring on the real
 * airframe (measured in the simulator: the interceptor circled the target instead
 * of closing on it). At 0.5 the law is stable over a lag of 0.3-1.0 s.
 *
 * The numbers below come from a sweep of the law against a model of the vehicle
 * (the X500 flight model with the autopilot's velocity PI) over the target's speed
 * (0-10 m/s), its manoeuvre (hover, straight, head-on, crossing, weave and the
 * demo's accelerating intruder), the initial range (5-30 m), the geometry and the
 * lag of the vehicle's velocity loop. Every scenario of the sweep reaches the
 * capture envelope with a relative speed of 1.5-3.3 m/s, against the 5 m/s limit,
 * and the range shrinks monotonically into it. See test/test_guidance.cpp.
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

/// The parameters of the terminal capture law.
struct CaptureLawParams
{
  // The range the interceptor settles at. It has to be inside the capture radius:
  // the range converges to it, so aiming at the envelope boundary would approach
  // the boundary asymptotically and never enter the envelope.
  double hold_radius = 1.2;          // m
  // The deceleration the closure profile assumes [m/s^2]. It is the deceleration
  // the vehicle is asked for all the way in, so it has to be within what the
  // airframe can deliver (about 7 m/s^2 horizontally at the 35 deg tilt limit).
  double approach_decel = 2.0;       // m/s^2
  // The closing speed at the hold radius [m/s]. This is the relative speed the
  // interceptor arrives with, so it is what the capture limit has to be checked
  // against.
  double final_closing_speed = 0.5;  // m/s
  // The cap on the commanded closing speed [m/s]. Beyond the vehicle's own speed
  // limit the command is pointless.
  double max_closing_speed = 8.0;    // m/s
  // The gain of the closing-speed feedback [-]. The profile is the reference and
  // the measured closing speed is fed back against it, which is what compensates
  // the lag of the vehicle's velocity loop: without it the vehicle carries more
  // speed into the envelope than the profile asks for and arrives too fast. The
  // gain has to stay well below 1: the feedback goes through the lag of the
  // vehicle, and a gain of 1 makes the loop ring on the real airframe.
  double closing_speed_feedback = 0.5;
};

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
  // The terminal capture law.
  CaptureLawParams capture;
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

// ---------------------------------------------------------------------------
// The terminal capture law
// ---------------------------------------------------------------------------

/**
 * @brief The closure profile: the commanded closing speed at a given range.
 *
 * The profile is the braking curve of a pursuer: the fastest speed from which the
 * vehicle can still slow down to `final_closing_speed` at the hold radius under a
 * constant deceleration `approach_decel`. The deceleration it asks for is therefore
 * `approach_decel` everywhere, which is a profile the vehicle can actually track,
 * and the closing speed at the hold radius is `final_closing_speed` by construction.
 *
 * @param range  the current range [m]
 * @param params  the capture law parameters
 * @return the closing speed the profile asks for [m/s], >= 0
 */
double closureProfile(double range, const CaptureLawParams & params);

/**
 * @brief The commanded closing speed: the profile, corrected by the tracking error.
 *
 * The profile alone is an open-loop schedule on the range, and the vehicle cannot
 * follow it: the range closes at the speed the vehicle actually flies, so the
 * profile falls behind, the vehicle carries more speed into the envelope than the
 * profile asks for, and it arrives at the capture too fast (measured: 3.9 m/s
 * instead of 2.4 m/s, and it overshot the target by a metre).
 *
 * The correction feeds the measured closing speed back against the profile:
 *
 *     v_close = v_profile - k * (V_c - v_profile)
 *
 * The steady state is the profile (the error decays with the time constant of the
 * velocity loop divided by 1 + k), but on the way in the vehicle is slowed down as
 * soon as it is closing faster than the profile wants. With k = 1 the measured
 * closing speed at the capture drops from 3.9 m/s to 2.4 m/s and the interceptor
 * stops overshooting the target.
 *
 * @param los  the LOS geometry of the engagement
 * @param params  the capture law parameters
 * @return the commanded closing speed [m/s], in [0, max_closing_speed]
 */
double closureSpeed(const LosKinematics & los, const CaptureLawParams & params);

/**
 * @brief The terminal capture law: the velocity setpoint.
 *
 *     v_M = v_T + u * v_close(r) + a_pn * tau
 *
 * The target's velocity is a feedforward, so the quantity the law shapes is the
 * relative velocity: the interceptor flies with the target and closes on it, instead
 * of flying at it. The closure along the line of sight is governed by the profile
 * (see closureSpeed()), and the PN lateral term is the only lateral authority.
 *
 * @param los                     output of computeLosKinematics()
 * @param target_velocity         the target's velocity v_T [m/s], world frame
 * @param guidance_accel          the PN acceleration a_pn [m/s^2], world frame
 * @param params                  the capture law parameters
 * @param accel_to_velocity_gain  s, how much the acceleration bends the velocity
 * @return velocity setpoint [m/s], world frame
 */
Eigen::Vector3d captureVelocitySetpoint(
  const LosKinematics & los, const Eigen::Vector3d & target_velocity,
  const Eigen::Vector3d & guidance_accel, const CaptureLawParams & params,
  double accel_to_velocity_gain);

/**
 * PN guidance law: LOS kinematics -> selected law -> saturation.
 * Stateless apart from its parameters; the phase logic lives in the node.
 */
class PnGuidance
{
public:
  explicit PnGuidance(const GuidanceParams & params = GuidanceParams());

  GuidanceResult update(const EngagementState & state) const;

  /// The velocity setpoint of the terminal capture law, for the current engagement.
  Eigen::Vector3d velocitySetpoint(
    const EngagementState & state, const GuidanceResult & result,
    double accel_to_velocity_gain) const;

  const GuidanceParams & params() const {return params_;}

private:
  GuidanceParams params_;
};

}  // namespace guidance
}  // namespace interceptor
