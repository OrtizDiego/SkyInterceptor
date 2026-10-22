/**
 * @file follow_planner.hpp
 * @brief The FOLLOW planner: framing presets in the target's heading frame, a filtered
 *        heading and the lost-track state machine. Plain C++ and Eigen, no ROS, so the
 *        geometry can be unit tested on synthetic poses.
 *
 * Frames and units: every vector is in the world frame (ENU, metres, seconds), the
 * same frame as /odom and /target/state.
 *
 * What it does
 * ------------
 * The plan (section 4.2) frames the target from a preset expressed in the *target's
 * heading frame*:
 *
 *     p_des = p_t + R_z(psi_t) . [dx, dy, dz]
 *     v_des = v_t + K_p (p_des - p)              capped at 15 m/s
 *     yaw   = the bearing from the drone to the target
 *
 * The presets are, with the offsets of follow_params.yaml:
 *
 *     BEHIND   behind the target, along its heading        [-behind,  0, height]
 *     SIDE     beside the target, laterally                 [ 0,  side, height]
 *     LEAD     in front of the target, looking back at it   [ behind, 0, height]
 *     ORBIT    circling the target at a fixed angular rate, R_z(psi_t + theta + pi)
 *              applied to [behind, 0, height]
 *
 * ORBIT starts where BEHIND does (theta = 0) and walks the framing point around the
 * target at orbit_rate, in the target's heading frame, so the circle follows the
 * target's own turns instead of staying fixed in the world.
 *
 * Three things the formula hides
 * ------------------------------
 * 1. *The heading is filtered.* The tracker's heading is the direction of the target's
 *    velocity: it rotates fast when the target turns and it jumps by 180 degrees when
 *    a noisy velocity flips sign. The preset frame is therefore built from a low-pass
 *    filtered heading (time constant heading_tau, 0.8 s by default), so the framing
 *    point slides around the target instead of teleporting from one side to the other.
 *    The *commanded yaw* does not use the filtered heading at all: it is the geometric
 *    bearing to the target, so the camera keeps the target in the middle of the frame
 *    whatever the target does.
 * 2. *The target's velocity is a feed-forward.* Without the v_t term the drone lags a
 *    fast target by v_t / K_p metres; with it, a target that moves at a constant
 *    velocity is followed exactly. The cost is that the drone inherits the target's
 *    speed, which is why the whole command is capped.
 * 3. *The velocity is capped twice.* The vertical component at max_vertical_speed (the
 *    safety filter caps it too) and the whole vector at max_speed (15 m/s, the plan's
 *    cap and the safety filter's cap for FOLLOW).
 *
 * The lost-track machine
 * ----------------------
 * A target is *usable* when a message arrived recently (target_timeout), the track is
 * confirmed and valid, and the track is the one the operator asked for. Anything else
 * counts as a lost track, and the time since the last usable target drives the state
 * machine:
 *
 *     [0, lost_hold_time)      COAST   keep flying the last command
 *     [lost_hold_time, ...)    HOLD    hover: zero velocity, hold the position
 *     [lost_rtl_time, ...)     RTL     fly back to the home position
 *
 * COAST is the grace period for a momentary dropout (a one frame miss, an occlusion):
 * the last command is the best guess available and the drone keeps flying it. HOLD is
 * the safe answer to a target that is really gone: the drone stops where it is rather
 * than chasing a stale position. RTL is the last resort, after ten seconds of a lost
 * target.
 *
 * The home position is the position the drone held on the first usable frame of the
 * follow session, i.e. where the follow started. It is captured when the mode enters
 * FOLLOW and thrown away when the mode leaves it, so every follow session has its own
 * home. If no home is known - the target was never seen, so the follow never started
 * - the drone keeps holding instead of guessing a position to fly to.
 *
 * The mode gate
 * -------------
 * The planner only has something to say in the FOLLOW mode. In HOLD and INTERCEPT the
 * command is *inactive* and the node publishes nothing at all: the intercept guidance
 * owns /setpoint/raw in INTERCEPT, and a second publisher on that topic would make
 * the safety filter alternate between two commands. The safety filter brakes to hover
 * on its own when the setpoint goes stale, so a silent planner is safe.
 */

#pragma once

#include <Eigen/Dense>

#include <cstdint>
#include <optional>
#include <string>

namespace interceptor
{
namespace follow
{

// The FlightSetpoint source values (interceptor_interfaces/msg/FlightSetpoint.msg).
// They are repeated here so the library does not depend on the message package.
constexpr uint8_t kSourceHold = 0;
constexpr uint8_t kSourceFollow = 1;
constexpr uint8_t kSourceIntercept = 2;
constexpr uint8_t kSourceRtl = 3;

/// The framing preset of plan section 4.2.
enum class FramingPreset
{
  BEHIND,  // behind the target, along its heading
  SIDE,    // beside the target, laterally
  LEAD,    // in front of the target, looking back at it
  ORBIT,   // circling the target at a fixed angular rate
};

/// "BEHIND" | "SIDE" | "LEAD" | "ORBIT" (case-insensitive); std::nullopt otherwise.
std::optional<FramingPreset> parseFramingPreset(const std::string & name);
std::string toString(FramingPreset preset);

/// The mission modes, as far as the planner is concerned.
enum class FollowMode
{
  HOLD,
  FOLLOW,
  INTERCEPT,
};

/// What the planner is doing. The node maps it to a FlightSetpoint source.
enum class FollowPhase
{
  IDLE,      // the mode is not FOLLOW: the planner has nothing to say
  TRACKING,  // the target is usable and is being followed
  COAST,     // the target went missing, but not for long enough to hold yet
  HOLD,      // the target is lost: hover
  RTL,       // the target is lost for a long time: fly back to the home position
};

struct FollowPlannerParams
{
  FramingPreset preset = FramingPreset::BEHIND;

  // The framing offsets, in the target's heading frame [m]
  double offset_behind = 8.0;   // the standoff from the target (BEHIND, LEAD, ORBIT)
  double offset_side = 6.0;     // the lateral offset (SIDE)
  double offset_height = 5.0;   // above the target

  double orbit_rate = 0.2;      // rad/s, the ORBIT angular rate
  double heading_tau = 0.8;     // s, the low-pass time constant of the target's heading
  double kp_pos = 0.8;          // 1/s, the gain of the position term

  // Limits
  double max_speed = 15.0;          // m/s, the cap on the whole command
  double max_vertical_speed = 5.0;  // m/s, the cap on the vertical component

  // The lost-track machine [s]
  double lost_hold_time = 2.0;
  double lost_rtl_time = 10.0;

  // A target message older than this counts as a lost track [s]
  double target_timeout = 0.5;

  // Below this speed the target's heading is meaningless (the tracker holds the last
  // one), so the heading filter is not updated [m/s]
  double heading_min_speed = 0.3;
};

/// The target the planner follows, as it comes from /target/state.
struct FollowTarget
{
  int track_id = -1;                          // -1: no track selected
  Eigen::Vector3d position = Eigen::Vector3d::Zero();  // m
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();  // m/s
  double heading = 0.0;                       // rad, the target's heading
  bool confirmed = false;                     // the track passed the M-of-N confirmation
  bool is_valid = false;                      // the track is fresh, not coasting
};

/// The drone's own state, from /odom. World frame.
struct FollowState
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();  // m
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();  // m/s
  double yaw = 0.0;                                    // rad
};

/// The command for one cycle.
struct FollowCommand
{
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();  // m/s, the velocity setpoint
  double yaw = 0.0;                                    // rad, the world heading
  uint8_t source = kSourceHold;      // the FlightSetpoint source of this command
  FollowPhase phase = FollowPhase::IDLE;
  /// False when the planner has nothing to say: the mode is not FOLLOW. The node
  /// publishes nothing at all in that case.
  bool active = false;
  /// The framing point, the position the preset asks for. Telemetry and tests.
  Eigen::Vector3d desired_position = Eigen::Vector3d::Zero();
};

/**
 * @brief The follow planner: framing presets, a filtered heading and the lost-track
 *        state machine.
 *
 * The class is not thread safe and expects to be driven from one timer. The node
 * around it only does the plumbing: the subscriptions, the timer and the publishing.
 */
class FollowPlanner
{
public:
  explicit FollowPlanner(const FollowPlannerParams & params = FollowPlannerParams());

  const FollowPlannerParams & params() const {return params_;}

  /// The framing preset. Takes effect on the next update.
  void setPreset(FramingPreset preset) {params_.preset = preset;}
  FramingPreset preset() const {return params_.preset;}

  /// The operator-selected track id, or -1 for the track the tracker selected. When it
  /// is set, only that exact track is followed: another track on /target/state is
  /// treated as a lost target, never as a target to follow.
  void setOperatorTrackId(int track_id) {operator_track_id_ = track_id;}
  int operatorTrackId() const {return operator_track_id_;}

  /**
   * @brief The mission mode.
   *
   * Entering FOLLOW starts a new follow session: the home position is forgotten (it
   * is captured again on the first usable frame) and the lost-track machine is
   * re-armed, so the timer of a previous session does not leak into this one.
   *
   * @param mode    the new mode
   * @param time_s  the current time on the node clock [s]
   */
  void setMode(FollowMode mode, double time_s);

  /**
   * @brief A new target from the tracker.
   *
   * The message is stored as it is: a track that is not confirmed, not valid or not
   * selected (track_id < 0) is stored too, and is simply not usable.
   *
   * @param target  the target state
   * @param time_s  the current time on the node clock [s]
   */
  void onTarget(const FollowTarget & target, double time_s);

  /**
   * @brief The command for this cycle.
   *
   * @param state   the drone's own state
   * @param time_s  the current time on the node clock [s]
   * @return the command; `active` is false when the mode is not FOLLOW.
   */
  FollowCommand update(const FollowState & state, double time_s);

  /// The framing point of the current preset, for the current target and heading.
  Eigen::Vector3d desiredPosition() const;

  FollowPhase phase() const {return phase_;}
  FollowMode mode() const {return mode_;}
  /// The home position of the current follow session, if there is one.
  bool hasHome() const {return home_.has_value();}
  Eigen::Vector3d home() const {return home_.value_or(Eigen::Vector3d::Zero());}
  /// The filtered heading of the target [rad].
  double smoothedHeading() const {return heading_filtered_;}
  /// How long the target has been lost [s].
  double lostDuration() const {return lost_duration_;}
  /// The orbit phase, in the target's heading frame [rad].
  double orbitPhase() const {return orbit_phase_;}

private:
  /// True when the target can be followed right now.
  bool targetUsable(double time_s) const;
  /// The low-pass filter on the target's heading.
  void filterHeading(const FollowTarget & target, double dt);
  /// The velocity command, with the caps applied.
  Eigen::Vector3d capVelocity(const Eigen::Vector3d & velocity) const;
  /// The bearing from one point to another [rad]; the fallback when they coincide.
  static double bearing(
    const Eigen::Vector3d & from, const Eigen::Vector3d & to, double fallback);

  FollowPlannerParams params_;

  FollowMode mode_ = FollowMode::HOLD;
  FollowPhase phase_ = FollowPhase::IDLE;

  // The last target message and when it arrived
  FollowTarget target_;
  bool has_target_ = false;
  double target_time_ = 0.0;

  int operator_track_id_ = -1;

  // The filtered heading of the target and the orbit phase
  double heading_filtered_ = 0.0;
  bool has_heading_ = false;
  double orbit_phase_ = 0.0;

  // Where the follow started: the RTL destination
  std::optional<Eigen::Vector3d> home_;

  // The lost-track machine
  double last_good_time_ = 0.0;
  bool has_good_time_ = false;
  double lost_duration_ = 0.0;

  // The last command, kept for the coast phase
  Eigen::Vector3d last_velocity_ = Eigen::Vector3d::Zero();
  double last_yaw_ = 0.0;
  bool has_last_command_ = false;

  // The clock of the last update
  double last_time_ = 0.0;
  bool has_last_time_ = false;
};

}  // namespace follow
}  // namespace interceptor
