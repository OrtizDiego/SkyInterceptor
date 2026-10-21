/**
 * @file safety_filter.hpp
 * @brief The keep-out safety filter: a control barrier function on the velocity, plus
 *        the global flight limits. Plain C++ and Eigen, no ROS, so the projection can be
 *        unit tested on synthetic geometry.
 *
 * Frames and units: every vector is in the world frame (ENU, metres, seconds), the same
 * frame as /odom and /target/state.
 *
 * What it does
 * ------------
 * The filter sits between the planners and the trajectory controller and has the final
 * say: whatever the planner commands, the drone never comes closer to a person, a
 * vehicle or a static obstacle than the configured standoff.
 *
 * It is a *velocity* filter, because the controller flies a velocity loop. A position
 * setpoint is first turned into a velocity (v = kp * (p_sp - p) + v_ff), so everything
 * after that is a single velocity command.
 *
 * The keep-out barrier
 * --------------------
 * For every person or vehicle track, and for every static obstacle, the filter builds a
 * half-space the velocity has to stay in. For a track:
 *
 *     h  = |p_xy - p_t,xy| - d_eff              (d_eff = d_min + k * sigma_pos)
 *     n  = (p_xy - p_t,xy) / |p_xy - p_t,xy|    (unit vector from the track to the drone)
 *
 * The plan's condition (section 4.3) is h_dot >= -alpha * h. Since h_dot is minus the
 * closing speed c, that is an upper bound on the closing speed, c_cmd <= alpha * h, and
 * in the velocity it is the half-space
 *
 *     n . (v_xy - v_t,xy) >= -c_cmd_max
 *
 * The subtlety is the *relative degree*. h_dot depends on the closing speed the drone
 * actually has, not on the one it is told to have, and the autopilot's velocity loop
 * lags: the time constant measured on the X500 flight model is 0.34 s. A barrier that
 * only constrains the command is therefore satisfied instantly and then coasted past.
 * Measured on the flight model, that overshoots the keep-out margin by up to 6 m
 * against a 5 m standoff.
 *
 * So the barrier is relative degree 2, on the achieved motion:
 *
 *     h_ddot + 2*w*h_dot + w^2*h >= 0
 *
 * With c_dot = (c_cmd - c)/tau and h_dot = -c this becomes
 *
 *     c_cmd <= c*(1 - 2*w*tau) + w^2*tau*h
 *
 * The filter uses the stricter of that and the plan's own bound:
 *
 *     c_cmd_max = min( alpha*h , c*(1 - 2*w*tau) + w^2*tau*h )
 *
 * The first term is the plan's exponential approach, the second is the lag correction.
 * The correction bites exactly when the drone is closing too fast to stop in time: the
 * term c*(1 - 2*w*tau) is negative, so the bound goes below zero and the drone is told
 * to open the distance. Numerically (a sweep over the plant's time constant, over a
 * mismatch between the assumed and the real one, and with noise on the measured speed)
 * this holds the margin exactly for every state from which stopping is physically
 * possible, where the plan's form alone overshoots.
 *
 * Static obstacles are the same thing with a stationary target.
 *
 * Note what happens when h < 0, that is when the drone is already inside the keep-out:
 * then the bound is positive and the constraint does not just brake the drone, it pushes
 * it back out. That is the whole point of a barrier.
 *
 * Several half-spaces are satisfied by projecting onto them one after the other
 * (projection onto convex sets), iterating until every one holds or the iteration
 * budget runs out. When the intersection is empty the iteration never converges, which
 * is how a boxed-in drone is detected.
 *
 * Precedence
 * ----------
 * The keep-out wins over the flight limits. The speed cap is applied to the planner's
 * request before the barriers, and the barriers are never traded away to satisfy it: when
 * the constraints cannot all be satisfied the filter commands hover and says so in the
 * reason string. A distance is never relaxed to make a command feasible.
 */

#pragma once

#include <Eigen/Dense>

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace interceptor
{
namespace safety
{

/// A static obstacle: a cylinder standing on the ground, from safety_params.yaml.
struct Obstacle
{
  std::string name;
  double x = 0.0;       // m, world frame
  double y = 0.0;       // m, world frame
  double radius = 0.0;  // m, the obstacle's own radius
  double height = 0.0;  // m, its top; the keep-out only applies below this
};

/// A track the filter keeps out of. One per person or vehicle on /tracks.
struct KeepOutTarget
{
  int track_id = -1;
  int class_id = 0;               // TargetClass value
  Eigen::Vector3d position = Eigen::Vector3d::Zero();   // m, world frame
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();   // m/s, world frame
  /// Position uncertainty [m]: the standard deviation of the horizontal position. The
  /// effective standoff grows with it, so a poorly tracked person gets a bigger margin.
  double position_sigma = 0.0;
  /// False once the tracker stopped updating the track. A stale track still keeps its
  /// keep-out: the coasting prediction is the best estimate available.
  bool is_valid = true;
};

struct SafetyFilterParams
{
  // --- Keep-out distances -------------------------------------------------
  double d_min_horizontal = 5.0;   // m, to any person or vehicle track
  double d_obstacle_min = 3.0;      // m, to the surface of any static obstacle
  double h_min_above = 3.0;         // m, vertical clearance above a person or vehicle
  double uncertainty_gain = 2.0;    // k in d_eff = d_min + k * sigma_pos
  double drone_radius = 0.3;        // m, the drone's own radius, so the distances are
                                    // surface to surface and not centre to centre

  // --- Barrier ------------------------------------------------------------
  double cbf_alpha = 1.0;           // 1/s, the rate at which the barrier is enforced
  int max_projection_iterations = 10;
  // The assumed time constant of the autopilot's velocity loop [s]. The measured
  // value on the X500 flight model is 0.34 s; the default is deliberately larger,
  // so the filter assumes a slower vehicle than it is.
  double velocity_lag_tau = 0.5;
  // The natural frequency of the lag correction [1/s]. The correction is the relative
  // degree 2 barrier h_ddot + 2*w*h_dot + w^2*h >= 0.
  double barrier_omega = 3.0;

  // --- Flight limits ------------------------------------------------------
  double altitude_floor = 2.0;      // m AGL
  double altitude_ceiling = 120.0;  // m AGL
  double geofence_x_min = -100.0;   // m
  double geofence_x_max = 100.0;    // m
  double geofence_y_min = -100.0;   // m
  double geofence_y_max = 100.0;    // m
  double geofence_margin = 1.0;     // m, the boundary is enforced this far before it
  double speed_cap_follow = 15.0;   // m/s
  double speed_cap_intercept = 30.0;  // m/s
  double max_vertical_speed = 5.0;  // m/s

  // --- Timeouts -----------------------------------------------------------
  double setpoint_timeout = 0.3;  // s, no fresh setpoint: brake to hover
  double odom_timeout = 0.3;      // s, stale odometry: brake to hover
  double coast_timeout = 2.0;     // s, a track that disappeared keeps its keep-out
                                  // this long before the filter lets it go

  // --- Position setpoints -------------------------------------------------
  // The controller's kp_pos, used to turn a position setpoint into a velocity.
  double kp_pos = 1.0;
};

/// The mode the filter is running in. Decides the speed cap.
enum class FilterMode
{
  HOLD,
  FOLLOW,
  INTERCEPT,
};

/// The setpoint the filter works on. World frame (ENU), like FlightSetpoint.
struct FilterSetpoint
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();  // m, used when position_valid
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();  // m/s
  double yaw = 0.0;                                    // rad
  double yaw_rate = 0.0;                               // rad/s
  bool position_valid = false;
  uint8_t source = 0;  // FlightSetpoint source
};

/// The drone's own state, from /odom.
struct FilterState
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();  // m
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();  // m/s
  double yaw = 0.0;                                    // rad
};

/// What the filter decided, and why.
struct FilterResult
{
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();  // m/s, the command to the controller
  double yaw = 0.0;                                    // rad
  double yaw_rate = 0.0;                               // rad/s
  bool safety_active = false;  // true when the filter modified the planner's command
  bool feasible = true;        // false when the constraints could not all be satisfied
  std::string reason;          // why the filter intervened (empty when nominal)
};

/// The safety filter: keep-out barriers plus the global flight limits.
///
/// All the maths lives here so it can be tested without ROS. The node around it only
/// does the plumbing: subscriptions, the timer and the publishing.
class SafetyFilter
{
public:
  explicit SafetyFilter(const SafetyFilterParams & params = SafetyFilterParams());

  const SafetyFilterParams & params() const {return params_;}

  /// The speed cap that applies in the given mode [m/s].
  double speedCap(FilterMode mode) const;

  /// Filters one setpoint.
  ///
  /// Not const, because the filter remembers the tracks it keeps out of: a track that
  /// disappears from the tracker output keeps its keep-out for coast_timeout.
  ///
  /// @param setpoint        the planner's command
  /// @param state           the drone's state
  /// @param targets         the person and vehicle tracks to keep out of
  /// @param obstacles       the static obstacles to keep out of
  /// @param mode            the mission mode, for the speed cap
  /// @param time_s          the current time on the node clock [s]
  /// @param setpoint_age_s  the age of the setpoint [s]
  /// @param odom_age_s      the age of the odometry [s]
  /// @param estop_latched   true when the e-stop has been latched
  /// @param engage_allowed  false to refuse an INTERCEPT engagement (the gate, §4.4)
  /// @return the command to send to the trajectory controller
  FilterResult filter(
    const FilterSetpoint & setpoint, const FilterState & state,
    const std::vector<KeepOutTarget> & targets, const std::vector<Obstacle> & obstacles,
    FilterMode mode, double time_s, double setpoint_age_s, double odom_age_s,
    bool estop_latched, bool engage_allowed = true);

  /// A hover command: everything the filter can do, undone.
  FilterResult hold(const FilterState & state, const std::string & reason);

  /// Forgets the cached tracks and the integrator state. Call on a mission mode change.
  void reset();

  /// The tracks the filter is currently keeping out of, including the stale ones it
  /// still remembers. For the RViz keep-out circles.
  std::vector<KeepOutTarget> activeKeepOuts() const;

private:
  /// A track the filter remembers after it disappeared from the tracker output.
  struct CachedTarget
  {
    KeepOutTarget target;
    double last_seen = 0.0;  // s
  };

  /// Turns the planner's setpoint into the velocity the filter works on.
  Eigen::Vector3d requestedVelocity(
    const FilterSetpoint & setpoint, const FilterState & state) const;

  /// The effective standoff for a track: d_min inflated by the position uncertainty.
  double effectiveDistance(const KeepOutTarget & target) const;

  /// Merges the tracks from the tracker with the ones the filter still remembers, and
  /// drops the ones that have been gone for longer than coast_timeout. Returns true
  /// when a keep-out was forgotten, which makes the filter hold for that cycle.
  bool collectTargets(
    const std::vector<KeepOutTarget> & targets, double time_s,
    std::vector<KeepOutTarget> & out);

  SafetyFilterParams params_;
  std::map<int, CachedTarget> cached_targets_;
};

}  // namespace safety
}  // namespace interceptor
