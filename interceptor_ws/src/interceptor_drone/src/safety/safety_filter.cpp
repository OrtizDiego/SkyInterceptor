#include "safety/safety_filter.hpp"

#include <algorithm>
#include <cmath>

namespace interceptor
{
namespace safety
{

namespace
{

/// Only people and vehicles are keep-out targets: a uav is the thing the drone is
/// chasing in INTERCEPT, and it is never a ground obstacle.
bool isGroundClass(int class_id)
{
  // TargetClass: person=0, car=1, truck=2, bicycle=3, uav=4
  return class_id >= 0 && class_id <= 3;
}

/// The largest closing speed the drone may command towards one keep-out target.
///
/// The closing speed c is the rate at which the range shrinks, positive while the
/// drone and the target come together. The bound is the strictest of three:
///
///   - the plan's bound (section 4.3): c_cmd <= alpha * h, an exponential approach;
///   - the lag correction, a relative degree 2 barrier on the achieved motion,
///     c_cmd <= c*(1 - 2*w*tau) + w^2*tau*h;
///   - the yield: when the target is closing on the drone, the drone does not close
///     any further, so c_cmd <= 0 and the range is held or opened.
///
/// @param h        the margin above the keep-out distance [m]
/// @param closing  the achieved closing speed towards the target [m/s]
/// @param target_closing  the target's velocity along the LOS [m/s], positive while
///                    the target moves towards the drone
/// @return the largest closing speed that may be commanded [m/s]
double maxClosingSpeed(
  double h, double closing, double target_closing, double alpha, double omega, double tau)
{
  const double plan_bound = alpha * h;
  const double lag_bound =
    closing * (1.0 - 2.0 * omega * tau) + omega * omega * tau * h;
  double bound = std::min(plan_bound, lag_bound);
  if (target_closing > 0.0) {
    // The target is coming towards the drone: the drone yields and does not close any
    // further. The range is then held, and the margin with it.
    bound = std::min(bound, 0.0);
  }
  return bound;
}

}  // namespace

SafetyFilter::SafetyFilter(const SafetyFilterParams & params)
: params_(params)
{
}

double SafetyFilter::speedCap(FilterMode mode) const
{
  switch (mode) {
    case FilterMode::FOLLOW:
      return params_.speed_cap_follow;
    case FilterMode::INTERCEPT:
      return params_.speed_cap_intercept;
    case FilterMode::HOLD:
      break;
  }
  // HOLD is not a flight mode, but the filter still has to produce a legal command.
  return std::min(params_.speed_cap_follow, params_.speed_cap_intercept);
}

double SafetyFilter::effectiveDistance(const KeepOutTarget & target) const
{
  // d_eff = d_min + k * sigma_pos: a poorly tracked person gets a bigger margin.
  return params_.d_min_horizontal + params_.uncertainty_gain * target.position_sigma;
}

void SafetyFilter::reset()
{
  cached_targets_.clear();
}

std::vector<KeepOutTarget> SafetyFilter::activeKeepOuts() const
{
  std::vector<KeepOutTarget> out;
  out.reserve(cached_targets_.size());
  for (const auto & entry : cached_targets_) {
    out.push_back(entry.second.target);
  }
  return out;
}

bool SafetyFilter::collectTargets(
  const std::vector<KeepOutTarget> & targets, double time_s,
  std::vector<KeepOutTarget> & out)
{
  // Remember every ground track the tracker reports, so it can be kept out of even
  // after it disappears from the tracker output. This happens on every call, including
  // the ones that end in a hold: a track that is dropped from the cache while the
  // drone holds would be forgotten at the worst possible moment.
  for (const KeepOutTarget & target : targets) {
    if (!isGroundClass(target.class_id)) {
      continue;
    }
    CachedTarget & cached = cached_targets_[target.track_id];
    cached.target = target;
    cached.last_seen = time_s;
  }

  // Drop the tracks that have been gone for longer than coast_timeout.
  bool forgot = false;
  for (auto it = cached_targets_.begin(); it != cached_targets_.end(); ) {
    if (time_s - it->second.last_seen > params_.coast_timeout) {
      it = cached_targets_.erase(it);
      forgot = true;
    } else {
      ++it;
    }
  }

  out.clear();
  out.reserve(cached_targets_.size());
  for (const auto & entry : cached_targets_) {
    out.push_back(entry.second.target);
  }
  return forgot;
}

Eigen::Vector3d SafetyFilter::requestedVelocity(
  const FilterSetpoint & setpoint, const FilterState & state) const
{
  if (!setpoint.position_valid) {
    return setpoint.velocity;
  }
  // A position setpoint is turned into a velocity with the controller's own gain, so
  // the filter and the controller agree on how fast the setpoint is approached.
  return params_.kp_pos * (setpoint.position - state.position) + setpoint.velocity;
}

FilterResult SafetyFilter::hold(const FilterState & state, const std::string & reason)
{
  FilterResult result;
  // Hold the current position: zero velocity, so the drone stops where it is.
  result.velocity = Eigen::Vector3d::Zero();
  result.yaw = state.yaw;
  result.yaw_rate = 0.0;
  result.safety_active = true;
  result.feasible = false;
  result.reason = reason;
  return result;
}

FilterResult SafetyFilter::filter(
  const FilterSetpoint & setpoint, const FilterState & state,
  const std::vector<KeepOutTarget> & targets, const std::vector<Obstacle> & obstacles,
  FilterMode mode, double time_s, double setpoint_age_s, double odom_age_s,
  bool estop_latched, bool engage_allowed)
{
  FilterResult result;
  result.yaw = setpoint.yaw;
  result.yaw_rate = setpoint.yaw_rate;

  // --- The tracks to keep out of -------------------------------------------
  // This comes first, so the cache is kept up to date even on a cycle that holds.
  std::vector<KeepOutTarget> keep_outs;
  const bool forgot = collectTargets(targets, time_s, keep_outs);

  // --- The conditions that stop the drone dead -----------------------------
  if (estop_latched) {
    return hold(state, "e-stop latched");
  }
  if (odom_age_s > params_.odom_timeout) {
    return hold(state, "stale odometry");
  }
  if (setpoint_age_s > params_.setpoint_timeout) {
    return hold(state, "stale setpoint");
  }
  // The INTERCEPT engagement gate (section 4.4): the planner is not allowed to fly at
  // a target the filter has not cleared. The gate is enforced here so a planner bug
  // cannot bypass it.
  if (mode == FilterMode::INTERCEPT && !engage_allowed) {
    return hold(state, "engagement gate closed");
  }
  if (forgot) {
    // A keep-out was forgotten: the drone may be closer to that person than it thinks.
    // Hold for this cycle rather than continuing on a command that assumes the
    // keep-out is gone.
    return hold(state, "keep-out forgotten (track lost)");
  }

  // --- The planner's command, as a velocity -------------------------------
  Eigen::Vector3d velocity = requestedVelocity(setpoint, state);

  // The speed cap is applied to the request, before the barriers: the planner cannot
  // ask for more than the mode allows.
  const double cap = speedCap(mode);
  const double horizontal = velocity.head<2>().norm();
  if (horizontal > cap) {
    velocity.head<2>() *= cap / horizontal;
  }
  velocity.z() = std::clamp(velocity.z(), -params_.max_vertical_speed, params_.max_vertical_speed);

  // --- The keep-out barriers ----------------------------------------------
  // Each barrier is the plan's half-space (section 4.3):
  //
  //     n . (v_xy - v_t,xy) >= -c_cmd_max
  //
  // where n points from the target to the drone and c_cmd_max is the largest closing
  // speed the drone may command along the line of sight. Since
  // n . (v_xy - v_t,xy) is minus the commanded closing speed, this is an upper bound on
  // the closing speed.
  struct Barrier
  {
    Eigen::Vector2d normal;
    double target_los = 0.0;  // the target's velocity along the LOS [m/s]
    double c_cmd_max = 0.0;   // the largest closing speed that may be commanded [m/s]
    std::string name;
  };
  std::vector<Barrier> barriers;

  for (const KeepOutTarget & target : keep_outs) {
    const Eigen::Vector2d delta = state.position.head<2>() - target.position.head<2>();
    const double distance = delta.norm();
    if (distance < 1e-6) {
      // The drone is exactly on top of the track: the direction is undefined, so the
      // barrier is not well posed. Hold: the safest answer when the geometry is
      // degenerate.
      return hold(state, "on top of track " + std::to_string(target.track_id));
    }
    // n points from the target to the drone.
    const Eigen::Vector2d normal = delta / distance;
    // The standoff is measured between the surfaces, so the drone's own radius comes
    // off the distance to the track's centre.
    const double h = distance - effectiveDistance(target) - params_.drone_radius;

    // The velocities along the line of sight. n points from the target to the drone,
    // so the closing speed (the rate at which the range shrinks) is
    // n . (v_target - v_drone), and the target's own closing rate on the drone is
    // n . v_target (positive when the target moves towards the drone).
    const double target_los = normal.dot(target.velocity.head<2>());
    const double drone_los = normal.dot(state.velocity.head<2>());
    const double closing = target_los - drone_los;
    const double target_closing = target_los;

    const double c_cmd_max = maxClosingSpeed(
      h, closing, target_closing, params_.cbf_alpha,
      params_.barrier_omega, params_.velocity_lag_tau);
    barriers.push_back(
      {normal, target_los, c_cmd_max, "track " + std::to_string(target.track_id)});
  }

  for (const Obstacle & obstacle : obstacles) {
    // A short obstacle is not a hazard for a drone flying above it.
    if (obstacle.height > 0.0 && state.position.z() > obstacle.height) {
      continue;
    }
    const Eigen::Vector2d delta =
      state.position.head<2>() - Eigen::Vector2d(obstacle.x, obstacle.y);
    const double distance = delta.norm();
    if (distance < 1e-6) {
      return hold(state, "inside obstacle " + obstacle.name);
    }
    const Eigen::Vector2d normal = delta / distance;
    const double h = distance - (obstacle.radius + params_.d_obstacle_min) - params_.drone_radius;

    // Static: the obstacle does not move, so the target's velocity along the LOS is
    // zero and the closing speed is just the drone's own.
    const double closing = -normal.dot(state.velocity.head<2>());
    const double c_cmd_max = maxClosingSpeed(
      h, closing, 0.0, params_.cbf_alpha, params_.barrier_omega, params_.velocity_lag_tau);
    barriers.push_back({normal, 0.0, c_cmd_max, "obstacle " + obstacle.name});
  }

  // --- Project onto the barriers -------------------------------------------
  // Projection onto a convex set converges to a point in every set, so iterating
  // satisfies one barrier after the other. When the barriers conflict the iteration
  // does not converge, which is how a boxed-in drone is detected.
  //
  // The constraint is c_cmd <= c_cmd_max, with c_cmd = n . (v_t - v_d), so
  //
  //     n . v_d >= n . v_t - c_cmd_max
  //
  // which is a half-space in the drone's velocity. The closest point of it is on its
  // boundary, along the normal.
  const auto satisfied = [&barriers](const Eigen::Vector3d & v) {
      for (const Barrier & barrier : barriers) {
        if (barrier.normal.dot(v.head<2>()) < barrier.target_los - barrier.c_cmd_max - 1e-9) {
          return false;
        }
      }
      return true;
    };

  bool all_satisfied = true;
  std::string conflicting;
  std::string first_violation;
  for (int iteration = 0; iteration < params_.max_projection_iterations; ++iteration) {
    all_satisfied = true;
    for (const Barrier & barrier : barriers) {
      const double value = barrier.normal.dot(velocity.head<2>());
      const double bound = barrier.target_los - barrier.c_cmd_max;
      if (value < bound - 1e-9) {
        all_satisfied = false;
        if (conflicting.empty()) {
          conflicting = barrier.name;
        }
        if (first_violation.empty()) {
          first_violation = barrier.name;
        }
        velocity.x() += barrier.normal.x() * (bound - value);
        velocity.y() += barrier.normal.y() * (bound - value);
      }
    }
    if (all_satisfied) {
      break;
    }
  }

  if (!all_satisfied) {
    // The barriers could not all be satisfied: the drone is boxed in. Hover and say
    // so. A distance is never relaxed to make a command feasible.
    return hold(state, "conflicting keep-out constraints (" + conflicting + ")");
  }

  // The filter changed the command if it had to move it at all.
  if (!first_violation.empty()) {
    result.safety_active = true;
    result.reason = "keep-out from " + first_violation;
  }

  // --- Vertical keep-out ---------------------------------------------------
  // Never descend below z_t + h_min_above while horizontally inside 2 * d_min.
  for (const KeepOutTarget & target : keep_outs) {
    const double horizontal_distance =
      (state.position.head<2>() - target.position.head<2>()).norm();
    if (horizontal_distance > 2.0 * params_.d_min_horizontal) {
      continue;
    }
    const double floor = target.position.z() + params_.h_min_above;
    if (state.position.z() <= floor && velocity.z() < 0.0) {
      velocity.z() = 0.0;
      result.safety_active = true;
      result.reason = "vertical keep-out above track " + std::to_string(target.track_id);
    }
  }

  // --- The global limits ---------------------------------------------------
  // The altitude floor and ceiling: the vertical velocity is stopped at the boundary.
  if (state.position.z() <= params_.altitude_floor && velocity.z() < 0.0) {
    velocity.z() = 0.0;
    result.safety_active = true;
    result.reason = "altitude floor";
  } else if (state.position.z() >= params_.altitude_ceiling && velocity.z() > 0.0) {
    velocity.z() = 0.0;
    result.safety_active = true;
    result.reason = "altitude ceiling";
  }

  // The geofence: a velocity pointing out of the box is zeroed at the boundary.
  const double margin = params_.geofence_margin;
  if (state.position.x() <= params_.geofence_x_min + margin && velocity.x() < 0.0) {
    velocity.x() = 0.0;
    result.safety_active = true;
    result.reason = "geofence x_min";
  } else if (state.position.x() >= params_.geofence_x_max - margin && velocity.x() > 0.0) {
    velocity.x() = 0.0;
    result.safety_active = true;
    result.reason = "geofence x_max";
  }
  if (state.position.y() <= params_.geofence_y_min + margin && velocity.y() < 0.0) {
    velocity.y() = 0.0;
    result.safety_active = true;
    result.reason = "geofence y_min";
  } else if (state.position.y() >= params_.geofence_y_max - margin && velocity.y() > 0.0) {
    velocity.y() = 0.0;
    result.safety_active = true;
    result.reason = "geofence y_max";
  }

  // The speed cap is applied again, after the projection: the barrier may have pushed
  // the velocity past the cap. The keep-out wins: the cap is only applied when the
  // capped velocity still satisfies every barrier, because scaling the velocity down
  // along its own direction would break a barrier that is holding the drone off.
  const double final_horizontal = velocity.head<2>().norm();
  if (final_horizontal > cap) {
    Eigen::Vector3d capped = velocity;
    capped.head<2>() *= cap / final_horizontal;
    if (satisfied(capped)) {
      velocity = capped;
    } else {
      result.safety_active = true;
      result.reason += " (speed cap relaxed: the keep-out needs it)";
    }
  }
  velocity.z() = std::clamp(velocity.z(), -params_.max_vertical_speed, params_.max_vertical_speed);

  result.velocity = velocity;
  return result;
}

}  // namespace safety
}  // namespace interceptor
