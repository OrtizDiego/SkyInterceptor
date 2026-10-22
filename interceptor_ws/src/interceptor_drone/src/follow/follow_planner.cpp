/**
 * @file follow_planner.cpp
 * @brief The follow planner: framing presets, a filtered heading and the lost-track
 *        state machine. Plain C++ and Eigen, no ROS.
 */

#include "follow/follow_planner.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>

#include "common/math_utils.hpp"

namespace interceptor
{
namespace follow
{

namespace
{
// The longest time step the filters integrate over. A longer gap (a stalled timer, a
// simulation pause) is treated as one step, so the heading filter cannot jump.
constexpr double kMaxTimeStep = 0.1;  // s
// Below this length the vector between two points has no meaningful direction.
constexpr double kMinBearingLength = 1e-6;  // m
constexpr double kPi = 3.14159265358979323846;
}  // namespace

std::optional<FramingPreset> parseFramingPreset(const std::string & name)
{
  std::string lowered;
  lowered.reserve(name.size());
  for (const char c : name) {
    lowered.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
  }
  if (lowered == "behind") {
    return FramingPreset::BEHIND;
  }
  if (lowered == "side") {
    return FramingPreset::SIDE;
  }
  if (lowered == "lead") {
    return FramingPreset::LEAD;
  }
  if (lowered == "orbit") {
    return FramingPreset::ORBIT;
  }
  return std::nullopt;
}

std::string toString(FramingPreset preset)
{
  switch (preset) {
    case FramingPreset::BEHIND:
      return "BEHIND";
    case FramingPreset::SIDE:
      return "SIDE";
    case FramingPreset::LEAD:
      return "LEAD";
    case FramingPreset::ORBIT:
      return "ORBIT";
  }
  return "unknown";
}

FollowPlanner::FollowPlanner(const FollowPlannerParams & params)
: params_(params)
{
}

void FollowPlanner::setMode(FollowMode mode, double time_s)
{
  if (mode == mode_) {
    return;
  }
  const bool entering_follow = (mode == FollowMode::FOLLOW) && (mode_ != FollowMode::FOLLOW);
  mode_ = mode;

  if (entering_follow) {
    // A new follow session. The home is captured again on the first usable frame, and
    // the lost-track timer restarts, so the state of a previous session cannot leak
    // into this one.
    home_.reset();
    has_heading_ = false;
    orbit_phase_ = 0.0;
    has_last_command_ = false;
    last_good_time_ = time_s;
    has_good_time_ = true;
    lost_duration_ = 0.0;
  }

  if (mode != FollowMode::FOLLOW) {
    phase_ = FollowPhase::IDLE;
  }
}

void FollowPlanner::onTarget(const FollowTarget & target, double time_s)
{
  target_ = target;
  has_target_ = true;
  target_time_ = time_s;
}

bool FollowPlanner::targetUsable(double time_s) const
{
  if (!has_target_) {
    return false;
  }
  // The message is stale: the tracker stopped publishing.
  if (time_s - target_time_ > params_.target_timeout) {
    return false;
  }
  // No track is selected (the tracker publishes track_id -1 when there is none).
  if (target_.track_id < 0) {
    return false;
  }
  // A tentative track, or one that is coasting past its timeout, is not a target.
  if (!target_.confirmed || !target_.is_valid) {
    return false;
  }
  // The operator picked a track: only that one is followed. Another track is not a
  // substitute, it is a lost target.
  if (operator_track_id_ >= 0 && target_.track_id != operator_track_id_) {
    return false;
  }
  return true;
}

void FollowPlanner::filterHeading(const FollowTarget & target, double dt)
{
  if (!has_heading_) {
    // The first reading is taken as it is: the tracker already holds the heading of a
    // slow target instead of publishing noise.
    heading_filtered_ = target.heading;
    has_heading_ = true;
    return;
  }
  // A target that is barely moving has no meaningful heading, so the filter is held.
  if (target.velocity.head<2>().norm() < params_.heading_min_speed) {
    return;
  }
  if (params_.heading_tau <= 0.0 || dt <= 0.0) {
    heading_filtered_ = target.heading;
    return;
  }
  // The exponential low pass, with the angle difference taken the short way round.
  const double error = wrapAngle(target.heading - heading_filtered_);
  const double gain = std::min(1.0, dt / params_.heading_tau);
  heading_filtered_ = wrapAngle(heading_filtered_ + error * gain);
}

Eigen::Vector3d FollowPlanner::desiredPosition() const
{
  // The offset in the target's heading frame.
  Eigen::Vector3d offset(0.0, 0.0, params_.offset_height);
  double angle = heading_filtered_;

  switch (params_.preset) {
    case FramingPreset::BEHIND:
      offset.x() = -params_.offset_behind;
      break;
    case FramingPreset::SIDE:
      offset.y() = params_.offset_side;
      break;
    case FramingPreset::LEAD:
      offset.x() = params_.offset_behind;
      break;
    case FramingPreset::ORBIT:
      // The orbit starts behind the target (theta = 0) and walks around it.
      offset.x() = params_.offset_behind;
      angle = heading_filtered_ + orbit_phase_ + kPi;
      break;
  }

  const double c = std::cos(angle);
  const double s = std::sin(angle);
  const Eigen::Vector3d rotated(
    c * offset.x() - s * offset.y(),
    s * offset.x() + c * offset.y(),
    offset.z());
  return target_.position + rotated;
}

Eigen::Vector3d FollowPlanner::capVelocity(const Eigen::Vector3d & velocity) const
{
  Eigen::Vector3d capped = velocity;
  // The vertical speed is limited on its own, like the safety filter does, so a big
  // horizontal error cannot buy a fast descent.
  capped.z() = std::clamp(capped.z(), -params_.max_vertical_speed, params_.max_vertical_speed);
  // And the whole command is limited: the plan's cap, and the safety filter's cap for
  // the FOLLOW mode.
  return saturateVector(capped, params_.max_speed);
}

double FollowPlanner::bearing(
  const Eigen::Vector3d & from, const Eigen::Vector3d & to, double fallback)
{
  const double dx = to.x() - from.x();
  const double dy = to.y() - from.y();
  if (std::abs(dx) < kMinBearingLength && std::abs(dy) < kMinBearingLength) {
    // The drone is on top of the target: there is no direction to point at.
    return fallback;
  }
  return std::atan2(dy, dx);
}

FollowCommand FollowPlanner::update(const FollowState & state, double time_s)
{
  FollowCommand command;

  double dt = has_last_time_ ? time_s - last_time_ : 0.0;
  last_time_ = time_s;
  has_last_time_ = true;
  dt = std::clamp(dt, 0.0, kMaxTimeStep);

  if (mode_ != FollowMode::FOLLOW) {
    // The mode is not FOLLOW: the planner has nothing to say. The node publishes
    // nothing, so the planner can never fight the intercept guidance for the
    // /setpoint/raw topic.
    phase_ = FollowPhase::IDLE;
    command.active = false;
    command.phase = phase_;
    command.yaw = state.yaw;
    return command;
  }

  if (!has_good_time_) {
    // The first update in this session: the lost-track timer starts now.
    last_good_time_ = time_s;
    has_good_time_ = true;
  }

  if (targetUsable(time_s)) {
    lost_duration_ = 0.0;
    last_good_time_ = time_s;
    filterHeading(target_, dt);
    if (!home_.has_value()) {
      // The home is where the follow started: the position of the first usable frame.
      home_ = state.position;
    }
    phase_ = FollowPhase::TRACKING;
  } else {
    lost_duration_ = time_s - last_good_time_;
    if (lost_duration_ < params_.lost_hold_time) {
      // The grace period for a momentary dropout: the last command is the best guess
      // available.
      phase_ = FollowPhase::COAST;
    } else if (lost_duration_ < params_.lost_rtl_time || !home_.has_value()) {
      // The target is gone. Hover. Without a home there is nowhere to go back to, so
      // the drone holds instead of guessing a position.
      phase_ = FollowPhase::HOLD;
    } else {
      phase_ = FollowPhase::RTL;
    }
  }

  switch (phase_) {
    case FollowPhase::TRACKING: {
        const Eigen::Vector3d p_des = desiredPosition();
        // The target's own velocity is a feed-forward, the position term closes what is
        // left of the error.
        const Eigen::Vector3d velocity =
          target_.velocity + params_.kp_pos * (p_des - state.position);
        command.velocity = capVelocity(velocity);
        // The camera looks at the target, always: the bearing does not depend on the
        // filtered heading, so a target that turns does not whip the camera around.
        command.yaw = bearing(state.position, target_.position, state.yaw);
        command.source = kSourceFollow;
        command.desired_position = p_des;
        last_velocity_ = command.velocity;
        last_yaw_ = command.yaw;
        has_last_command_ = true;
        if (params_.preset == FramingPreset::ORBIT) {
          // The orbit phase is advanced after the framing point is computed, so the
          // first frame of an orbit is exactly the BEHIND position.
          orbit_phase_ = wrapAngle(orbit_phase_ + params_.orbit_rate * dt);
        }
        break;
      }
    case FollowPhase::COAST: {
        command.velocity = has_last_command_ ? last_velocity_ : Eigen::Vector3d::Zero();
        command.yaw = has_last_command_ ? last_yaw_ : state.yaw;
        command.source = has_last_command_ ? kSourceFollow : kSourceHold;
        break;
      }
    case FollowPhase::HOLD: {
        // Hover: zero velocity, so the drone stops where it is.
        command.velocity = Eigen::Vector3d::Zero();
        command.yaw = state.yaw;
        command.source = kSourceHold;
        break;
      }
    case FollowPhase::RTL: {
        const Eigen::Vector3d destination = home_.value();
        const Eigen::Vector3d velocity = params_.kp_pos * (destination - state.position);
        command.velocity = capVelocity(velocity);
        command.yaw = bearing(state.position, destination, state.yaw);
        command.source = kSourceRtl;
        break;
      }
    case FollowPhase::IDLE:
      break;
  }

  command.phase = phase_;
  command.active = true;
  return command;
}

}  // namespace follow
}  // namespace interceptor
