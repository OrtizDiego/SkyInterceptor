#include "safety/engagement_gate.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace interceptor
{
namespace safety
{

namespace
{

/// The TargetClass value of a uav. The values match the msg constants:
/// person=0, car=1, truck=2, bicycle=3, uav=4.
constexpr int kUavClass = 4;

/// The name of a class, for the reason string.
std::string className(int class_id)
{
  switch (class_id) {
    case 0: return "person";
    case 1: return "car";
    case 2: return "truck";
    case 3: return "bicycle";
    case 4: return "uav";
    default: break;
  }
  return "unknown";
}

/// True when the class is a person or a vehicle: the classes the interceptor
/// keeps out of, and the only classes a bystander can have.
bool isGroundClass(int class_id)
{
  return class_id >= 0 && class_id <= 3;
}

}  // namespace

EngagementGate::EngagementGate(const EngagementParams & params)
: params_(params)
{
}

void EngagementGate::reset()
{
  gate_latched_ = false;
  abort_latched_ = false;
  engaging_ = false;
  confirmed_cycles_ = 0;
  confirmed_track_id_ = -1;
  reason_.clear();
}

bool EngagementGate::isUavClass(int class_id)
{
  return class_id == kUavClass;
}

bool EngagementGate::isKnownClass(int class_id)
{
  return class_id >= 0 && class_id <= 4;
}

bool EngagementGate::insideGeofence(const Eigen::Vector3d & position) const
{
  const double margin = params_.geofence_margin;
  return position.x() >= params_.geofence_x_min + margin &&
         position.x() <= params_.geofence_x_max - margin &&
         position.y() >= params_.geofence_y_min + margin &&
         position.y() <= params_.geofence_y_max - margin;
}

EngagementGate::ClassVerdict EngagementGate::classIsEngageable(
  const EngagementContext & context, std::string & reason) const
{
  // The class of the selected target has to be a uav. A class that is not
  // known yet (-1) is not a uav: an unknown class is refused, never assumed.
  // It is not fatal though, the class may still arrive.
  if (!isKnownClass(context.target_class_id)) {
    reason = "the class of the selected target is not known yet";
    return ClassVerdict::UNKNOWN;
  }
  if (!isUavClass(context.target_class_id)) {
    reason = "the selected target is a " + className(context.target_class_id) +
      ", not a uav";
    return ClassVerdict::WRONG_CLASS;
  }
  // The cross-check against the track list. The tracker knows the class of
  // every track it publishes, so when the track with the selected id is
  // reported as something else, that is the truth: the class arrived late or
  // it changed. The gate closes instead of believing the selection.
  for (const KeepOutTarget & track : context.tracks) {
    if (track.track_id != context.target_track_id) {
      continue;
    }
    if (!isKnownClass(track.class_id)) {
      reason = "the class of the selected target is not known yet";
      return ClassVerdict::UNKNOWN;
    }
    if (!isUavClass(track.class_id)) {
      reason = "the track list reports the selected target as a " +
        className(track.class_id) + ", not a uav";
      return ClassVerdict::WRONG_CLASS;
    }
    return ClassVerdict::ENGAGEABLE;
  }
  // The track is not in the list (it was just dropped): the class of the
  // selected target is all there is.
  return ClassVerdict::ENGAGEABLE;
}

bool EngagementGate::bystanderTooClose(
  const EngagementContext & context, std::string & reason) const
{
  const double radius = params_.abort_ground_radius;
  if (!(radius > 0.0)) {
    return false;
  }
  for (const KeepOutTarget & track : context.tracks) {
    if (!isGroundClass(track.class_id)) {
      continue;  // a uav is not a bystander
    }
    if (track.track_id == context.target_track_id) {
      continue;  // the engaged target is not a bystander
    }
    // The distance to the interceptor, and to the capture point. The capture
    // happens at the intruder's position, so a bystander next to it would
    // force the interceptor to break the safety distance to finish.
    //
    // The distance is measured in 3D, like the safety distance it protects: a
    // bystander 12 m below the interceptor is 12 m away, not 0. A horizontal
    // radius would abort every engagement over a crowd, which is the same
    // "no altitude exemption" mistake the keep-out had.
    const double to_interceptor = (track.position - context.interceptor_position).norm();
    const double to_capture = (track.position - context.target_position).norm();
    if (to_interceptor <= radius || to_capture <= radius) {
      const bool near_interceptor = to_interceptor <= to_capture;
      const double distance = near_interceptor ? to_interceptor : to_capture;
      reason = "a " + className(track.class_id) + " (track " +
        std::to_string(track.track_id) + ") is " + std::to_string(distance) +
        " m from the " + (near_interceptor ? "interceptor" : "capture point") +
        ", the abort radius is " + std::to_string(radius) + " m";
      return true;
    }
  }
  return false;
}

void EngagementGate::latchAbort(const std::string & reason)
{
  abort_latched_ = true;
  gate_latched_ = true;
  engaging_ = false;
  reason_ = reason;
}

EngagementDecision EngagementGate::update(const EngagementContext & context)
{
  EngagementDecision decision;

  if (!context.intercept_mode) {
    // The gate only applies to INTERCEPT. In the other modes the filter's
    // keep-out barriers and the flight limits are the rules, and the e-stop is
    // the filter's own latch: there is no engagement here to abort.
    decision.engage_allowed = true;
    return decision;
  }

  // The e-stop is the operator's last word: it aborts the engagement, and the
  // abort is latched, so the engagement does not resume when the e-stop is
  // released. A new mission (a mode change or a re-arm) is needed to fly
  // again.
  if (context.estop) {
    latchAbort("the e-stop was pulled");
  }

  if (abort_latched_) {
    decision.aborted = true;
    decision.latched = true;
    decision.reason = reason_;
    return decision;
  }
  if (gate_latched_) {
    decision.latched = true;
    decision.reason = reason_;
    return decision;
  }

  // The confirmation counter: the number of consecutive cycles the target has
  // been confirmed. It is restarted when the selection changes.
  if (context.target_track_id != confirmed_track_id_) {
    confirmed_track_id_ = context.target_track_id;
    confirmed_cycles_ = 0;
  }
  if (context.target_confirmed) {
    ++confirmed_cycles_;
  } else {
    confirmed_cycles_ = 0;
  }

  std::string reason;
  bool allowed = false;
  bool fatal = false;  // a violation that no mission can repair
  // The altitude the interceptor has to keep to: it may be at most
  // engage_altitude_band below the intruder to start the engagement, and the
  // hysteresis further below once the engagement is running.
  const double interceptor_floor =
    context.target_position.z() - params_.engage_altitude_band -
    (engaging_ ? params_.engage_altitude_hysteresis : 0.0);

  if (params_.require_armed && !context.armed) {
    reason = "the mission is not armed";
  } else if (context.target_track_id < 0) {
    reason = "no target is selected";
  } else {
    const ClassVerdict verdict = classIsEngageable(context, reason);
    if (verdict == ClassVerdict::WRONG_CLASS) {
      // The class rule: absolute, and latched for the rest of the mission.
      fatal = true;
    } else if (verdict == ClassVerdict::UNKNOWN) {
      // The class has not arrived yet: refuse, but do not latch. The class may
      // still be reported a moment later.
      reason = "the class of the selected target is not known yet";
    } else if (!context.target_confirmed) {
      reason = "the target is not confirmed";
    } else if (confirmed_cycles_ < params_.min_confirmed_frames) {
      reason = "the target is not confirmed for enough frames yet";
    } else if (!context.target_valid) {
      reason = "the target track is not valid";
    } else if (context.target_position.z() < params_.min_engage_altitude_agl) {
      reason = "the intruder is below " +
        std::to_string(params_.min_engage_altitude_agl) + " m";
    } else if (context.interceptor_position.z() < interceptor_floor) {
      reason = "the interceptor is more than " +
        std::to_string(params_.engage_altitude_band) + " m below the intruder";
    } else if (!insideGeofence(context.target_position)) {
      reason = "the intruder is outside the geofence";
    } else if (!insideGeofence(context.interceptor_position)) {
      reason = "the interceptor is outside the geofence";
    } else {
      allowed = true;
    }
  }

  if (!allowed) {
    if (fatal) {
      // A person is never engaged: the gate latches closed for the mission.
      gate_latched_ = true;
      reason_ = reason;
      decision.latched = true;
      decision.reason = reason_;
      if (engaging_) {
        // The target changed its class in the middle of the engagement: the
        // engagement has to stop.
        latchAbort(reason);
        decision.aborted = true;
      }
      return decision;
    }
    if (engaging_) {
      // A gate condition that was met when the engagement started is not met
      // any more: the engagement is broken off.
      latchAbort("the engagement was aborted: " + reason);
      decision.aborted = true;
      decision.latched = true;
      decision.reason = reason_;
      return decision;
    }
    // The mission has not started yet: this is not a latch, the condition may
    // be met a moment later. An armed mission may climb meanwhile, so the
    // interceptor can reach the altitude band the gate asks for.
    decision.reason = reason;
    decision.climb_allowed = !params_.require_armed || context.armed;
    return decision;
  }

  // The gate is open. From here on the engagement is running, and a gate
  // condition that is violated again aborts it.
  engaging_ = true;
  decision.engage_allowed = true;

  std::string abort_reason;
  if (bystanderTooClose(context, abort_reason)) {
    latchAbort("the engagement was aborted: " + abort_reason);
    decision.engage_allowed = false;
    decision.aborted = true;
    decision.latched = true;
    decision.reason = reason_;
  }
  return decision;
}

}  // namespace safety
}  // namespace interceptor
