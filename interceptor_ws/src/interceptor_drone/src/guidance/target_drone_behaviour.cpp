/**
 * @file target_drone_behaviour.cpp
 * @brief The flight behaviours of the target drone (the INTERCEPT intruder).
 *
 * See include/guidance/target_drone_behaviour.hpp for the description of the
 * behaviours and of the two properties they keep: the position is the exact integral
 * of the velocity, and the velocity is continuous.
 */

#include "guidance/target_drone_behaviour.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace interceptor
{
namespace guidance
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

/// The smallest leg or arc that still counts as a piece of path [m].
constexpr double kMinLength = 1e-9;
}  // namespace

std::optional<TargetBehaviour> parseTargetBehaviour(const std::string & name)
{
  if (name == "hover") {return TargetBehaviour::HOVER;}
  if (name == "waypoints") {return TargetBehaviour::WAYPOINTS;}
  if (name == "straight") {return TargetBehaviour::STRAIGHT;}
  if (name == "weave") {return TargetBehaviour::WEAVE;}
  if (name == "curve") {return TargetBehaviour::CURVE;}
  return std::nullopt;
}

std::string toString(TargetBehaviour behaviour)
{
  switch (behaviour) {
    case TargetBehaviour::HOVER: return "hover";
    case TargetBehaviour::WAYPOINTS: return "waypoints";
    case TargetBehaviour::STRAIGHT: return "straight";
    case TargetBehaviour::WEAVE: return "weave";
    case TargetBehaviour::CURVE: return "curve";
  }
  return "curve";
}

TargetDroneBehaviour::TargetDroneBehaviour(const TargetBehaviourParams & params)
: params_(params)
{
  reset();
}

void TargetDroneBehaviour::reset()
{
  state_ = TargetBehaviourState();
  state_.yaw = params_.start_yaw;

  if (params_.behaviour == TargetBehaviour::WAYPOINTS) {
    buildPath();
    if (!path_.empty()) {
      // The waypoints behaviour starts on its first waypoint, on the path.
      Eigen::Vector3d path_position;
      double heading = params_.start_yaw;
      double curvature = 0.0;
      evaluatePath(0.0, path_position, heading, curvature);
      const double altitude = params_.waypoints.front().z();
      state_.position = Eigen::Vector3d(path_position.x(), path_position.y(), altitude);
      state_.yaw = heading;
    } else {
      state_.position = params_.start_position;
    }
  } else {
    state_.position = params_.start_position;
  }

  // The velocity and the acceleration of the first instant.
  switch (params_.behaviour) {
    case TargetBehaviour::HOVER: updateHover(); break;
    case TargetBehaviour::WAYPOINTS: updateWaypoints(); break;
    case TargetBehaviour::STRAIGHT: updateStraight(); break;
    case TargetBehaviour::WEAVE: updateWeave(); break;
    case TargetBehaviour::CURVE: updateCurve(); break;
  }
}

const TargetBehaviourState & TargetDroneBehaviour::step(double dt)
{
  if (!(dt > 0.0)) {
    return state_;
  }
  state_.time += dt;

  switch (params_.behaviour) {
    case TargetBehaviour::HOVER: updateHover(); break;
    case TargetBehaviour::WAYPOINTS: updateWaypoints(); break;
    case TargetBehaviour::STRAIGHT: updateStraight(); break;
    case TargetBehaviour::WEAVE: updateWeave(); break;
    case TargetBehaviour::CURVE: updateCurve(); break;
  }

  // The position is the exact integral of the velocity that is published with it.
  state_.position += state_.velocity * dt;
  return state_;
}

// ---------------------------------------------------------------------------
// The behaviours
// ---------------------------------------------------------------------------

void TargetDroneBehaviour::updateHover()
{
  const double drift = std::max(params_.hover_drift, 0.0);
  if (drift <= 0.0) {
    // The drone holds its position exactly: no velocity, no acceleration.
    state_.velocity.setZero();
    state_.acceleration.setZero();
    return;
  }

  // A slow wander sideways: the drone keeps its station and drifts at most as far
  // as the radius of the drift. The drift starts and ends at rest, so the
  // velocity is continuous, and the velocity is the exact derivative of the
  // wander, so the position stays the integral of the velocity.
  const double period = std::max(params_.hover_drift_period, 1e-3);
  const double omega = kTwoPi / period;
  const double phase = omega * state_.time;
  // The direction of the wander, perpendicular to the heading of the drone.
  const double lateral_x = -std::sin(params_.start_yaw);
  const double lateral_y = std::cos(params_.start_yaw);
  const double amplitude = 0.5 * drift;
  state_.velocity = Eigen::Vector3d(
    amplitude * omega * std::sin(phase) * lateral_x,
    amplitude * omega * std::sin(phase) * lateral_y, 0.0);
  state_.acceleration = Eigen::Vector3d(
    amplitude * omega * omega * std::cos(phase) * lateral_x,
    amplitude * omega * omega * std::cos(phase) * lateral_y, 0.0);
}

void TargetDroneBehaviour::updateWaypoints()
{
  const double speed = std::max(params_.waypoint_speed, 0.0);
  if (path_.empty() || speed <= 0.0) {
    state_.velocity.setZero();
    state_.acceleration.setZero();
    return;
  }

  // The speed along the path is constant, so the arclength is the speed times the
  // time. The lead-in from the first waypoint to the loop is flown once; the loop
  // itself repeats, and it is closed, so the drone comes back to where it started.
  const double travelled = speed * state_.time;
  const bool in_lead_in = (travelled < lead_in_);
  const double s = in_lead_in ? travelled : lead_in_ + std::fmod(
    travelled - lead_in_,
    polygon_length_);
  Eigen::Vector3d path_position;
  double heading = 0.0;
  double curvature = 0.0;
  evaluatePath(s, path_position, heading, curvature);

  // The altitude profile runs along the loop. The lead-in is flat: it happens
  // once, before the loop starts.
  const double loop_s = in_lead_in ? 0.0 : (s - lead_in_);

  // The horizontal velocity follows the tangent of the path.
  const double vx = speed * std::cos(heading);
  const double vy = speed * std::sin(heading);
  const double vz = in_lead_in ? 0.0 : altitudeRateAt(loop_s) * speed;
  state_.velocity = Eigen::Vector3d(vx, vy, vz);

  // The acceleration: the centripetal term of the fillets, and the curvature of the
  // altitude profile.
  const double normal_x = -std::sin(heading);
  const double normal_y = std::cos(heading);
  const double a_normal = speed * speed * curvature;
  const double a_vertical = in_lead_in ? 0.0 : altitudeAccelerationAt(loop_s) * speed * speed;
  state_.acceleration = Eigen::Vector3d(a_normal * normal_x, a_normal * normal_y, a_vertical);
  state_.yaw = heading;
}

void TargetDroneBehaviour::updateStraight()
{
  const double t = state_.time;
  const double v_max = std::max(params_.straight_speed, 0.0);
  const double ramp = std::max(params_.straight_ramp_time, 0.0);

  // When the run ends: after the configured distance, or after the configured
  // duration, or never.
  double t_end = std::max(params_.straight_duration, 0.0);
  if (params_.straight_distance > 0.0 && v_max > 0.0) {
    // The distance of the run: the two ramps together cover what a cruise of
    // ramp seconds would, so the run lasts D / v_max + ramp.
    t_end = params_.straight_distance / v_max + ramp;
  }
  if (t_end > 0.0 && ramp > 0.0) {
    // The run has to be long enough for the two ramps.
    t_end = std::max(t_end, 2.0 * ramp);
  }

  // The speed: a raised cosine ramp up, the cruise, a raised cosine ramp down.
  // The ramp has zero slope at both ends, so the acceleration is continuous too.
  double speed = 0.0;
  double speed_rate = 0.0;
  if (v_max > 0.0) {
    const double ramp_up_end = ramp;
    const double ramp_down_start =
      (t_end > 0.0) ? t_end - ramp : std::numeric_limits<double>::infinity();
    if (ramp > 0.0 && t < ramp_up_end) {
      const double phase = kPi * t / ramp;
      speed = 0.5 * v_max * (1.0 - std::cos(phase));
      speed_rate = 0.5 * v_max * (kPi / ramp) * std::sin(phase);
    } else if (ramp > 0.0 && t > ramp_down_start) {
      const double phase = kPi * std::max(t_end - t, 0.0) / ramp;
      speed = 0.5 * v_max * (1.0 - std::cos(phase));
      speed_rate = 0.5 * v_max * (kPi / ramp) * std::sin(phase);
    } else if (t_end > 0.0 && t >= t_end) {
      speed = 0.0;
      speed_rate = 0.0;
    } else {
      speed = v_max;
      speed_rate = 0.0;
    }
  }

  const double heading = params_.straight_direction_yaw;
  const double dir_x = std::cos(heading);
  const double dir_y = std::sin(heading);
  state_.velocity = Eigen::Vector3d(speed * dir_x, speed * dir_y, 0.0);
  state_.acceleration = Eigen::Vector3d(speed_rate * dir_x, speed_rate * dir_y, 0.0);
  state_.yaw = heading;
}

void TargetDroneBehaviour::updateWeave()
{
  const double speed = std::max(params_.weave_speed, 0.0);
  const double amplitude = std::max(params_.weave_amplitude, 0.0);
  const double frequency = std::max(params_.weave_frequency, 0.0);
  const double omega = kTwoPi * frequency;

  // The forward direction of the weave and the lateral one, 90 degrees to the left.
  const double heading = params_.weave_direction_yaw;
  const double forward_x = std::cos(heading);
  const double forward_y = std::sin(heading);
  const double lateral_x = -std::sin(heading);
  const double lateral_y = std::cos(heading);

  // The forward speed is constant and the lateral speed oscillates, so the lateral
  // offset is a sine of the amplitude and the frequency of the parameters.
  const double phase = omega * state_.time;
  const double v_lateral = amplitude * omega * std::cos(phase);
  const double a_lateral = -amplitude * omega * omega * std::sin(phase);

  state_.velocity = Eigen::Vector3d(
    speed * forward_x + v_lateral * lateral_x,
    speed * forward_y + v_lateral * lateral_y, 0.0);
  state_.acceleration = Eigen::Vector3d(a_lateral * lateral_x, a_lateral * lateral_y, 0.0);
  state_.yaw = std::atan2(state_.velocity.y(), state_.velocity.x());
}

void TargetDroneBehaviour::updateCurve()
{
  // The original profile: the drone starts nearly stationary and accelerates to
  // final_speed over accel_time, while it turns at a constant turn_rate the whole
  // time. The acceleration therefore has two parts: the tangential one from the
  // speed ramp and the lateral one from the turn. That combination is what pure
  // proportional navigation cannot follow and what augmented PN compensates.
  const double t = state_.time;
  const double ramp_time = std::max(params_.accel_time, 1e-6);
  const double heading0 = params_.start_yaw;
  const double turn_rate = params_.turn_rate;

  // The heading turns at a constant rate from the start.
  const double heading = heading0 + turn_rate * t;
  const double cos_h = std::cos(heading);
  const double sin_h = std::sin(heading);

  // Speed: the ramp, then constant.
  const bool ramping = t < ramp_time;
  const double speed = ramping ? params_.final_speed * t / ramp_time : params_.final_speed;
  const double speed_rate = ramping ? params_.final_speed / ramp_time : 0.0;

  state_.velocity = Eigen::Vector3d(speed * cos_h, speed * sin_h, params_.vertical_rate);

  // d(velocity)/dt: the speed ramp along the heading plus the turn rate across it.
  state_.acceleration = Eigen::Vector3d(
    speed_rate * cos_h - speed * turn_rate * sin_h,
    speed_rate * sin_h + speed * turn_rate * cos_h,
    0.0);
  state_.yaw = heading;
}

// ---------------------------------------------------------------------------
// The path of the waypoints behaviour
// ---------------------------------------------------------------------------

void TargetDroneBehaviour::buildPath()
{
  path_.clear();
  knot_s_.clear();
  knot_z_.clear();
  path_length_ = 0.0;

  const std::vector<Eigen::Vector3d> & wps = params_.waypoints;
  const size_t n = wps.size();
  if (n < 2) {
    return;
  }

  // The direction and the length of every leg of the loop.
  std::vector<Eigen::Vector3d> leg_dir(n);
  std::vector<double> leg_len(n);
  for (size_t i = 0; i < n; ++i) {
    const Eigen::Vector3d d = wps[(i + 1) % n] - wps[i];
    leg_len[i] = std::hypot(d.x(), d.y());
    if (leg_len[i] > kMinLength) {
      leg_dir[i] = Eigen::Vector3d(d.x() / leg_len[i], d.y() / leg_len[i], 0.0);
    } else {
      leg_dir[i] = Eigen::Vector3d::UnitX();
    }
  }

  // The turn at every waypoint, and the circular fillet that rounds it off. The
  // fillet is what keeps the velocity continuous: the drone turns at a constant
  // rate around the corner instead of changing its direction in one step.
  std::vector<double> turn(n, 0.0);
  std::vector<double> tangent(n, 0.0);
  std::vector<double> radius(n, 0.0);
  std::vector<Eigen::Vector3d> arc_exit(n);
  std::vector<Eigen::Vector3d> arc_centre(n);
  std::vector<double> arc_angle(n, 0.0);

  for (size_t i = 0; i < n; ++i) {
    const Eigen::Vector3d & u_in = leg_dir[(i + n - 1) % n];
    const Eigen::Vector3d & u_out = leg_dir[i];
    const double cross = u_in.x() * u_out.y() - u_in.y() * u_out.x();
    const double dot = u_in.dot(u_out);
    // A U turn has no fillet: the geometry degenerates at exactly pi.
    const double angle = std::clamp(std::atan2(cross, dot), -(kPi - 1e-3), kPi - 1e-3);
    turn[i] = angle;

    const double half = std::abs(angle) / 2.0;
    if (half < 1e-6) {
      continue;
    }
    // The tangent length of the fillet, limited so that two neighbouring fillets
    // never eat the whole leg between them.
    const double wanted = std::max(params_.waypoint_turn_radius, 0.0) * std::tan(half);
    const double limit = 0.45 * std::min(leg_len[(i + n - 1) % n], leg_len[i]);
    tangent[i] = std::min(wanted, limit);
    radius[i] = tangent[i] / std::tan(half);

    // The entry and the exit of the fillet, on the two legs it joins.
    const Eigen::Vector3d entry = wps[i] - u_in * tangent[i];
    arc_exit[i] = wps[i] + u_out * tangent[i];

    // The centre of the circle is on the inside of the turn, on the bisector of
    // the two legs.
    const Eigen::Vector3d bisector = u_in + u_out;
    const Eigen::Vector3d middle = (bisector.norm() > kMinLength) ?
      Eigen::Vector3d(bisector.normalized()) : Eigen::Vector3d::UnitX();
    const Eigen::Vector3d left(-middle.y(), middle.x(), 0.0);
    const double sign = (angle > 0.0) ? 1.0 : -1.0;
    const double offset = radius[i] / std::cos(half);
    arc_centre[i] = Eigen::Vector3d(wps[i].x(), wps[i].y(), 0.0) + sign * offset * left;
    arc_angle[i] = std::atan2(entry.y() - arc_centre[i].y(), entry.x() - arc_centre[i].x());
  }

  // The path itself: a closed loop of straights joined by the fillets. It starts
  // at the exit of the fillet of the first waypoint, which is where the lead-in
  // from the waypoint itself ends.
  double s = 0.0;
  for (size_t i = 0; i < n; ++i) {
    const size_t j = (i + 1) % n;

    // The knot of the altitude profile: the start of the leg.
    knot_s_.push_back(s);
    knot_z_.push_back(wps[i].z());

    // The straight of the leg, from the exit of the fillet of the waypoint it
    // leaves to the entry of the fillet of the waypoint it reaches.
    const double straight_length = leg_len[i] - tangent[i] - tangent[j];
    if (straight_length > kMinLength) {
      PathElement element;
      element.is_arc = false;
      element.length = straight_length;
      element.start_s = s;
      element.start = (i == 0) ?
        Eigen::Vector3d(arc_exit[0].x(), arc_exit[0].y(), 0.0) :
        Eigen::Vector3d(arc_exit[i].x(), arc_exit[i].y(), 0.0);
      element.direction = leg_dir[i];
      path_.push_back(element);
      s += straight_length;
    }

    // The fillet at the waypoint the leg reaches.
    if (tangent[j] > kMinLength) {
      PathElement element;
      element.is_arc = true;
      element.length = radius[j] * std::abs(turn[j]);
      element.start_s = s;
      element.centre = arc_centre[j];
      element.radius = radius[j];
      element.start_angle = arc_angle[j];
      element.turn = turn[j];
      path_.push_back(element);
      s += element.length;
    }
  }

  polygon_length_ = s;

  // The lead-in: from the first waypoint to the start of the loop. It is flown
  // once, and the drone starts on the waypoint itself.
  lead_in_ = tangent[0];
  lead_start_ = Eigen::Vector3d(wps[0].x(), wps[0].y(), 0.0);
  lead_direction_ = leg_dir[0];
  path_length_ = lead_in_ + polygon_length_;
}

void TargetDroneBehaviour::evaluatePath(
  double s, Eigen::Vector3d & position, double & heading, double & curvature) const
{
  if (s < lead_in_) {
    // The lead-in, from the first waypoint to the start of the loop.
    position = lead_start_ + lead_direction_ * s;
    heading = std::atan2(lead_direction_.y(), lead_direction_.x());
    curvature = 0.0;
    return;
  }

  double local_s = s - lead_in_;
  if (local_s < 0.0) {
    local_s += polygon_length_;
  }

  // The element the arclength falls into.
  size_t index = 0;
  while (index + 1 < path_.size() && local_s >= path_[index].length) {
    local_s -= path_[index].length;
    ++index;
  }
  const PathElement & element = path_[index];

  if (element.is_arc) {
    const double sign = (element.turn >= 0.0) ? 1.0 : -1.0;
    const double angle = element.start_angle + sign * local_s / element.radius;
    position = Eigen::Vector3d(
      element.centre.x() + element.radius * std::cos(angle),
      element.centre.y() + element.radius * std::sin(angle), 0.0);
    heading = angle + sign * kPi / 2.0;
    curvature = sign / element.radius;
  } else {
    position = element.start + element.direction * local_s;
    heading = std::atan2(element.direction.y(), element.direction.x());
    curvature = 0.0;
  }
}

double TargetDroneBehaviour::altitudeAt(double s) const
{
  if (knot_s_.size() < 2) {
    return knot_z_.empty() ? 0.0 : knot_z_.front();
  }
  const size_t n = knot_s_.size();
  size_t index = 0;
  for (size_t i = n; i-- > 0; ) {
    if (s >= knot_s_[i]) {
      index = i;
      break;
    }
  }
  const size_t next = (index + 1) % n;
  const double seg_len = segmentLength(index);
  if (seg_len < kMinLength) {
    return knot_z_[index];
  }
  const double u = (s - knot_s_[index]) / seg_len;
  // A smooth step: the slope is zero at both ends, so the vertical velocity is
  // continuous at the waypoints.
  return knot_z_[index] + (knot_z_[next] - knot_z_[index]) * (3.0 * u * u - 2.0 * u * u * u);
}

double TargetDroneBehaviour::altitudeRateAt(double s) const
{
  if (knot_s_.size() < 2) {
    return 0.0;
  }
  const size_t n = knot_s_.size();
  size_t index = 0;
  for (size_t i = n; i-- > 0; ) {
    if (s >= knot_s_[i]) {
      index = i;
      break;
    }
  }
  const size_t next = (index + 1) % n;
  const double seg_len = segmentLength(index);
  if (seg_len < kMinLength) {
    return 0.0;
  }
  const double u = (s - knot_s_[index]) / seg_len;
  return (knot_z_[next] - knot_z_[index]) * (6.0 * u - 6.0 * u * u) / seg_len;
}

double TargetDroneBehaviour::altitudeAccelerationAt(double s) const
{
  if (knot_s_.size() < 2) {
    return 0.0;
  }
  const size_t n = knot_s_.size();
  size_t index = 0;
  for (size_t i = n; i-- > 0; ) {
    if (s >= knot_s_[i]) {
      index = i;
      break;
    }
  }
  const size_t next = (index + 1) % n;
  const double seg_len = segmentLength(index);
  if (seg_len < kMinLength) {
    return 0.0;
  }
  const double u = (s - knot_s_[index]) / seg_len;
  return (knot_z_[next] - knot_z_[index]) * (6.0 - 12.0 * u) / (seg_len * seg_len);
}

double TargetDroneBehaviour::segmentLength(size_t index) const
{
  const size_t n = knot_s_.size();
  if (index + 1 < n) {
    return knot_s_[index + 1] - knot_s_[index];
  }
  // The last segment wraps around to the first knot.
  return polygon_length_ - knot_s_[index] + knot_s_.front();
}

}  // namespace guidance
}  // namespace interceptor
