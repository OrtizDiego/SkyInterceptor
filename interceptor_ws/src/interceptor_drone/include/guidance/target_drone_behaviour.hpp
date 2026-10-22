/**
 * @file target_drone_behaviour.hpp
 * @brief The flight behaviours of the target drone (the INTERCEPT intruder).
 *
 * Plain C++ and Eigen, no ROS, so the trajectories can be unit tested without a
 * simulator. The node around it (src/guidance/target_drone_behavior_node.cpp) only
 * does the plumbing: it reads the parameters, integrates the behaviour and pushes the
 * pose into Gazebo with /set_entity_state.
 *
 * The behaviours
 * --------------
 *   HOVER      holds a fixed position, with an optional slow station keeping drift
 *   WAYPOINTS  flies a closed loop through a list of waypoints, at a constant speed
 *   STRAIGHT   flies a straight line at a constant speed, then stops
 *   WEAVE      flies forward at a constant speed while weaving sideways
 *   CURVE      the original profile: accelerates to a cruise speed and turns at a
 *              constant rate the whole time
 *
 * Two properties every behaviour keeps
 * -------------------------------------
 * 1. The published position is the exact integral of the published velocity. The
 *    behaviour integrates its own velocity, so the finite difference of two
 *    published positions is exactly the published velocity: the tracker's estimate
 *    of the target's velocity and acceleration matches what the drone really does.
 * 2. The published velocity is continuous. No behaviour steps its velocity: the
 *    drone eases into a turn, into a new speed and into a new behaviour, so the IMM
 *    tracker can follow it. The acceleration may step (a fillet really does change
 *    the curvature abruptly), but the velocity never does.
 *
 * Frames and units: the world frame (ENU), metres and seconds, like /odom and
 * /target/state.
 */

#pragma once

#include <Eigen/Dense>

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace interceptor
{
namespace guidance
{

/// The behaviour the target drone flies.
enum class TargetBehaviour
{
  HOVER,       // holds a fixed position
  WAYPOINTS,   // a closed loop through a list of waypoints
  STRAIGHT,    // a straight line at a constant speed
  WEAVE,       // forward at a constant speed, weaving sideways
  CURVE,       // accelerates and turns at a constant rate
};

/// "hover" | "waypoints" | "straight" | "weave" | "curve" -> TargetBehaviour.
///
/// std::nullopt when the name is not one of them.
std::optional<TargetBehaviour> parseTargetBehaviour(const std::string & name);

/// The name of a behaviour, as it is written in the config file.
std::string toString(TargetBehaviour behaviour);

/// Everything the behaviours are tuned with. One set for all of them: only the
/// parameters of the selected behaviour are read.
struct TargetBehaviourParams
{
  TargetBehaviour behaviour = TargetBehaviour::CURVE;

  // The start pose. Every behaviour starts here, except the waypoints behaviour,
  // which starts at the first waypoint of its list.
  Eigen::Vector3d start_position = Eigen::Vector3d(10.0, 5.0, 12.0);  // m
  double start_yaw = 0.0;                                             // rad

  // --- HOVER ---------------------------------------------------------------
  // The radius of the slow station keeping circle. Zero means the drone holds its
  // position exactly.
  double hover_drift = 0.0;         // m
  double hover_drift_period = 20.0;  // s

  // --- WAYPOINTS -----------------------------------------------------------
  // The waypoints of the loop, in order. The drone starts at the first one and
  // flies the loop again and again.
  std::vector<Eigen::Vector3d> waypoints;
  double waypoint_speed = 6.0;        // m/s, the horizontal speed
  double waypoint_turn_radius = 3.0;  // m, the radius of the fillet at a corner

  // --- STRAIGHT ------------------------------------------------------------
  double straight_speed = 10.0;         // m/s
  double straight_direction_yaw = 0.0;   // rad, the direction of the run
  double straight_duration = 30.0;       // s, 0 = fly until the behaviour is switched
  double straight_distance = 0.0;        // m, 0 = unlimited; wins over the duration
  double straight_ramp_time = 1.0;       // s, the smooth speed ramp at both ends

  // --- WEAVE ---------------------------------------------------------------
  double weave_speed = 6.0;          // m/s, the forward speed
  double weave_amplitude = 2.0;      // m, the amplitude of the lateral offset
  double weave_frequency = 0.25;     // Hz
  double weave_direction_yaw = 0.0;  // rad, the heading the weave is centred on

  // --- CURVE ---------------------------------------------------------------
  double final_speed = 6.0;    // m/s, the cruise speed reached after accel_time
  double accel_time = 4.0;     // s, the time it takes to reach final_speed
  double turn_rate = 1.2;      // rad/s, the constant curve
  double vertical_rate = 0.0;  // m/s, the drone flies level
};

/// The state of the intruder at one instant. World frame (ENU).
struct TargetBehaviourState
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();      // m
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();      // m/s
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();  // m/s^2
  double yaw = 0.0;                                          // rad
  double time = 0.0;                                         // s, since the start of the run
};

/// The behaviour of the target drone: it turns a time step into a state.
///
/// The class is the whole behaviour: the node around it only reads the parameters
/// and publishes the state. The position is integrated from the velocity, so the
/// two are consistent by construction.
class TargetDroneBehaviour
{
public:
  explicit TargetDroneBehaviour(const TargetBehaviourParams & params);

  const TargetBehaviourParams & params() const {return params_;}
  TargetBehaviour behaviour() const {return params_.behaviour;}

  /// Restarts the behaviour: the drone is back at the start pose and the clock is
  /// back at zero.
  void reset();

  /// Advances the behaviour by dt [s] and returns the new state.
  ///
  /// The new position is the old one plus the new velocity times dt, so the
  /// published position is the exact integral of the published velocity.
  const TargetBehaviourState & step(double dt);

  const TargetBehaviourState & state() const {return state_;}

private:
  /// One element of the waypoints path: a straight or a circular fillet.
  struct PathElement
  {
    bool is_arc = false;
    double length = 0.0;   // m, the arclength of the element
    double start_s = 0.0;   // m, the arclength of the element on the path
    // A straight
    Eigen::Vector3d start = Eigen::Vector3d::Zero();
    Eigen::Vector3d direction = Eigen::Vector3d::UnitX();
    // An arc
    Eigen::Vector3d centre = Eigen::Vector3d::Zero();
    double radius = 0.0;      // m
    double start_angle = 0.0;  // rad, the angle of the entry point around the centre
    double turn = 0.0;         // rad, the signed sweep of the arc
  };

  void buildPath();
  void updateHover();
  void updateWaypoints();
  void updateStraight();
  void updateWeave();
  void updateCurve();

  /// The state of the path at the arclength s: the position, the heading and the
  /// signed curvature (1 / radius, positive for a left turn).
  void evaluatePath(
    double s, Eigen::Vector3d & position, double & heading, double & curvature) const;

  /// The altitude of the path at the arclength s, and its rate of change.
  double altitudeAt(double s) const;
  double altitudeRateAt(double s) const;
  double altitudeAccelerationAt(double s) const;

  /// The arclength of the leg that starts at the knot index [m]. The last leg
  /// wraps around to the first knot.
  double segmentLength(size_t index) const;

  TargetBehaviourParams params_;
  TargetBehaviourState state_;

  // The path of the waypoints behaviour.
  std::vector<PathElement> path_;
  std::vector<double> knot_s_;  // m, the arclength of the start of every leg
  std::vector<double> knot_z_;  // m, the altitude of the waypoint of every leg
  double polygon_length_ = 0.0;  // m, the arclength of the closed loop
  double lead_in_ = 0.0;         // m, the straight from the first waypoint to the loop
  Eigen::Vector3d lead_start_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d lead_direction_ = Eigen::Vector3d::UnitX();
  double path_length_ = 0.0;     // m, the lead-in plus the loop
};

}  // namespace guidance
}  // namespace interceptor
