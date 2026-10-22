/**
 * @file follow_scenario.hpp
 * @brief The FOLLOW scenarios of plan 5.2.4: the walker, the jogger with turns, the
 *        occlusion behind a tree and the bystander. Plain C++ and Eigen, no ROS, so
 *        the paths can be unit tested on their own.
 *
 * Why the paths are computed and not scripted
 * -------------------------------------------
 * The world has a walking actor on a Catmull-Rom spline, but a spline is baked into
 * the world file: changing the speed or the corners means regenerating hundreds of
 * waypoints, and the scenario cannot be *seeded*, so two runs of the same scenario
 * are not the same run. The exit criteria of the phase are stated over 100
 * randomized runs, and a run has to be reproducible from its seed.
 *
 * So the target of the FOLLOW scenarios is a kinematic model that this library
 * drives: the position at a given time is a pure function of the time and the
 * parameters, and the only randomness of a run is the measurement noise of the
 * ground-truth source and the seed of the start offset. The path is a rounded
 * rectangle, walked at a constant speed: the straight legs and the circular fillets
 * give a target that walks and a target that turns, without a discontinuity in the
 * velocity (a sharp reversal would confuse the tracker).
 *
 * The scenarios
 * -------------
 *   walker     a person walking a 1.4 m/s loop: the plain case.
 *   jogger     the same loop with tighter corners at 4.0 m/s, so the target turns
 *              sharply and the framing point has to follow the turn.
 *   occlusion  a loop that walks the target straight through the canopy of an oak
 *              tree, so the line of sight from the drone to the target is blocked
 *              for several seconds and the track has to be reacquired.
 *   bystander  the walker with a second person standing next to the path, so the
 *              keep-out of a person who is *not* the target is exercised.
 *
 * The geometry of the loops is not free: the drone flies 8 m behind the target and
 * 5 m above it, and the safety filter keeps it 7.3 m away from the centre of an
 * oak (the canopy's radius, the 3 m standoff and the drone's own radius). The loops
 * below are placed so that the whole loop stays outside that radius and outside the
 * 4.9 m of the pines, and so that the point 8 m behind the start of the target is
 * clear of the trees: the drone spawns there and climbs.
 */

#pragma once

#include <Eigen/Dense>

#include <optional>
#include <string>

namespace interceptor
{
namespace follow
{

/// The scenarios of the FOLLOW mode.
enum class FollowScenarioType
{
  WALKER,     // a person walking at 1.4 m/s
  JOGGER,     // the same at 4.0 m/s, with tight turns
  OCCLUSION,  // a loop through the canopy of a tree
  BYSTANDER,  // the walker, with a second person next to the path
};

/// "walker" | "jogger" | "occlusion" | "bystander" (case-insensitive).
std::optional<FollowScenarioType> parseFollowScenario(const std::string & name);
std::string toString(FollowScenarioType scenario);

/// A closed path: a rounded rectangle, walked counter-clockwise.
///
/// The path starts at the beginning of the bottom leg (the leftmost point of the
/// bottom edge, where the bottom edge starts after the corner fillet) and goes
/// counter-clockwise: +x along the bottom, +y along the right, -x along the top.
class ScenarioPath
{
public:
  ScenarioPath() = default;

  /// A rounded rectangle. The corner radius is clamped to half of the smaller side.
  static ScenarioPath loop(
    const Eigen::Vector2d & centre, double half_width, double half_height,
    double corner_radius);

  /// The perimeter [m].
  double length() const {return perimeter_;}
  bool valid() const {return perimeter_ > 0.0;}

  /// The position at the distance s along the path [m]. The path is closed, so s is
  /// taken modulo the perimeter.
  Eigen::Vector2d position(double s) const;

  /// The unit tangent (the direction of travel) at the distance s along the path.
  Eigen::Vector2d tangent(double s) const;

private:
  /// The position and the tangent at the distance s along the path. The path is
  /// made of four legs and four quarter circles, and every one of them has an
  /// exact tangent.
  void evaluate(double s, Eigen::Vector2d & position, Eigen::Vector2d & tangent) const;

  Eigen::Vector2d centre_ = Eigen::Vector2d::Zero();
  double half_width_ = 0.0;
  double half_height_ = 0.0;
  double radius_ = 0.0;
  double straight_x_ = 0.0;  // the length of a leg parallel to x
  double straight_y_ = 0.0;  // the length of a leg parallel to y
  double arc_ = 0.0;         // the length of a quarter circle
  double perimeter_ = 0.0;
};

/// The parameters of a scenario. They come from config/follow_scenario_params.yaml.
struct FollowScenarioParams
{
  FollowScenarioType scenario = FollowScenarioType::WALKER;

  /// The target's path.
  ScenarioPath path;
  double speed = 1.4;       // m/s, along the path
  double start_phase = 0.0;  // m, the distance along the path the target starts at

  /// The second person. It stands still: a person waiting next to the path. It is
  /// at the far end of the park in the scenarios that are not about it.
  bool bystander_enabled = false;
  Eigen::Vector2d bystander_position = Eigen::Vector2d(-1.0, 15.0);
  double bystander_yaw = 0.0;

  /// The height of the entities' origin [m]. The model stands on the ground and
  /// the ground-truth source adds its own offset to report the body centre.
  double entity_z = 0.0;

  /// The seed of the run. It shifts the start of the target along its path, so two
  /// runs with the same scenario but different seeds start at different places.
  uint64_t seed = 0;
};

/// The state of the scenario at one instant.
struct FollowScenarioState
{
  Eigen::Vector3d target_position = Eigen::Vector3d::Zero();   // m
  Eigen::Vector3d target_velocity = Eigen::Vector3d::Zero();   // m/s
  double target_yaw = 0.0;                                     // rad
  Eigen::Vector3d bystander_position = Eigen::Vector3d::Zero();  // m
  double bystander_yaw = 0.0;                                  // rad
};

/// The FOLLOW scenarios: the target's path and the second person.
class FollowScenario
{
public:
  explicit FollowScenario(const FollowScenarioParams & params = FollowScenarioParams());

  const FollowScenarioParams & params() const {return params_;}

  /// The state of the scenario at the time t [s] on the node clock. The position is
  /// a pure function of t, so a run is reproducible.
  FollowScenarioState state(double t) const;

private:
  FollowScenarioParams params_;
};

}  // namespace follow
}  // namespace interceptor
