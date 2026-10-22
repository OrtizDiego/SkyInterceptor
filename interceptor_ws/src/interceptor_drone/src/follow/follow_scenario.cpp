/**
 * @file follow_scenario.cpp
 * @brief The FOLLOW scenarios: the target's path and the second person. Plain C++ and
 *        Eigen, no ROS.
 */

#include "follow/follow_scenario.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>

namespace interceptor
{
namespace follow
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
}  // namespace

std::optional<FollowScenarioType> parseFollowScenario(const std::string & name)
{
  std::string lowered;
  lowered.reserve(name.size());
  for (const char c : name) {
    lowered.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
  }
  if (lowered == "walker") {
    return FollowScenarioType::WALKER;
  }
  if (lowered == "jogger") {
    return FollowScenarioType::JOGGER;
  }
  if (lowered == "occlusion") {
    return FollowScenarioType::OCCLUSION;
  }
  if (lowered == "bystander") {
    return FollowScenarioType::BYSTANDER;
  }
  return std::nullopt;
}

std::string toString(FollowScenarioType scenario)
{
  switch (scenario) {
    case FollowScenarioType::WALKER:
      return "walker";
    case FollowScenarioType::JOGGER:
      return "jogger";
    case FollowScenarioType::OCCLUSION:
      return "occlusion";
    case FollowScenarioType::BYSTANDER:
      return "bystander";
  }
  return "unknown";
}

ScenarioPath ScenarioPath::loop(
  const Eigen::Vector2d & centre, double half_width, double half_height,
  double corner_radius)
{
  ScenarioPath path;
  path.centre_ = centre;
  path.half_width_ = std::max(0.0, half_width);
  path.half_height_ = std::max(0.0, half_height);
  // The corner radius cannot be larger than half of the smaller side, or the fillets
  // would overlap and the loop would not be a loop.
  const double limit = 0.5 * std::min(path.half_width_, path.half_height_);
  path.radius_ = std::clamp(corner_radius, 0.0, limit);
  path.straight_x_ = 2.0 * (path.half_width_ - path.radius_);
  path.straight_y_ = 2.0 * (path.half_height_ - path.radius_);
  path.arc_ = 0.5 * kPi * path.radius_;
  path.perimeter_ =
    2.0 * path.straight_x_ + 2.0 * path.straight_y_ + 4.0 * path.arc_;
  return path;
}

void ScenarioPath::evaluate(
  double s, Eigen::Vector2d & position, Eigen::Vector2d & tangent) const
{
  if (perimeter_ <= 0.0) {
    position = centre_;
    tangent = Eigen::Vector2d(1.0, 0.0);
    return;
  }
  s = std::fmod(s, perimeter_);
  if (s < 0.0) {
    s += perimeter_;
  }
  const double x0 = centre_.x() - half_width_ + radius_;
  const double y0 = centre_.y() - half_height_ + radius_;
  const double x1 = centre_.x() + half_width_ - radius_;
  const double y1 = centre_.y() + half_height_ - radius_;
  const double bottom = centre_.y() - half_height_;
  const double top = centre_.y() + half_height_;
  const double left = centre_.x() - half_width_;
  const double right = centre_.x() + half_width_;

  // The bottom leg, from x0 to x1.
  if (s < straight_x_) {
    position = Eigen::Vector2d(x0 + s, bottom);
    tangent = Eigen::Vector2d(1.0, 0.0);
    return;
  }
  s -= straight_x_;
  // The bottom right fillet, from the bottom leg to the right leg.
  if (s < arc_) {
    const double angle = s / radius_;
    position = Eigen::Vector2d(x1 + std::sin(angle) * radius_, y0 - std::cos(angle) * radius_);
    tangent = Eigen::Vector2d(std::cos(angle), std::sin(angle));
    return;
  }
  s -= arc_;
  // The right leg, from y0 to y1.
  if (s < straight_y_) {
    position = Eigen::Vector2d(right, y0 + s);
    tangent = Eigen::Vector2d(0.0, 1.0);
    return;
  }
  s -= straight_y_;
  // The top right fillet.
  if (s < arc_) {
    const double angle = s / radius_;
    position = Eigen::Vector2d(x1 + std::cos(angle) * radius_, y1 + std::sin(angle) * radius_);
    tangent = Eigen::Vector2d(-std::sin(angle), std::cos(angle));
    return;
  }
  s -= arc_;
  // The top leg, from x1 to x0.
  if (s < straight_x_) {
    position = Eigen::Vector2d(x1 - s, top);
    tangent = Eigen::Vector2d(-1.0, 0.0);
    return;
  }
  s -= straight_x_;
  // The top left fillet.
  if (s < arc_) {
    const double angle = s / radius_;
    position = Eigen::Vector2d(x0 - std::sin(angle) * radius_, y1 + std::cos(angle) * radius_);
    tangent = Eigen::Vector2d(-std::cos(angle), -std::sin(angle));
    return;
  }
  s -= arc_;
  // The left leg, from y1 to y0.
  if (s < straight_y_) {
    position = Eigen::Vector2d(left, y1 - s);
    tangent = Eigen::Vector2d(0.0, -1.0);
    return;
  }
  s -= straight_y_;
  // The bottom left fillet, back to the bottom leg.
  const double angle = s / radius_;
  position = Eigen::Vector2d(x0 - std::cos(angle) * radius_, y0 - std::sin(angle) * radius_);
  tangent = Eigen::Vector2d(std::sin(angle), -std::cos(angle));
}

Eigen::Vector2d ScenarioPath::position(double s) const
{
  Eigen::Vector2d position;
  Eigen::Vector2d tangent;
  evaluate(s, position, tangent);
  return position;
}

Eigen::Vector2d ScenarioPath::tangent(double s) const
{
  Eigen::Vector2d position;
  Eigen::Vector2d tangent;
  evaluate(s, position, tangent);
  return tangent;
}

FollowScenario::FollowScenario(const FollowScenarioParams & params)
: params_(params)
{
}

FollowScenarioState FollowScenario::state(double t) const
{
  FollowScenarioState state;

  // The seed shifts the start of the target along its path, so two runs of the same
  // scenario with different seeds do not start at the same place.
  const double phase = params_.start_phase +
    static_cast<double>(params_.seed % 1000) * 0.37;
  const double s = phase + params_.speed * t;

  const Eigen::Vector2d position = params_.path.position(s);
  const Eigen::Vector2d tangent = params_.path.tangent(s);
  state.target_position = Eigen::Vector3d(position.x(), position.y(), params_.entity_z);
  state.target_velocity = Eigen::Vector3d(
    tangent.x() * params_.speed, tangent.y() * params_.speed, 0.0);
  state.target_yaw = std::atan2(tangent.y(), tangent.x());

  state.bystander_position = Eigen::Vector3d(
    params_.bystander_position.x(), params_.bystander_position.y(), params_.entity_z);
  state.bystander_yaw = params_.bystander_yaw;
  return state;
}

}  // namespace follow
}  // namespace interceptor
