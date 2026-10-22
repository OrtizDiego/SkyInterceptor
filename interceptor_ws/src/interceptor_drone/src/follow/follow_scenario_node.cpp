/**
 * @file follow_scenario_node.cpp
 * @brief Runs a FOLLOW scenario: it moves the target (and the second person) of the
 *        scenario and keeps the tracker pointed at the right one of them.
 *
 * Topics
 * ------
 *   out /follow/target_truth  PointStamped  the ground truth of the target
 *
 * Services
 * --------
 *   in  /mission/set_mode     SetMissionMode  the operator's track, when it changes
 *   out /set_entity_state                     the pose of the scenario's entities
 *
 * What it does
 * ------------
 * The target of a FOLLOW scenario is a kinematic model: the node computes its pose
 * from the scenario library (a pure function of the time) and pushes it into Gazebo
 * with /set_entity_state, the way target_drone_behavior_node moves the intruder of
 * the INTERCEPT scenarios. The ground-truth source reads the pose back through
 * /get_entity_state, so the whole perception and estimation chain stays in the loop:
 * the tracker only ever sees noisy, delayed measurements of the target.
 *
 * The second person of the "bystander" scenario is a person who is *not* the target.
 * The tracker selects the closest confirmed person, so with two people in the scene
 * it could pick the wrong one. The class whitelist of the FOLLOW mode cannot
 * separate two people, so the node does what an operator would do: it identifies the
 * track of the scenario's target (the one whose position is the ground truth of the
 * target) and asks the mission manager to follow exactly that track. The tracker
 * and the follow planner both enforce the operator's track, and neither of them
 * falls back to another one.
 *
 * The match is temporal: the id of the target's track is kept as long as the track
 * exists, and a new id is only adopted when the track of the target has gone and
 * another one shows up at the ground truth of the target. That way a second person
 * crossing the path cannot steal the selection.
 */

#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Geometry>

#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <gazebo_msgs/srv/set_entity_state.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <interceptor_interfaces/msg/mission_mode.hpp>
#include <interceptor_interfaces/msg/target_state_array.hpp>
#include <interceptor_interfaces/srv/set_mission_mode.hpp>

#include "common/mission.hpp"
#include "follow/follow_scenario.hpp"

namespace interceptor
{

using SetEntityState = gazebo_msgs::srv::SetEntityState;
using SetMissionMode = interceptor_interfaces::srv::SetMissionMode;
using MissionModeMsg = interceptor_interfaces::msg::MissionMode;
using TargetStateArrayMsg = interceptor_interfaces::msg::TargetStateArray;
using PointStampedMsg = geometry_msgs::msg::PointStamped;

class FollowScenarioNode : public rclcpp::Node
{
public:
  explicit FollowScenarioNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("follow_scenario_node", options)
  {
    const double rate_hz = declare_parameter("rate_hz", 50.0);
    if (!(rate_hz > 0.0)) {
      throw std::invalid_argument("rate_hz must be positive");
    }
    target_entity_ = declare_parameter("target_entity", std::string("follow_target"));
    bystander_entity_ = declare_parameter("bystander_entity", std::string("bystander"));
    world_frame_ = declare_parameter("world_frame", std::string("map"));
    match_radius_ = declare_parameter("track_match_radius", 3.0);

    follow::FollowScenarioParams params = readScenarioParams();
    scenario_ = std::make_unique<follow::FollowScenario>(params);

    client_ = create_client<SetEntityState>("/set_entity_state");
    client_mode_ = create_client<SetMissionMode>("/mission/set_mode");
    pub_truth_ = create_publisher<PointStampedMsg>("/follow/target_truth", 10);

    sub_tracks_ = create_subscription<TargetStateArrayMsg>(
      "/tracks", rclcpp::QoS(10),
      [this](TargetStateArrayMsg::ConstSharedPtr msg) {onTracks(*msg);});
    sub_mode_ = create_subscription<MissionModeMsg>(
      "/mission/mode", rclcpp::QoS(1).reliable().transient_local(),
      [this](MissionModeMsg::ConstSharedPtr msg) {onMode(*msg);});

    // On the node clock, so the scenario follows the simulation time.
    timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::duration<double>(1.0 / rate_hz),
      [this]() {onTimer();});

    const std::string bystander = params.bystander_enabled ?
      "'" + bystander_entity_ + "'" : "disabled";
    RCLCPP_INFO(
      get_logger(),
      "FOLLOW scenario '%s': target '%s' at %.2f m/s, bystander %s at (%.1f, %.1f) m, "
      "seed %" PRIu64, follow::toString(params.scenario).c_str(), target_entity_.c_str(),
      params.speed, bystander.c_str(),
      params.bystander_position.x(), params.bystander_position.y(),
      static_cast<uint64_t>(params.seed));
  }

private:
  /// Reads the parameters of the selected scenario.
  ///
  /// The parameters of the four scenarios live side by side in the config file, under
  /// "scenarios.<name>.". The node only declares the ones of the scenario it runs,
  /// so a parameter that is missing from the file is caught here and not silently
  /// ignored.
  follow::FollowScenarioParams readScenarioParams()
  {
    follow::FollowScenarioParams params;
    const std::string name = declare_parameter("scenario", std::string("walker"));
    const auto parsed = follow::parseFollowScenario(name);
    if (!parsed) {
      throw std::invalid_argument(
              "scenario must be walker, jogger, occlusion or bystander, got '" + name + "'");
    }
    params.scenario = *parsed;
    const std::string prefix = "scenarios." + name + ".";

    params.speed = declare_parameter(prefix + "speed", params.speed);
    params.start_phase = declare_parameter(prefix + "start_phase", params.start_phase);
    params.entity_z = declare_parameter(prefix + "entity_z", params.entity_z);
    params.seed = declare_parameter("seed", static_cast<int64_t>(0));
    params.bystander_enabled = declare_parameter(prefix + "bystander.enabled", false);
    params.bystander_position = Eigen::Vector2d(
      declare_parameter(prefix + "bystander.x", params.bystander_position.x()),
      declare_parameter(prefix + "bystander.y", params.bystander_position.y()));
    params.bystander_yaw = declare_parameter(prefix + "bystander.yaw", params.bystander_yaw);

    const double centre_x = declare_parameter(prefix + "loop.centre_x", 8.5);
    const double centre_y = declare_parameter(prefix + "loop.centre_y", 6.0);
    const double half_width = declare_parameter(prefix + "loop.half_width", 4.0);
    const double half_height = declare_parameter(prefix + "loop.half_height", 6.0);
    const double corner_radius = declare_parameter(prefix + "loop.corner_radius", 1.5);
    params.path = follow::ScenarioPath::loop(
      Eigen::Vector2d(centre_x, centre_y), half_width, half_height, corner_radius);
    if (!params.path.valid()) {
      throw std::invalid_argument("the loop of the scenario has no length");
    }
    return params;
  }

  void onTracks(const TargetStateArrayMsg & msg)
  {
    tracks_ = msg.tracks;
    // The track of the target, if the one we follow is still there.
    if (target_track_id_ >= 0) {
      for (const auto & track : tracks_) {
        if (track.track_id == target_track_id_) {
          return;
        }
      }
      // The track of the target is gone: look for a new one at the ground truth.
      target_track_id_ = -1;
    }
    const follow::FollowScenarioState state = scenario_->state(now().seconds());
    int best_id = -1;
    double best_distance = match_radius_;
    for (const auto & track : tracks_) {
      if (!track.confirmed) {
        continue;
      }
      const double distance = std::hypot(
        track.position.x - state.target_position.x(),
        track.position.y - state.target_position.y());
      if (distance < best_distance) {
        best_distance = distance;
        best_id = track.track_id;
      }
    }
    if (best_id >= 0) {
      target_track_id_ = best_id;
      RCLCPP_INFO(
        get_logger(), "The target of the scenario is track %d", best_id);
    }
  }

  void onMode(const MissionModeMsg & msg)
  {
    mode_ = msg.mode;
    operator_track_id_ = msg.track_id;
  }

  void onTimer()
  {
    const double t = now().seconds();
    const follow::FollowScenarioState state = scenario_->state(t);

    sendPose(target_entity_, state.target_position, state.target_yaw);
    // The second person is given its pose in every scenario: a model that is
    // never moved falls under gravity, and the ground-truth source would report
    // it thousands of metres below the ground.
    sendPose(bystander_entity_, state.bystander_position, state.bystander_yaw);

    PointStampedMsg truth;
    truth.header.stamp = now();
    truth.header.frame_id = world_frame_;
    truth.point.x = state.target_position.x();
    truth.point.y = state.target_position.y();
    truth.point.z = state.target_position.z();
    pub_truth_->publish(truth);

    updateOperatorTrack();
  }

  /// Asks the mission manager to follow the track of the target.
  ///
  /// The service is only called when the track changes: calling it on every cycle
  /// would fill the mission manager's log with the same line.
  void updateOperatorTrack()
  {
    if (target_track_id_ < 0 || target_track_id_ == operator_track_id_) {
      return;
    }
    if (mode_ != static_cast<uint8_t>(MissionMode::FOLLOW)) {
      return;
    }
    if (!client_mode_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Waiting for /mission/set_mode");
      return;
    }
    auto request = std::make_shared<SetMissionMode::Request>();
    request->mode = static_cast<uint8_t>(MissionMode::FOLLOW);
    // The motors are armed by the follow planner, not here: the mission manager
    // only arms an INTERCEPT mission.
    request->armed = false;
    request->track_id = target_track_id_;
    client_mode_->async_send_request(request);
    RCLCPP_INFO(
      get_logger(), "Following track %d (the target of the scenario)", target_track_id_);
  }

  void sendPose(const std::string & entity, const Eigen::Vector3d & position, double yaw)
  {
    if (!client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Waiting for /set_entity_state");
      return;
    }
    auto request = std::make_shared<SetEntityState::Request>();
    request->state.name = entity;
    request->state.reference_frame = "world";
    request->state.pose.position.x = position.x();
    request->state.pose.position.y = position.y();
    request->state.pose.position.z = position.z();
    request->state.pose.orientation.x = 0.0;
    request->state.pose.orientation.y = 0.0;
    request->state.pose.orientation.z = std::sin(yaw * 0.5);
    request->state.pose.orientation.w = std::cos(yaw * 0.5);
    // The entity is kinematic: it must not fall under gravity, so it carries no
    // velocity of its own.
    client_->async_send_request(request);
  }

  std::string target_entity_;
  std::string bystander_entity_;
  std::string world_frame_;
  double match_radius_ = 3.0;

  std::unique_ptr<follow::FollowScenario> scenario_;

  rclcpp::Client<SetEntityState>::SharedPtr client_;
  rclcpp::Client<SetMissionMode>::SharedPtr client_mode_;
  rclcpp::Publisher<PointStampedMsg>::SharedPtr pub_truth_;
  rclcpp::Subscription<TargetStateArrayMsg>::SharedPtr sub_tracks_;
  rclcpp::Subscription<MissionModeMsg>::SharedPtr sub_mode_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<interceptor_interfaces::msg::TargetState> tracks_;
  int target_track_id_ = -1;
  int operator_track_id_ = -1;
  uint8_t mode_ = 0;
};

}  // namespace interceptor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<interceptor::FollowScenarioNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
