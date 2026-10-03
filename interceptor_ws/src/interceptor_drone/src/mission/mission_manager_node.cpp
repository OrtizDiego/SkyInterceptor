/**
 * @file mission_manager_node.cpp
 * @brief Owns the mission mode: serves /mission/set_mode and latches it on /mission/mode
 */

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <interceptor_interfaces/msg/mission_mode.hpp>
#include <interceptor_interfaces/msg/target_detection.hpp>
#include <interceptor_interfaces/srv/set_mission_mode.hpp>

#include "common/mission.hpp"

namespace interceptor
{

using MissionModeMsg = interceptor_interfaces::msg::MissionMode;
using SetMissionMode = interceptor_interfaces::srv::SetMissionMode;
using TargetDetectionMsg = interceptor_interfaces::msg::TargetDetection;

static_assert(static_cast<int>(MissionMode::HOLD) == MissionModeMsg::MODE_HOLD);
static_assert(static_cast<int>(MissionMode::FOLLOW) == MissionModeMsg::MODE_FOLLOW);
static_assert(static_cast<int>(MissionMode::INTERCEPT) == MissionModeMsg::MODE_INTERCEPT);
static_assert(MissionModeMsg::MODE_INTERCEPT == SetMissionMode::Request::MODE_INTERCEPT);
static_assert(static_cast<int>(TargetClass::PERSON) == TargetDetectionMsg::PERSON);
static_assert(static_cast<int>(TargetClass::CAR) == TargetDetectionMsg::CAR);
static_assert(static_cast<int>(TargetClass::TRUCK) == TargetDetectionMsg::TRUCK);
static_assert(static_cast<int>(TargetClass::BICYCLE) == TargetDetectionMsg::BICYCLE);
static_assert(static_cast<int>(TargetClass::UAV) == TargetDetectionMsg::UAV);

class MissionManagerNode : public rclcpp::Node
{
public:
  explicit MissionManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("mission_manager_node", options)
  {
    const std::string initial_mode = declare_parameter("initial_mode", "hold");
    const auto mode = missionModeFromString(initial_mode);
    if (!mode) {
      throw std::invalid_argument(
              "initial_mode must be hold, follow or intercept, got '" + initial_mode + "'");
    }

    // Latched so nodes started later still receive the current mode
    pub_mode_ = create_publisher<MissionModeMsg>(
      "/mission/mode", rclcpp::QoS(1).reliable().transient_local());
    srv_set_mode_ = create_service<SetMissionMode>(
      "/mission/set_mode",
      std::bind(
        &MissionManagerNode::setModeCallback, this,
        std::placeholders::_1, std::placeholders::_2));

    // Always start disarmed: engaging needs an explicit operator request
    current_.mode = static_cast<uint8_t>(*mode);
    current_.armed = false;
    current_.track_id = -1;
    publishMode();
    RCLCPP_INFO(get_logger(), "Mission mode: %s (disarmed)", toString(*mode).c_str());
  }

private:
  void setModeCallback(
    const std::shared_ptr<SetMissionMode::Request> request,
    std::shared_ptr<SetMissionMode::Response> response)
  {
    const auto mode = missionModeFromValue(request->mode);
    if (!mode) {
      response->success = false;
      response->message = "unknown mode " + std::to_string(request->mode);
    } else if (request->armed && *mode != MissionMode::INTERCEPT) {
      response->success = false;
      response->message = "only INTERCEPT can be armed";
    } else if (request->track_id < -1) {
      response->success = false;
      response->message = "track_id must be -1 (auto) or a track id";
    } else {
      current_.mode = request->mode;
      current_.armed = request->armed;
      current_.track_id = request->track_id;
      publishMode();
      response->success = true;
      response->message = "mode " + toString(*mode) + (request->armed ? " (armed)" : "");
    }

    if (response->success) {
      RCLCPP_INFO(get_logger(), "Mission mode set: %s", response->message.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "SetMissionMode rejected: %s", response->message.c_str());
    }
  }

  void publishMode()
  {
    current_.header.stamp = now();
    pub_mode_->publish(current_);
  }

  MissionModeMsg current_;
  rclcpp::Publisher<MissionModeMsg>::SharedPtr pub_mode_;
  rclcpp::Service<SetMissionMode>::SharedPtr srv_set_mode_;
};

}  // namespace interceptor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<interceptor::MissionManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
