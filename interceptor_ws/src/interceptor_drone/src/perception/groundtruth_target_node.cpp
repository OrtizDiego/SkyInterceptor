/**
 * @file groundtruth_target_node.cpp
 * @brief Publishes TargetDetection messages from Gazebo ground truth
 *
 * Polls /get_entity_state (gazebo_ros_state world plugin) for every configured target entity,
 * applies dropout, Gaussian position noise and optional line-of-sight occlusion by vertical
 * cylinders (tree canopies), and publishes the result on /target/detection_3d in place of the
 * vision pipeline. Requests are asynchronous, so the timer never blocks on Gazebo.
 */

#include <Eigen/Dense>

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <gazebo_msgs/srv/get_entity_state.hpp>
#include <interceptor_interfaces/msg/target_detection.hpp>
#include <rclcpp/rclcpp.hpp>

#include "common/mission.hpp"
#include "perception/groundtruth_sensor.hpp"

namespace interceptor
{

using GetEntityState = gazebo_msgs::srv::GetEntityState;
using TargetDetectionMsg = interceptor_interfaces::msg::TargetDetection;

class GroundTruthTargetNode : public rclcpp::Node
{
public:
  explicit GroundTruthTargetNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("groundtruth_target_node", options),
    sensor_(readSensorConfig(), readSeed())
  {
    const auto entity_names = declare_parameter<std::vector<std::string>>(
      "entity_names", {"target_person"});
    const auto entity_classes = declare_parameter<std::vector<std::string>>(
      "entity_classes", {"person"});
    auto entity_z_offsets = declare_parameter<std::vector<double>>(
      "entity_z_offsets", std::vector<double>{});
    const double rate_hz = declare_parameter("rate_hz", 30.0);
    frame_id_ = declare_parameter("frame_id", "map");
    reference_frame_ = declare_parameter("reference_frame", "world");
    const std::string service_name = declare_parameter(
      "get_entity_state_service", "/get_entity_state");
    const std::string observer_entity = declare_parameter("observer_entity", "interceptor");
    occlusion_enabled_ = declare_parameter("occlusion.enabled", false);
    occluders_ = parseOcclusionCylinders(
      declare_parameter<std::vector<double>>("occlusion.cylinders", std::vector<double>{}));

    if (entity_names.empty()) {
      throw std::invalid_argument("entity_names must not be empty");
    }
    if (entity_names.size() != entity_classes.size()) {
      throw std::invalid_argument("entity_names and entity_classes must have the same length");
    }
    if (entity_z_offsets.empty()) {
      entity_z_offsets.assign(entity_names.size(), 0.0);
    } else if (entity_z_offsets.size() != entity_names.size()) {
      throw std::invalid_argument("entity_z_offsets must be empty or match entity_names");
    }
    if (!(rate_hz > 0.0)) {
      throw std::invalid_argument("rate_hz must be > 0");
    }
    const std::vector<TargetClass> classes = parseTargetClasses(entity_classes);
    for (std::size_t i = 0; i < entity_names.size(); ++i) {
      targets_.emplace_back(entity_names[i], classes[i], entity_z_offsets[i]);
    }
    if (!observer_entity.empty()) {
      // The class is unused: the observer is only polled for occlusion and range
      observer_.emplace(observer_entity, TargetClass::UAV, 0.0);
    } else if (occlusion_enabled_) {
      throw std::invalid_argument("occlusion.enabled needs an observer_entity");
    }

    client_ = create_client<GetEntityState>(service_name);
    pub_detection_ = create_publisher<TargetDetectionMsg>("/target/detection_3d", 10);
    // Runs on the node clock, so the rate is in sim time when use_sim_time is set
    timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::duration<double>(1.0 / rate_hz),
      std::bind(&GroundTruthTargetNode::poll, this));

    RCLCPP_INFO(
      get_logger(),
      "Ground-truth targets: %zu entities at %.1f Hz, noise %.2f m, dropout %.2f, "
      "occlusion %s (%zu cylinders)",
      targets_.size(), rate_hz, sensor_.config().pos_noise_std, sensor_.config().dropout_prob,
      occlusion_enabled_ ? "on" : "off", occluders_.size());
  }

private:
  // One Gazebo entity polled through /get_entity_state, at most one request in flight
  struct EntityPoll
  {
    EntityPoll(std::string entity_name, TargetClass entity_class, double entity_z_offset)
    : name(std::move(entity_name)), target_class(entity_class), z_offset(entity_z_offset) {}

    std::string name;
    TargetClass target_class;
    double z_offset;  // m, moves the entity origin to the point a detector would report
    std::optional<int64_t> pending_request;
    std::chrono::steady_clock::time_point sent_at{};
  };

  // Gazebo did not answer: drop the request so the entity is polled again
  static constexpr std::chrono::seconds REQUEST_TIMEOUT{1};

  GroundTruthSensorConfig readSensorConfig()
  {
    GroundTruthSensorConfig config;
    config.pos_noise_std = declare_parameter("pos_noise_std", config.pos_noise_std);
    config.dropout_prob = declare_parameter("dropout_prob", config.dropout_prob);
    return config;
  }

  uint64_t readSeed()
  {
    // 0 draws a fresh seed, anything else makes the noise and dropout reproducible
    const int64_t seed = declare_parameter<int64_t>("seed", 0);
    return seed != 0 ? static_cast<uint64_t>(seed) : std::random_device{}();
  }

  void poll()
  {
    if (!client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for %s (gazebo_ros_state world plugin)", client_->get_service_name());
      return;
    }
    if (observer_) {
      request(*observer_);
    }
    for (EntityPoll & target : targets_) {
      request(target);
    }
  }

  void request(EntityPoll & entity)
  {
    const auto now = std::chrono::steady_clock::now();
    if (entity.pending_request) {
      if (now - entity.sent_at < REQUEST_TIMEOUT) {
        return;
      }
      client_->remove_pending_request(*entity.pending_request);
      RCLCPP_WARN(get_logger(), "get_entity_state for '%s' timed out", entity.name.c_str());
    }

    auto req = std::make_shared<GetEntityState::Request>();
    req->name = entity.name;
    req->reference_frame = reference_frame_;
    // targets_ and observer_ never change size after construction, so the pointer stays valid
    EntityPoll * entity_ptr = &entity;
    const auto future = client_->async_send_request(
      req, [this, entity_ptr](rclcpp::Client<GetEntityState>::SharedFuture response) {
        entity_ptr->pending_request.reset();
        onState(*entity_ptr, *response.get());
      });
    entity.pending_request = future.request_id;
    entity.sent_at = now;
  }

  void onState(const EntityPoll & entity, const GetEntityState::Response & response)
  {
    if (!response.success) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Gazebo has no entity '%s'", entity.name.c_str());
      return;
    }
    const auto & p = response.state.pose.position;
    const Eigen::Vector3d position(p.x, p.y, p.z + entity.z_offset);

    if (observer_ && &entity == &*observer_) {
      observer_position_ = position;
      return;
    }

    if (occlusion_enabled_ && observer_position_ &&
      isLineOfSightBlocked(*observer_position_, position, occluders_))
    {
      return;
    }
    const std::optional<Eigen::Vector3d> measured = sensor_.measure(position);
    if (!measured) {
      return;
    }

    TargetDetectionMsg msg;
    // Stamp with the sim time at which Gazebo sampled the pose
    msg.header.stamp = response.header.stamp;
    if (msg.header.stamp.sec == 0 && msg.header.stamp.nanosec == 0) {
      msg.header.stamp = now();
    }
    msg.header.frame_id = frame_id_;
    msg.detection_id = next_detection_id_++;
    msg.track_id = -1;
    msg.class_id = static_cast<int32_t>(entity.target_class);
    msg.class_name = toString(entity.target_class);
    msg.confidence = 1.0;
    msg.position_world.x = measured->x();
    msg.position_world.y = measured->y();
    msg.position_world.z = measured->z();
    msg.world_position_valid = true;
    // No camera here: the bbox and position_camera stay zero, depth is the range to the drone
    if (observer_position_) {
      msg.depth_meters = (position - *observer_position_).norm();
    }
    msg.depth_variance = sensor_.config().pos_noise_std * sensor_.config().pos_noise_std;
    pub_detection_->publish(msg);
  }

  GroundTruthSensor sensor_;
  std::vector<EntityPoll> targets_;
  std::optional<EntityPoll> observer_;
  std::optional<Eigen::Vector3d> observer_position_;
  std::vector<OcclusionCylinder> occluders_;
  bool occlusion_enabled_ = false;
  std::string frame_id_;
  std::string reference_frame_;
  int32_t next_detection_id_ = 0;

  rclcpp::Client<GetEntityState>::SharedPtr client_;
  rclcpp::Publisher<TargetDetectionMsg>::SharedPtr pub_detection_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace interceptor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<interceptor::GroundTruthTargetNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
