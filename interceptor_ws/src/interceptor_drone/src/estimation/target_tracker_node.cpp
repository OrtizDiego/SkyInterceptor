/**
 * @file target_tracker_node.cpp
 * @brief Multi-target IMM-EKF tracker (docs/TRACKER.md): groups /target/detection_3d
 *        into frames, publishes /tracks, the selected /target/state and /tracks/markers
 */

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <array>
#include <cstdio>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <interceptor_interfaces/msg/mission_mode.hpp>
#include <interceptor_interfaces/msg/target_detection.hpp>
#include <interceptor_interfaces/msg/target_state.hpp>
#include <interceptor_interfaces/msg/target_state_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "common/mission.hpp"
#include "estimation/frame_assembler.hpp"
#include "estimation/target_selector.hpp"
#include "estimation/track_manager.hpp"

namespace interceptor
{

using MissionModeMsg = interceptor_interfaces::msg::MissionMode;
using TargetDetectionMsg = interceptor_interfaces::msg::TargetDetection;
using TargetStateMsg = interceptor_interfaces::msg::TargetState;
using TargetStateArrayMsg = interceptor_interfaces::msg::TargetStateArray;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

namespace
{

std::vector<std::string> modelNames(const estimation::ImmParams & params)
{
  std::vector<std::string> names;
  for (const auto type : params.models) {
    names.push_back(estimation::toString(type));
  }
  return names;
}

std::vector<double> rowMajor(const Eigen::MatrixXd & matrix)
{
  std::vector<double> values;
  for (Eigen::Index i = 0; i < matrix.rows(); ++i) {
    for (Eigen::Index j = 0; j < matrix.cols(); ++j) {
      values.push_back(matrix(i, j));
    }
  }
  return values;
}

void copyMatrix(const Eigen::Matrix3d & matrix, std::array<double, 9> & out)
{
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      out[static_cast<size_t>(3 * i + j)] = matrix(i, j);
    }
  }
}

std_msgs::msg::ColorRGBA color(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA c;
  c.r = r;
  c.g = g;
  c.b = b;
  c.a = a;
  return c;
}

std::string className(int class_id)
{
  const auto target_class = targetClassFromValue(class_id);
  return target_class ? toString(*target_class) : "unknown";
}

}  // namespace

class TargetTrackerNode : public rclcpp::Node
{
public:
  explicit TargetTrackerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("target_tracker_node", options),
    tracker_(declareTrackManagerParams()),
    assembler_(declareFrameAssemblerParams()),
    selector_(declareTargetSelectorParams())
  {
    world_frame_ = declare_parameter("world_frame", std::string("map"));
    const double sigma = declare_parameter("measurement_noise_pos", 0.3);
    measurement_variance_ = sigma * sigma;
    const double publish_rate = declare_parameter("publish_rate", 50.0);
    if (!(publish_rate > 0.0) || !(sigma > 0.0)) {
      throw std::invalid_argument("publish_rate and measurement_noise_pos must be positive");
    }

    // Class whitelists of the two modes (follow_params.yaml / intercept_params.yaml)
    follow_classes_ = declareClasses("follow.eligible_classes", {"person", "bicycle", "car"});
    intercept_classes_ = declareClasses("intercept.eligible_classes", {"uav"});

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pub_tracks_ = create_publisher<TargetStateArrayMsg>("/tracks", 10);
    pub_target_ = create_publisher<TargetStateMsg>("/target/state", 10);
    pub_markers_ = create_publisher<MarkerArray>("/tracks/markers", 10);

    // Perception publishes one message per object in a burst: keep the queue deep
    sub_detection_ = create_subscription<TargetDetectionMsg>(
      "/target/detection_3d", rclcpp::QoS(100),
      std::bind(&TargetTrackerNode::detectionCallback, this, std::placeholders::_1));
    sub_mode_ = create_subscription<MissionModeMsg>(
      "/mission/mode", rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&TargetTrackerNode::modeCallback, this, std::placeholders::_1));
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::QoS(10),
      [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {odom_ = msg;});

    // On the node clock, so it follows simulation time
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / publish_rate),
      std::bind(&TargetTrackerNode::timerCallback, this));

    RCLCPP_INFO(
      get_logger(), "Target tracker started: %.0f Hz, frame %s, waiting for /mission/mode",
      publish_rate, world_frame_.c_str());
  }

private:
  // --- Parameters --------------------------------------------------------------

  estimation::TrackManagerParams declareTrackManagerParams()
  {
    // Defaults come from the library (docs/TRACKER.md); ekf_params.yaml mirrors them
    const estimation::TrackManagerParams defaults;
    estimation::TrackManagerParams params;

    estimation::ProcessNoise noise;
    noise.acc = declare_parameter("process_noise_acc", defaults.ground_imm.noise.acc);
    noise.jerk = declare_parameter("process_noise_jerk", defaults.ground_imm.noise.jerk);
    noise.turn_rate =
      declare_parameter("process_noise_turn_rate", defaults.aerial_imm.noise.turn_rate);

    estimation::InitialUncertainty ground_prior = defaults.ground_imm.prior;
    estimation::InitialUncertainty aerial_prior = defaults.aerial_imm.prior;
    ground_prior.velocity =
      declare_parameter("imm.ground_init_velocity_std", ground_prior.velocity);
    aerial_prior.velocity =
      declare_parameter("imm.aerial_init_velocity_std", aerial_prior.velocity);
    ground_prior.acceleration = aerial_prior.acceleration =
      declare_parameter("imm.init_acceleration_std", ground_prior.acceleration);
    ground_prior.turn_rate = aerial_prior.turn_rate =
      declare_parameter("imm.init_turn_rate_std", aerial_prior.turn_rate);

    const double interval =
      declare_parameter("imm.transition_interval", defaults.ground_imm.transition_interval);
    params.ground_imm = estimation::makeImmParams(
      declare_parameter("imm.ground_models", modelNames(defaults.ground_imm)),
      declare_parameter("imm.ground_transition", rowMajor(defaults.ground_imm.transition)),
      noise, ground_prior);
    params.aerial_imm = estimation::makeImmParams(
      declare_parameter("imm.aerial_models", modelNames(defaults.aerial_imm)),
      declare_parameter("imm.aerial_transition", rowMajor(defaults.aerial_imm.transition)),
      noise, aerial_prior);
    params.ground_imm.transition_interval = interval;
    params.aerial_imm.transition_interval = interval;

    params.gate_probability = declare_parameter("gate_probability", defaults.gate_probability);
    params.spawn_gate_probability =
      declare_parameter("spawn_gate_probability", defaults.spawn_gate_probability);
    params.confirm_m = declare_parameter("confirm_m", defaults.confirm_m);
    params.confirm_n = declare_parameter("confirm_n", defaults.confirm_n);
    params.max_missed_frames = declare_parameter("max_missed_frames", defaults.max_missed_frames);
    params.max_position_variance =
      declare_parameter("max_position_variance", defaults.max_position_variance);
    params.coast_timeout = declare_parameter("coast_timeout", defaults.coast_timeout);
    params.min_heading_speed = declare_parameter("min_heading_speed", defaults.min_heading_speed);
    params.heading_speed_sigmas =
      declare_parameter("heading_speed_sigmas", defaults.heading_speed_sigmas);
    return params;
  }

  estimation::FrameAssemblerParams declareFrameAssemblerParams()
  {
    estimation::FrameAssemblerParams params;
    params.stamp_tolerance =
      declare_parameter("frame_grouping.stamp_tolerance", params.stamp_tolerance);
    params.frame_timeout = declare_parameter("frame_grouping.timeout", params.frame_timeout);
    return params;
  }

  estimation::TargetSelectorParams declareTargetSelectorParams()
  {
    estimation::TargetSelectorParams params;
    params.switch_margin =
      declare_parameter("target_selection.switch_margin", params.switch_margin);
    return params;
  }

  std::vector<int> declareClasses(
    const std::string & name, const std::vector<std::string> & default_value)
  {
    std::vector<int> ids;
    for (const auto target_class : parseTargetClasses(declare_parameter(name, default_value))) {
      ids.push_back(static_cast<int>(target_class));
    }
    return ids;
  }

  // --- Inputs --------------------------------------------------------------------

  void detectionCallback(const TargetDetectionMsg::ConstSharedPtr msg)
  {
    if (!msg->world_position_valid) {
      return;
    }
    if (!targetClassFromValue(msg->class_id)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Dropping detection of unknown class %d",
        msg->class_id);
      return;
    }
    estimation::Measurement detection;
    detection.position = Eigen::Vector3d(
      msg->position_world.x, msg->position_world.y, msg->position_world.z);
    if (!detection.position.allFinite()) {
      return;
    }
    detection.class_id = msg->class_id;
    // Isotropic: the depth variance (when perception sets it) dominates the stereo error
    detection.covariance =
      Eigen::Matrix3d::Identity() * std::max(measurement_variance_, msg->depth_variance);

    const double arrival = now().seconds();
    double stamp = rclcpp::Time(msg->header.stamp).seconds();
    if (stamp <= 0.0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Detection without a stamp: using its arrival time");
      stamp = arrival;
    }
    if (!assembler_.add(stamp, detection, arrival)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Dropped a late detection (stamp %.3f s, last frame %.3f s), %zu so far", stamp,
        assembler_.lastStamp().value_or(0.0), assembler_.droppedLate());
    }
    processFrames(arrival);
  }

  void modeCallback(const MissionModeMsg::ConstSharedPtr msg)
  {
    const auto mode = missionModeFromValue(msg->mode);
    if (!mode) {
      RCLCPP_WARN(get_logger(), "Ignoring unknown mission mode %u", msg->mode);
      return;
    }
    mode_ = *mode;
    operator_track_id_ = msg->track_id;
    const std::string target =
      msg->track_id >= 0 ? "track " + std::to_string(msg->track_id) : "auto";
    RCLCPP_INFO(
      get_logger(), "Mission mode %s, target %s", toString(*mode).c_str(), target.c_str());
  }

  // Drone position in the world frame, the reference for the closest-track selection
  std::optional<Eigen::Vector3d> dronePosition()
  {
    if (!odom_) {
      return std::nullopt;
    }
    geometry_msgs::msg::PointStamped point;
    point.header.frame_id = odom_->header.frame_id;  // stamp 0: latest transform
    point.point = odom_->pose.pose.position;
    if (!point.header.frame_id.empty() && point.header.frame_id != world_frame_) {
      try {
        point = tf_buffer_->transform(point, world_frame_);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "No drone position in %s: %s",
          world_frame_.c_str(), ex.what());
        return std::nullopt;
      }
    }
    return Eigen::Vector3d(point.point.x, point.point.y, point.point.z);
  }

  std::vector<int> eligibleClasses() const
  {
    switch (mode_.value_or(MissionMode::HOLD)) {
      case MissionMode::FOLLOW:
        return follow_classes_;
      case MissionMode::INTERCEPT:
        return intercept_classes_;
      case MissionMode::HOLD:
        break;
    }
    return {};
  }

  // --- Tracking ------------------------------------------------------------------

  void processFrames(double now)
  {
    for (const auto & frame : assembler_.pop(now)) {
      if (!tracker_.processFrame(frame.stamp, frame.detections)) {
        RCLCPP_WARN(get_logger(), "Tracker dropped an out-of-order frame (%.3f s)", frame.stamp);
      }
    }
  }

  void timerCallback()
  {
    const rclcpp::Time stamp = now();
    const double t = stamp.seconds();
    if (last_tick_ && t < *last_tick_) {
      // Simulation reset: every stamp from before the jump is meaningless now
      RCLCPP_WARN(
        get_logger(), "Clock jumped back %.2f s: tracker reset", *last_tick_ - t);
      tracker_.clear();
      assembler_.clear();
      selector_.reset();
    }
    last_tick_ = t;

    processFrames(t);
    tracker_.prune(t);
    const std::vector<estimation::TrackEstimate> tracks = tracker_.tracks(t);

    estimation::SelectionRequest request;
    request.eligible_classes = eligibleClasses();
    request.operator_track_id = operator_track_id_;
    request.reference = dronePosition();
    const std::optional<int> previous = selector_.selected();
    const std::optional<int> selected = selector_.select(tracks, request);
    if (selected != previous) {
      logSelection(tracks, selected);
    }

    std_msgs::msg::Header header;
    header.stamp = stamp;
    header.frame_id = world_frame_;

    TargetStateArrayMsg array;
    array.header = header;
    array.tracks.reserve(tracks.size());
    TargetStateMsg target;  // No target: track_id -1, is_valid false
    target.header = header;
    target.track_id = -1;
    target.class_id = -1;
    for (const auto & track : tracks) {
      array.tracks.push_back(toMsg(track, header));
      if (selected && track.id == *selected) {
        target = array.tracks.back();
      }
    }
    pub_tracks_->publish(array);
    pub_target_->publish(target);
    publishMarkers(tracks, selected, header);
  }

  void logSelection(
    const std::vector<estimation::TrackEstimate> & tracks, std::optional<int> selected)
  {
    if (!selected) {
      RCLCPP_INFO(get_logger(), "Target: none");
      return;
    }
    for (const auto & track : tracks) {
      if (track.id == *selected) {
        RCLCPP_INFO(
          get_logger(), "Target: track %d (%s)", track.id, className(track.class_id).c_str());
      }
    }
  }

  // --- Outputs -------------------------------------------------------------------

  TargetStateMsg toMsg(
    const estimation::TrackEstimate & track, const std_msgs::msg::Header & header) const
  {
    TargetStateMsg msg;
    msg.header = header;
    msg.track_id = track.id;
    msg.class_id = track.class_id;
    msg.class_name = className(track.class_id);
    msg.position.x = track.position.x();
    msg.position.y = track.position.y();
    msg.position.z = track.position.z();
    copyMatrix(track.position_cov, msg.position_covariance);
    msg.velocity.x = track.velocity.x();
    msg.velocity.y = track.velocity.y();
    msg.velocity.z = track.velocity.z();
    copyMatrix(track.velocity_cov, msg.velocity_covariance);
    msg.acceleration.x = track.acceleration.x();
    msg.acceleration.y = track.acceleration.y();
    msg.acceleration.z = track.acceleration.z();
    msg.heading = track.heading;
    msg.quality_score = track.quality;
    msg.confirmed = track.confirmed;
    msg.is_valid = track.is_valid;
    msg.missed_detections = track.missed_frames;
    return msg;
  }

  // Per track: sphere, velocity arrow and a label (id, class, speed). Selected
  // track orange, confirmed green, tentative or lost grey.
  void publishMarkers(
    const std::vector<estimation::TrackEstimate> & tracks, std::optional<int> selected,
    const std_msgs::msg::Header & header)
  {
    MarkerArray array;
    std::set<int> ids;
    for (const auto & track : tracks) {
      ids.insert(track.id);
      const bool is_selected = selected && track.id == *selected;
      const std_msgs::msg::ColorRGBA track_color =
        is_selected ? color(1.0f, 0.5f, 0.0f, 1.0f) :
        (track.confirmed && track.is_valid) ? color(0.1f, 0.8f, 0.2f, 0.9f) :
        color(0.6f, 0.6f, 0.6f, 0.5f);

      Marker sphere;
      sphere.header = header;
      sphere.ns = kSphereNs;
      sphere.id = track.id;
      sphere.type = Marker::SPHERE;
      sphere.action = Marker::ADD;
      sphere.pose.position.x = track.position.x();
      sphere.pose.position.y = track.position.y();
      sphere.pose.position.z = track.position.z();
      sphere.pose.orientation.w = 1.0;
      sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.6;
      sphere.color = track_color;
      array.markers.push_back(sphere);

      const double speed = track.velocity.norm();
      Marker arrow;
      arrow.header = header;
      arrow.ns = kVelocityNs;
      arrow.id = track.id;
      if (speed > 0.05) {
        // Points the way the target moves, one metre per m/s
        arrow.type = Marker::ARROW;
        arrow.action = Marker::ADD;
        arrow.points.resize(2);
        arrow.points[0] = sphere.pose.position;
        arrow.points[1].x = track.position.x() + track.velocity.x();
        arrow.points[1].y = track.position.y() + track.velocity.y();
        arrow.points[1].z = track.position.z() + track.velocity.z();
        arrow.pose.orientation.w = 1.0;
        arrow.scale.x = 0.08;  // shaft diameter
        arrow.scale.y = 0.2;   // head diameter
        arrow.scale.z = 0.25;  // head length
        arrow.color = track_color;
      } else {
        arrow.action = Marker::DELETE;
      }
      array.markers.push_back(arrow);

      char text[96];
      std::snprintf(
        text, sizeof(text), "#%d %s\n%.1f m/s", track.id, className(track.class_id).c_str(),
        speed);
      Marker label;
      label.header = header;
      label.ns = kLabelNs;
      label.id = track.id;
      label.type = Marker::TEXT_VIEW_FACING;
      label.action = Marker::ADD;
      label.pose.position = sphere.pose.position;
      label.pose.position.z += 1.0;
      label.pose.orientation.w = 1.0;
      label.scale.z = 0.4;
      label.color = color(1.0f, 1.0f, 1.0f, track_color.a);
      label.text = text;
      array.markers.push_back(label);
    }

    // Remove the markers of deleted tracks
    for (const int id : published_marker_ids_) {
      if (ids.count(id) == 0) {
        for (const char * ns : {kSphereNs, kVelocityNs, kLabelNs}) {
          Marker marker;
          marker.header = header;
          marker.ns = ns;
          marker.id = id;
          marker.action = Marker::DELETE;
          array.markers.push_back(marker);
        }
      }
    }
    published_marker_ids_ = std::move(ids);
    if (!array.markers.empty()) {
      pub_markers_->publish(array);
    }
  }

  static constexpr const char * kSphereNs = "tracks";
  static constexpr const char * kVelocityNs = "track_velocity";
  static constexpr const char * kLabelNs = "track_labels";

  estimation::TrackManager tracker_;
  estimation::FrameAssembler assembler_;
  estimation::TargetSelector selector_;

  std::string world_frame_;
  double measurement_variance_ = 0.09;
  std::vector<int> follow_classes_;
  std::vector<int> intercept_classes_;

  // Mission mode from /mission/mode; nothing is selected until it arrives
  std::optional<MissionMode> mode_;
  int operator_track_id_ = -1;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  std::optional<double> last_tick_;
  std::set<int> published_marker_ids_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<TargetDetectionMsg>::SharedPtr sub_detection_;
  rclcpp::Subscription<MissionModeMsg>::SharedPtr sub_mode_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<TargetStateArrayMsg>::SharedPtr pub_tracks_;
  rclcpp::Publisher<TargetStateMsg>::SharedPtr pub_target_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_markers_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace interceptor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<interceptor::TargetTrackerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
