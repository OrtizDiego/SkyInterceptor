/**
 * @file safety_filter_node.cpp
 * @brief The safety filter node: /setpoint/raw -> /setpoint/safe (plan section 4.3).
 *
 * The node is the last word between the planners and the trajectory controller. It
 * reads the planner's setpoint, the tracks, the odometry and the mission mode, and
 * publishes a setpoint the drone can actually fly without coming closer to a person,
 * a vehicle or a static obstacle than the configured standoff.
 *
 * All the maths lives in the ROS-free library include/safety/safety_filter.hpp; this
 * file only does the plumbing.
 *
 * Topics
 * ------
 *   in   /setpoint/raw      FlightSetpoint  the planner's command
 *   in   /tracks            TargetStateArray every track, so every person is kept out of
 *   in   /odom              nav_msgs/Odometry the drone's own state
 *   in   /mission/mode      MissionMode      latched by the mission manager
 *   in   /mission/estop     std_msgs/Bool    the operator's emergency stop
 *   out  /setpoint/safe     FlightSetpoint  the command the controller flies
 *   out  /mission/status    MissionStatus    what the filter did and why
 *   out  /safety/keepouts   MarkerArray      the keep-out circles, for RViz
 */

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <interceptor_interfaces/msg/flight_setpoint.hpp>
#include <interceptor_interfaces/msg/mission_mode.hpp>
#include <interceptor_interfaces/msg/mission_status.hpp>
#include <interceptor_interfaces/msg/target_state.hpp>
#include <interceptor_interfaces/msg/target_state_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "common/mission.hpp"
#include "safety/safety_filter.hpp"

namespace interceptor
{

using FlightSetpointMsg = interceptor_interfaces::msg::FlightSetpoint;
using MissionModeMsg = interceptor_interfaces::msg::MissionMode;
using MissionStatusMsg = interceptor_interfaces::msg::MissionStatus;
using TargetStateMsg = interceptor_interfaces::msg::TargetState;
using TargetStateArrayMsg = interceptor_interfaces::msg::TargetStateArray;
using BoolMsg = std_msgs::msg::Bool;
using MarkerArrayMsg = visualization_msgs::msg::MarkerArray;

class SafetyFilterNode : public rclcpp::Node
{
public:
  explicit SafetyFilterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("safety_filter_node", options)
  {
    const safety::SafetyFilterParams defaults;

    safety::SafetyFilterParams params;
    params.d_min_horizontal =
      declare_parameter("safety.d_min_horizontal", defaults.d_min_horizontal);
    params.d_obstacle_min = declare_parameter("safety.d_obstacle_min", defaults.d_obstacle_min);
    params.h_min_above = declare_parameter("safety.h_min_above", defaults.h_min_above);
    params.uncertainty_gain =
      declare_parameter("safety.uncertainty_gain", defaults.uncertainty_gain);
    params.drone_radius = declare_parameter("safety.drone_radius", defaults.drone_radius);
    params.cbf_alpha = declare_parameter("safety.cbf_alpha", defaults.cbf_alpha);
    params.max_projection_iterations = declare_parameter(
      "safety.max_projection_iterations", defaults.max_projection_iterations);
    params.velocity_lag_tau =
      declare_parameter("safety.velocity_lag_tau", defaults.velocity_lag_tau);
    params.barrier_omega = declare_parameter("safety.barrier_omega", defaults.barrier_omega);
    params.altitude_floor = declare_parameter("safety.altitude_floor", defaults.altitude_floor);
    params.altitude_ceiling =
      declare_parameter("safety.altitude_ceiling", defaults.altitude_ceiling);
    params.geofence_x_min = declare_parameter("safety.geofence.x_min", defaults.geofence_x_min);
    params.geofence_x_max = declare_parameter("safety.geofence.x_max", defaults.geofence_x_max);
    params.geofence_y_min = declare_parameter("safety.geofence.y_min", defaults.geofence_y_min);
    params.geofence_y_max = declare_parameter("safety.geofence.y_max", defaults.geofence_y_max);
    params.geofence_margin = declare_parameter("safety.geofence_margin", defaults.geofence_margin);
    params.speed_cap_follow =
      declare_parameter("safety.speed_cap.follow", defaults.speed_cap_follow);
    params.speed_cap_intercept =
      declare_parameter("safety.speed_cap.intercept", defaults.speed_cap_intercept);
    params.max_vertical_speed =
      declare_parameter("safety.max_vertical_speed", defaults.max_vertical_speed);
    params.setpoint_timeout =
      declare_parameter("safety.setpoint_timeout", defaults.setpoint_timeout);
    params.odom_timeout = declare_parameter("safety.odom_timeout", defaults.odom_timeout);
    params.coast_timeout = declare_parameter("safety.coast_timeout", defaults.coast_timeout);
    params.kp_pos = declare_parameter("safety.kp_pos", defaults.kp_pos);
    // The obstacle list: a flat array of x, y, radius, height per obstacle. It is
    // declared here so the parameter always exists, whether the file provides it or
    // not.
    declare_parameter("safety.obstacles", std::vector<double>{});

    filter_ = std::make_unique<safety::SafetyFilter>(params);
    obstacles_ = readObstacles();

    const double rate_hz = declare_parameter("rate_hz", 50.0);
    if (!(rate_hz > 0.0)) {
      throw std::invalid_argument("rate_hz must be positive");
    }

    pub_setpoint_ = create_publisher<FlightSetpointMsg>("/setpoint/safe", 10);
    pub_status_ = create_publisher<MissionStatusMsg>("/mission/status", 10);
    pub_markers_ = create_publisher<MarkerArrayMsg>("/safety/keepouts", 10);

    setpoint_sub_ = create_subscription<FlightSetpointMsg>(
      "/setpoint/raw", 10,
      [this](FlightSetpointMsg::ConstSharedPtr msg) {onSetpoint(*msg);});
    tracks_sub_ = create_subscription<TargetStateArrayMsg>(
      "/tracks", 10,
      [this](TargetStateArrayMsg::ConstSharedPtr msg) {onTracks(*msg);});
    target_sub_ = create_subscription<TargetStateMsg>(
      "/target/state", 10,
      [this](TargetStateMsg::ConstSharedPtr msg) {onTarget(*msg);});
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {onOdom(*msg);});
    mode_sub_ = create_subscription<MissionModeMsg>(
      "/mission/mode", rclcpp::QoS(1).reliable().transient_local(),
      [this](MissionModeMsg::ConstSharedPtr msg) {onMode(*msg);});
    estop_sub_ = create_subscription<BoolMsg>(
      "/mission/estop", rclcpp::QoS(1).reliable().transient_local(),
      [this](BoolMsg::ConstSharedPtr msg) {onEstop(*msg);});

    // On the node clock, so it follows simulation time
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate_hz),
      [this]() {onTimer();});

    RCLCPP_INFO(
      get_logger(),
      "Safety filter started: d_min=%.1f m, d_obstacle_min=%.1f m, %zu obstacles",
      params.d_min_horizontal, params.d_obstacle_min, obstacles_.size());
  }

private:
  // --- The static obstacles, from the parameter list -----------------------
  std::vector<safety::Obstacle> readObstacles() const
  {
    std::vector<safety::Obstacle> obstacles;
    // The list is flat: four numbers per obstacle (x, y, radius, height), so it
    // survives the parameter server, which cannot hold a list of dictionaries.
    const auto values = get_parameter("safety.obstacles").as_double_array();
    for (std::size_t i = 0; i + 3 < values.size(); i += 4) {
      safety::Obstacle obstacle;
      obstacle.x = values[i];
      obstacle.y = values[i + 1];
      obstacle.radius = values[i + 2];
      obstacle.height = values[i + 3];
      obstacle.name = "obstacle_" + std::to_string(obstacles.size());
      obstacles.push_back(obstacle);
    }
    return obstacles;
  }

  void onSetpoint(const FlightSetpointMsg & msg)
  {
    setpoint_.position =
      Eigen::Vector3d(msg.position.x, msg.position.y, msg.position.z);
    setpoint_.velocity = Eigen::Vector3d(msg.velocity.x, msg.velocity.y, msg.velocity.z);
    setpoint_.yaw = msg.yaw;
    setpoint_.yaw_rate = msg.yaw_rate;
    setpoint_.position_valid = msg.position_valid;
    setpoint_.source = msg.source;
    setpoint_time_ = now().seconds();
    has_setpoint_ = true;
  }

  void onTracks(const TargetStateArrayMsg & msg)
  {
    tracks_.clear();
    tracks_.reserve(msg.tracks.size());
    for (const TargetStateMsg & track : msg.tracks) {
      safety::KeepOutTarget target;
      target.track_id = track.track_id;
      target.class_id = track.class_id;
      target.position =
        Eigen::Vector3d(track.position.x, track.position.y, track.position.z);
      target.velocity = Eigen::Vector3d(track.velocity.x, track.velocity.y, track.velocity.z);
      target.position_sigma = horizontalSigma(track);
      target.is_valid = track.is_valid;
      tracks_.push_back(target);
    }
  }

  /// The selected target, from /target/state. The filter needs it for the engagement
  /// gate: the class of the track the planner is flying at decides whether the
  /// engagement is allowed at all.
  void onTarget(const TargetStateMsg & msg)
  {
    selected_track_id_ = msg.track_id;
    selected_class_id_ = msg.class_id;
    selected_confirmed_ = msg.confirmed;
    selected_valid_ = msg.is_valid;
    selected_position_ =
      Eigen::Vector3d(msg.position.x, msg.position.y, msg.position.z);
  }

  /// The horizontal position uncertainty of a track [m]: the root mean square of the
  /// two horizontal standard deviations. A poorly tracked person gets a bigger margin.
  static double horizontalSigma(const TargetStateMsg & track)
  {
    const double pxx = track.position_covariance[0];
    const double pyy = track.position_covariance[4];
    return std::sqrt(std::max(0.0, 0.5 * (pxx + pyy)));
  }

  void onOdom(const nav_msgs::msg::Odometry & msg)
  {
    const auto & q = msg.pose.pose.orientation;
    const Eigen::Quaterniond orientation(q.w, q.x, q.y, q.z);
    // nav_msgs/Odometry twist is expressed in the child (body) frame
    const auto & v = msg.twist.twist.linear;
    state_.velocity = orientation.normalized() * Eigen::Vector3d(v.x, v.y, v.z);
    state_.position =
      Eigen::Vector3d(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
    state_.yaw = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    odom_time_ = now().seconds();
    has_odom_ = true;
  }

  void onMode(const MissionModeMsg & msg)
  {
    mode_ = msg.mode;
    armed_ = msg.armed;
    has_mode_ = true;
  }

  void onEstop(const BoolMsg & msg)
  {
    // A true e-stop latches: the filter holds until it is released with a false.
    if (msg.data) {
      if (!estop_latched_) {
        RCLCPP_WARN(get_logger(), "E-stop latched: the filter holds position");
      }
      estop_latched_ = true;
    } else {
      estop_latched_ = false;
    }
  }

  /// The INTERCEPT engagement gate (plan section 4.4).
  ///
  /// The gate is enforced here so a planner bug cannot bypass it. P3.4 adds the
  /// altitude and geofence conditions and the abort logic; the class check below is
  /// the one that must never be relaxed: a person is never a valid target, whatever
  /// else is configured.
  bool engagementAllowed() const
  {
    if (mode_ != static_cast<uint8_t>(MissionMode::INTERCEPT)) {
      return true;
    }
    if (!armed_) {
      return false;
    }
    // The selected target has to be a uav, confirmed and valid.
    if (selected_class_id_ != static_cast<int32_t>(TargetClass::UAV)) {
      return false;
    }
    return selected_confirmed_ && selected_valid_;
  }

  safety::FilterMode currentMode() const
  {
    switch (static_cast<MissionMode>(mode_)) {
      case MissionMode::FOLLOW:
        return safety::FilterMode::FOLLOW;
      case MissionMode::INTERCEPT:
        return safety::FilterMode::INTERCEPT;
      case MissionMode::HOLD:
        break;
    }
    return safety::FilterMode::HOLD;
  }

  void onTimer()
  {
    if (!has_odom_) {
      return;  // nothing to fly yet
    }

    const double now_s = now().seconds();
    const double setpoint_age = has_setpoint_ ? now_s - setpoint_time_ :
      std::numeric_limits<double>::infinity();
    const double odom_age = now_s - odom_time_;

    safety::FilterSetpoint setpoint = setpoint_;
    if (!has_setpoint_) {
      // No setpoint at all: the filter holds. The age is infinite, so the filter's
      // own stale-setpoint check does the work.
      setpoint.velocity = Eigen::Vector3d::Zero();
    }

    const safety::FilterResult result = filter_->filter(
      setpoint, state_, tracks_, obstacles_, currentMode(), now_s, setpoint_age, odom_age,
      estop_latched_, engagementAllowed());

    publishSetpoint(result);
    publishStatus(result);
    publishMarkers();
  }

  void publishSetpoint(const safety::FilterResult & result)
  {
    FlightSetpointMsg msg;
    msg.header.stamp = now();
    msg.header.frame_id = "map";
    // The filter always works on a velocity: the controller must not add a position
    // term of its own, or it would push the drone towards the setpoint behind the
    // filter's back.
    msg.position_valid = false;
    msg.velocity.x = result.velocity.x();
    msg.velocity.y = result.velocity.y();
    msg.velocity.z = result.velocity.z();
    msg.acceleration_ff.x = 0.0;
    msg.acceleration_ff.y = 0.0;
    msg.acceleration_ff.z = 0.0;
    msg.yaw = result.yaw;
    msg.yaw_rate = result.yaw_rate;
    msg.source = result.safety_active ? FlightSetpointMsg::SOURCE_HOLD : setpoint_.source;

    pub_setpoint_->publish(msg);
  }

  void publishStatus(const safety::FilterResult & result)
  {
    MissionStatusMsg msg;
    msg.header.stamp = now();
    msg.header.frame_id = "map";
    msg.mode = mode_;
    msg.phase = phaseFor(result);
    msg.target_track_id = selected_track_id_;
    msg.distance_to_target = distanceToSelected();
    msg.safety_active = result.safety_active;
    msg.reason = result.reason;

    pub_status_->publish(msg);
  }

  /// The phase the filter can see. The planners own the detailed phase; this is the
  /// filter's view of it, for the status overlay.
  uint8_t phaseFor(const safety::FilterResult & result) const
  {
    if (result.safety_active && estop_latched_) {
      return MissionStatusMsg::PHASE_IDLE;
    }
    switch (currentMode()) {
      case safety::FilterMode::FOLLOW:
        return MissionStatusMsg::PHASE_TRACKING;
      case safety::FilterMode::INTERCEPT:
        return engagementAllowed() ? MissionStatusMsg::PHASE_PURSUE :
               MissionStatusMsg::PHASE_SEARCH;
      case safety::FilterMode::HOLD:
        break;
    }
    return MissionStatusMsg::PHASE_IDLE;
  }

  double distanceToSelected() const
  {
    if (selected_track_id_ < 0) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    return (selected_position_ - state_.position).norm();
  }

  /// The keep-out circles, so the margins are visible in RViz.
  void publishMarkers()
  {
    MarkerArrayMsg array;
    int id = 0;
    for (const safety::KeepOutTarget & target : filter_->activeKeepOuts()) {
      visualization_msgs::msg::Marker circle;
      circle.header.stamp = now();
      circle.header.frame_id = "map";
      circle.ns = "keepout_track";
      circle.id = id++;
      circle.type = visualization_msgs::msg::Marker::CYLINDER;
      circle.action = visualization_msgs::msg::Marker::ADD;
      circle.pose.position.x = target.position.x();
      circle.pose.position.y = target.position.y();
      circle.pose.position.z = 0.05;
      circle.pose.orientation.w = 1.0;
      const double radius = filter_->params().d_min_horizontal +
        filter_->params().uncertainty_gain * target.position_sigma;
      circle.scale.x = 2.0 * radius;
      circle.scale.y = 2.0 * radius;
      circle.scale.z = 0.1;
      circle.color.r = 1.0f;
      circle.color.g = 0.3f;
      circle.color.b = 0.1f;
      circle.color.a = 0.25f;
      array.markers.push_back(circle);
    }
    for (const safety::Obstacle & obstacle : obstacles_) {
      visualization_msgs::msg::Marker circle;
      circle.header.stamp = now();
      circle.header.frame_id = "map";
      circle.ns = "keepout_obstacle";
      circle.id = id++;
      circle.type = visualization_msgs::msg::Marker::CYLINDER;
      circle.action = visualization_msgs::msg::Marker::ADD;
      circle.pose.position.x = obstacle.x;
      circle.pose.position.y = obstacle.y;
      circle.pose.position.z = 0.05;
      circle.pose.orientation.w = 1.0;
      const double radius = obstacle.radius + filter_->params().d_obstacle_min;
      circle.scale.x = 2.0 * radius;
      circle.scale.y = 2.0 * radius;
      circle.scale.z = 0.1;
      circle.color.r = 0.9f;
      circle.color.g = 0.9f;
      circle.color.b = 0.2f;
      circle.color.a = 0.25f;
      array.markers.push_back(circle);
    }
    pub_markers_->publish(array);
  }

  // The filter and its inputs
  std::unique_ptr<safety::SafetyFilter> filter_;
  std::vector<safety::Obstacle> obstacles_;
  std::vector<safety::KeepOutTarget> tracks_;
  safety::FilterSetpoint setpoint_;
  safety::FilterState state_;

  // Timing
  double setpoint_time_ = 0.0;
  double odom_time_ = 0.0;
  bool has_setpoint_ = false;
  bool has_odom_ = false;

  // Mission
  uint8_t mode_ = static_cast<uint8_t>(MissionMode::HOLD);
  bool armed_ = false;
  bool has_mode_ = false;
  bool estop_latched_ = false;

  // The selected target, from /target/state
  int32_t selected_track_id_ = -1;
  int32_t selected_class_id_ = -1;
  bool selected_confirmed_ = false;
  bool selected_valid_ = false;
  Eigen::Vector3d selected_position_ = Eigen::Vector3d::Zero();

  rclcpp::Publisher<FlightSetpointMsg>::SharedPtr pub_setpoint_;
  rclcpp::Publisher<MissionStatusMsg>::SharedPtr pub_status_;
  rclcpp::Publisher<MarkerArrayMsg>::SharedPtr pub_markers_;
  rclcpp::Subscription<FlightSetpointMsg>::SharedPtr setpoint_sub_;
  rclcpp::Subscription<TargetStateArrayMsg>::SharedPtr tracks_sub_;
  rclcpp::Subscription<TargetStateMsg>::SharedPtr target_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<MissionModeMsg>::SharedPtr mode_sub_;
  rclcpp::Subscription<BoolMsg>::SharedPtr estop_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace interceptor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<interceptor::SafetyFilterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
