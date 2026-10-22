/**
 * @file target_drone_behavior_node.cpp
 * @brief Flies the target drone (the intruder) with a selectable behaviour.
 *
 * The target drone is a kinematic model: this node integrates the behaviour it is
 * given and pushes the pose into Gazebo with /set_entity_state. The guidance never
 * sees this node's output: it only ever reads the tracker's estimate on
 * /target/state, so the whole perception and estimation chain stays in the loop.
 *
 * The behaviour is selected with the "behaviour" parameter and can be switched while
 * the node runs:
 *
 *     ros2 param set /target_drone_behavior_node behaviour weave
 *
 * The behaviours (plan 3.1) are hover, waypoints, straight, weave and curve; see
 * include/guidance/target_drone_behaviour.hpp for what they do and for the two
 * properties they keep: the published position is the exact integral of the
 * published velocity, and the published velocity is continuous.
 *
 * The node keeps the American spelling of its name because the name is also the
 * name of the ROS node the launch files start; the library uses the spelling of
 * the plan.
 */

#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <gazebo_msgs/srv/set_entity_state.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "guidance/target_drone_behaviour.hpp"

namespace interceptor
{

using SetEntityState = gazebo_msgs::srv::SetEntityState;

namespace
{

/// The value of a parameter that may be an integer or a double.
std::optional<double> parameterToDouble(const rclcpp::Parameter & parameter)
{
  if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
    return parameter.as_double();
  }
  if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
    return static_cast<double>(parameter.as_int());
  }
  return std::nullopt;
}

/// The setters of the tunable parameters of a behaviour.
///
/// The map is built from the parameter struct, so a value that is set at runtime is
/// applied exactly like the one that comes from the config file.
using DoubleSetter = std::function<void (double)>;

std::map<std::string, DoubleSetter> behaviourSetters(guidance::TargetBehaviourParams & params)
{
  return {
    {"start_x", [&params](double v) {params.start_position.x() = v;}},
    {"start_y", [&params](double v) {params.start_position.y() = v;}},
    {"start_z", [&params](double v) {params.start_position.z() = v;}},
    {"start_yaw", [&params](double v) {params.start_yaw = v;}},
    {"hover_drift", [&params](double v) {params.hover_drift = v;}},
    {"hover_drift_period", [&params](double v) {params.hover_drift_period = v;}},
    {"waypoint_speed", [&params](double v) {params.waypoint_speed = v;}},
    {"waypoint_turn_radius", [&params](double v) {params.waypoint_turn_radius = v;}},
    {"straight_speed", [&params](double v) {params.straight_speed = v;}},
    {"straight_direction_yaw", [&params](double v) {params.straight_direction_yaw = v;}},
    {"straight_duration", [&params](double v) {params.straight_duration = v;}},
    {"straight_distance", [&params](double v) {params.straight_distance = v;}},
    {"straight_ramp_time", [&params](double v) {params.straight_ramp_time = v;}},
    {"weave_speed", [&params](double v) {params.weave_speed = v;}},
    {"weave_amplitude", [&params](double v) {params.weave_amplitude = v;}},
    {"weave_frequency", [&params](double v) {params.weave_frequency = v;}},
    {"weave_direction_yaw", [&params](double v) {params.weave_direction_yaw = v;}},
    {"final_speed", [&params](double v) {params.final_speed = v;}},
    {"accel_time", [&params](double v) {params.accel_time = v;}},
    {"turn_rate", [&params](double v) {params.turn_rate = v;}},
    {"vertical_rate", [&params](double v) {params.vertical_rate = v;}},
  };
}

}  // namespace

class TargetDroneBehaviorNode : public rclcpp::Node
{
public:
  explicit TargetDroneBehaviorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("target_drone_behavior_node", options)
  {
    entity_name_ = declare_parameter("entity_name", "target_drone");
    rate_hz_ = declare_parameter("rate_hz", 50.0);
    // The frames of the target drone's own TF tree. The drone is moved by
    // /set_entity_state, so nothing else publishes its pose: this node is the
    // only source of the world -> target_base_link transform, and without it
    // robot_state_publisher has no root to hang the propeller frames on.
    world_frame_ = declare_parameter("world_frame", "map");
    base_frame_ = declare_parameter("base_frame", "target_base_link");

    // The behaviour and everything it is tuned with.
    behaviour_params_ = readBehaviourParams();
    behaviour_ = std::make_unique<guidance::TargetDroneBehaviour>(behaviour_params_);

    client_ = create_client<SetEntityState>("/set_entity_state");
    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("/target_drone/odom", 10);
    // The true acceleration of the intruder. It is not fed to the guidance: the
    // guidance only ever reads the tracker's estimate. It is published so the
    // profile can be checked against what the tracker believes.
    pub_accel_ = create_publisher<geometry_msgs::msg::AccelStamped>("/target_drone/accel", 10);
    // The propellers are visual only: the target is kinematic, so the joint angles
    // are published here and read by robot_state_publisher.
    pub_joints_ = create_publisher<sensor_msgs::msg::JointState>("/target_drone/joint_states", 10);

    // The target drone's pose. The interceptor's own dynamics plugin publishes
    // odom -> base_link, so the target needs the same treatment: a map ->
    // target_base_link transform, broadcast from here.
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // On the node clock, so the behaviour follows simulation time.
    resetTimer();

    // The behaviour can be switched while the node runs:
    //   ros2 param set /target_drone_behavior_node behaviour weave
    parameter_callback_handle_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        return onParametersSet(parameters);
      });

    RCLCPP_INFO(
      get_logger(), "Target drone '%s': behaviour '%s' from (%.1f, %.1f, %.1f) m, "
      "%zu waypoints", entity_name_.c_str(),
      guidance::toString(behaviour_params_.behaviour).c_str(),
      behaviour_params_.start_position.x(), behaviour_params_.start_position.y(),
      behaviour_params_.start_position.z(), behaviour_params_.waypoints.size());
  }

private:
  /// (Re)creates the timer that integrates the behaviour.
  void resetTimer()
  {
    timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::duration<double>(1.0 / std::max(rate_hz_, 1.0)),
      [this]() {onTimer();});
  }

  /// Reads the behaviour and its parameters from the node.
  guidance::TargetBehaviourParams readBehaviourParams()
  {
    guidance::TargetBehaviourParams params;

    const std::string behaviour = declare_parameter("behaviour", "curve");
    const auto parsed = guidance::parseTargetBehaviour(behaviour);
    if (parsed.has_value()) {
      params.behaviour = *parsed;
    } else {
      RCLCPP_ERROR(
        get_logger(), "Unknown behaviour '%s', falling back to 'curve'", behaviour.c_str());
    }

    params.start_position = Eigen::Vector3d(
      declare_parameter("start_x", 10.0),
      declare_parameter("start_y", 5.0),
      declare_parameter("start_z", 12.0));
    params.start_yaw = declare_parameter("start_yaw", 0.0);

    // HOVER
    params.hover_drift = declare_parameter("hover_drift", 0.0);
    params.hover_drift_period = declare_parameter("hover_drift_period", 20.0);

    // WAYPOINTS. The waypoints are a flat list of x, y, z triples, the way the
    // occlusion cylinders are written in groundtruth_params.yaml.
    const std::vector<double> flat_waypoints = declare_parameter(
      "waypoints", std::vector<double>{10.0, 5.0, 12.0, 30.0, 5.0, 12.0, 30.0, 20.0, 12.0});
    for (size_t i = 0; i + 2 < flat_waypoints.size(); i += 3) {
      params.waypoints.emplace_back(
        flat_waypoints[i], flat_waypoints[i + 1],
        flat_waypoints[i + 2]);
    }
    if (flat_waypoints.size() % 3 != 0) {
      RCLCPP_WARN(
        get_logger(), "The waypoints list has %zu values, which is not a whole number of x/y/z "
        "triples: the last one is dropped", flat_waypoints.size());
    }
    params.waypoint_speed = declare_parameter("waypoint_speed", 6.0);
    params.waypoint_turn_radius = declare_parameter("waypoint_turn_radius", 3.0);

    // STRAIGHT
    params.straight_speed = declare_parameter("straight_speed", 10.0);
    params.straight_direction_yaw = declare_parameter("straight_direction_yaw", 0.0);
    params.straight_duration = declare_parameter("straight_duration", 30.0);
    params.straight_distance = declare_parameter("straight_distance", 0.0);
    params.straight_ramp_time = declare_parameter("straight_ramp_time", 1.0);

    // WEAVE
    params.weave_speed = declare_parameter("weave_speed", 6.0);
    params.weave_amplitude = declare_parameter("weave_amplitude", 2.0);
    params.weave_frequency = declare_parameter("weave_frequency", 0.25);
    params.weave_direction_yaw = declare_parameter("weave_direction_yaw", 0.0);

    // CURVE
    params.final_speed = declare_parameter("final_speed", 6.0);
    params.accel_time = declare_parameter("accel_time", 4.0);
    params.turn_rate = declare_parameter("turn_rate", 1.2);
    params.vertical_rate = declare_parameter("vertical_rate", 0.0);

    return params;
  }

  /// Applies the parameters that are set while the node runs.
  ///
  /// A switch of the behaviour restarts the run: the intruder goes back to the
  /// start pose of the new behaviour.
  rcl_interfaces::msg::SetParametersResult onParametersSet(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    guidance::TargetBehaviourParams updated = behaviour_params_;

    for (const auto & parameter : parameters) {
      const std::string & name = parameter.get_name();
      if (name == "behaviour") {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
          RCLCPP_ERROR(get_logger(), "The behaviour has to be a string");
          result.successful = false;
          result.reason = "the behaviour has to be a string";
          return result;
        }
        const auto parsed = guidance::parseTargetBehaviour(parameter.as_string());
        if (!parsed.has_value()) {
          RCLCPP_ERROR(
            get_logger(), "Unknown behaviour '%s': hover, waypoints, straight, weave or curve",
            parameter.as_string().c_str());
          result.successful = false;
          result.reason = "unknown behaviour";
          return result;
        }
        updated.behaviour = *parsed;
        continue;
      }
      if (name == "waypoints") {
        std::vector<double> flat;
        if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
          flat = parameter.as_double_array();
        } else if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
          for (const int64_t value : parameter.as_integer_array()) {
            flat.push_back(static_cast<double>(value));
          }
        } else {
          RCLCPP_ERROR(get_logger(), "The waypoints have to be a list of numbers");
          result.successful = false;
          result.reason = "the waypoints have to be a list of numbers";
          return result;
        }
        updated.waypoints.clear();
        for (size_t i = 0; i + 2 < flat.size(); i += 3) {
          updated.waypoints.emplace_back(flat[i], flat[i + 1], flat[i + 2]);
        }
        continue;
      }
      if (name == "entity_name") {
        entity_name_ = parameter.as_string();
        continue;
      }
      if (name == "world_frame") {
        world_frame_ = parameter.as_string();
        continue;
      }
      if (name == "base_frame") {
        base_frame_ = parameter.as_string();
        continue;
      }
      if (name == "rate_hz") {
        const auto value = parameterToDouble(parameter);
        if (!value.has_value()) {
          result.successful = false;
          result.reason = "the rate has to be a number";
          return result;
        }
        rate_hz_ = *value;
        resetTimer();
        continue;
      }

      // The tunable parameters of the behaviours.
      const auto setters = behaviourSetters(updated);
      const auto setter = setters.find(name);
      if (setter == setters.end()) {
        continue;
      }
      const auto value = parameterToDouble(parameter);
      if (!value.has_value()) {
        RCLCPP_ERROR(get_logger(), "The parameter '%s' has to be a number", name.c_str());
        result.successful = false;
        result.reason = "the parameter has to be a number";
        return result;
      }
      setter->second(*value);
    }

    const bool switched = updated.behaviour != behaviour_params_.behaviour;
    behaviour_params_ = updated;
    behaviour_ = std::make_unique<guidance::TargetDroneBehaviour>(behaviour_params_);
    if (switched) {
      RCLCPP_INFO(
        get_logger(), "Target drone behaviour switched to '%s', the run restarts at "
        "(%.1f, %.1f, %.1f) m", guidance::toString(behaviour_params_.behaviour).c_str(),
        behaviour_params_.start_position.x(), behaviour_params_.start_position.y(),
        behaviour_params_.start_position.z());
    }
    return result;
  }

  void onTimer()
  {
    const double now_s = now().seconds();
    if (!has_last_time_) {
      has_last_time_ = true;
      last_time_ = now_s;
    }
    // The behaviour is integrated with the time that really passed, so the profile
    // does not depend on the rate of the timer. A long pause (a restarted
    // simulator) is not integrated: the intruder would jump across the world.
    const double dt = std::clamp(now_s - last_time_, 0.0, 0.5);
    last_time_ = now_s;

    const guidance::TargetBehaviourState & state = behaviour_->step(dt);

    sendPose(state.position, state.yaw);
    publishOdom(state.position, state.velocity, state.acceleration, state.yaw);
    publishJoints(now_s);
    publishTransform(state.position, state.yaw);
  }

  /// Broadcasts the world -> target_base_link transform.
  ///
  /// The target drone is moved by /set_entity_state, so no Gazebo plugin
  /// publishes its pose and robot_state_publisher has no root transform to
  /// attach the propeller frames to. Publishing it here completes the target's
  /// TF tree: map -> target_base_link -> target_prop_*_link.
  void publishTransform(const Eigen::Vector3d & position, double yaw)
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now();
    tf.header.frame_id = world_frame_;
    tf.child_frame_id = base_frame_;
    tf.transform.translation.x = position.x();
    tf.transform.translation.y = position.y();
    tf.transform.translation.z = position.z();
    const Eigen::Quaterniond q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(tf);
  }

  /// Spins the propeller joints so the intruder reads as a flying drone.
  void publishJoints(double t)
  {
    sensor_msgs::msg::JointState joints;
    joints.header.stamp = now();
    // The Crazyflie's props turn at about 2600 rad/s, slowed down so the rotation
    // stays visible.
    const double angle = propeller_speed_ * t / propeller_slowdown_;
    for (const auto & name : prop_joint_names_) {
      joints.name.push_back(name);
      joints.position.push_back(angle);
      joints.velocity.push_back(propeller_speed_ / propeller_slowdown_);
    }
    pub_joints_->publish(joints);
  }

  void sendPose(const Eigen::Vector3d & position, double yaw)
  {
    if (!client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Waiting for /set_entity_state");
      return;
    }
    auto request = std::make_shared<SetEntityState::Request>();
    // The entity name and the reference frame live inside the state message.
    request->state.name = entity_name_;
    request->state.reference_frame = "world";
    request->state.pose.position.x = position.x();
    request->state.pose.position.y = position.y();
    request->state.pose.position.z = position.z();
    request->state.pose.orientation.x = 0.0;
    request->state.pose.orientation.y = 0.0;
    request->state.pose.orientation.z = std::sin(yaw * 0.5);
    request->state.pose.orientation.w = std::cos(yaw * 0.5);
    // The target is kinematic: it must not fall under gravity, and it has no
    // velocity of its own, so the twist is left at zero.
    client_->async_send_request(request);
  }

  void publishOdom(
    const Eigen::Vector3d & position, const Eigen::Vector3d & velocity,
    const Eigen::Vector3d & acceleration, double yaw)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now();
    odom.header.frame_id = "map";
    odom.child_frame_id = "target_drone_base_link";
    odom.pose.pose.position.x = position.x();
    odom.pose.pose.position.y = position.y();
    odom.pose.pose.position.z = position.z();
    odom.pose.pose.orientation.z = std::sin(yaw * 0.5);
    odom.pose.pose.orientation.w = std::cos(yaw * 0.5);
    odom.twist.twist.linear.x = velocity.x();
    odom.twist.twist.linear.y = velocity.y();
    odom.twist.twist.linear.z = velocity.z();
    pub_odom_->publish(odom);

    geometry_msgs::msg::AccelStamped accel_msg;
    accel_msg.header.stamp = odom.header.stamp;
    accel_msg.header.frame_id = "map";
    accel_msg.accel.linear.x = acceleration.x();
    accel_msg.accel.linear.y = acceleration.y();
    accel_msg.accel.linear.z = acceleration.z();
    pub_accel_->publish(accel_msg);
  }

  std::string entity_name_;
  std::string world_frame_ = "map";
  std::string base_frame_ = "target_base_link";
  double rate_hz_ = 50.0;
  bool has_last_time_ = false;
  double last_time_ = 0.0;
  double propeller_speed_ = 2618.0;
  double propeller_slowdown_ = 20.0;
  std::vector<std::string> prop_joint_names_ = {
    "target_prop_fr_joint", "target_prop_fl_joint",
    "target_prop_rl_joint", "target_prop_rr_joint"};

  guidance::TargetBehaviourParams behaviour_params_;
  std::unique_ptr<guidance::TargetDroneBehaviour> behaviour_;

  rclcpp::Client<SetEntityState>::SharedPtr client_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr pub_accel_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joints_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace interceptor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<interceptor::TargetDroneBehaviorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
