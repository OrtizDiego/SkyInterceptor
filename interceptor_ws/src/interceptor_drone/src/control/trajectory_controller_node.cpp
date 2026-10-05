/**
 * @file trajectory_controller_node.cpp
 * @brief /setpoint/safe + /odom -> /cmd_vel (plan 4.5). Thin ROS wrapper around
 *        control::TrajectoryController: world-frame velocity and yaw rate out.
 */

#include <Eigen/Geometry>

#include <algorithm>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <interceptor_interfaces/msg/flight_setpoint.hpp>
#include <interceptor_interfaces/msg/mission_mode.hpp>

#include "control/trajectory_controller.hpp"
#include "flight/flight_controller.hpp"

namespace interceptor
{

using FlightSetpointMsg = interceptor_interfaces::msg::FlightSetpoint;
using MissionModeMsg = interceptor_interfaces::msg::MissionMode;

class TrajectoryControllerNode : public rclcpp::Node
{
public:
  explicit TrajectoryControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("trajectory_controller_node", options)
  {
    const control::TrajectoryControllerParams defaults;
    control::TrajectoryControllerParams params;
    params.kp_pos = declare_parameter("kp_pos", defaults.kp_pos);
    params.ki_pos = declare_parameter("ki_pos", defaults.ki_pos);
    params.kd_pos = declare_parameter("kd_pos", defaults.kd_pos);
    params.kb_antiwindup = declare_parameter("kb_antiwindup", defaults.kb_antiwindup);
    params.max_integral_velocity =
      declare_parameter("max_integral_velocity", defaults.max_integral_velocity);
    params.integral_zone = declare_parameter("integral_zone", defaults.integral_zone);
    params.kp_yaw = declare_parameter("kp_yaw", defaults.kp_yaw);
    params.max_horizontal_velocity =
      declare_parameter("max_horizontal_velocity", defaults.max_horizontal_velocity);
    params.max_vertical_velocity =
      declare_parameter("max_vertical_velocity", defaults.max_vertical_velocity);
    params.max_yaw_rate = declare_parameter("max_yaw_rate", defaults.max_yaw_rate);
    params.setpoint_timeout = declare_parameter("setpoint_timeout", defaults.setpoint_timeout);
    odom_timeout_ = declare_parameter("odom_timeout", 0.3);
    const double rate_hz = declare_parameter("rate_hz", 50.0);
    controller_ = std::make_unique<control::TrajectoryController>(params);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    setpoint_sub_ = create_subscription<FlightSetpointMsg>(
      "/setpoint/safe", 10,
      [this](FlightSetpointMsg::ConstSharedPtr msg) {onSetpoint(*msg);});
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {onOdom(*msg);});
    mode_sub_ = create_subscription<MissionModeMsg>(
      "/mission/mode", rclcpp::QoS(1).transient_local(),
      [this](MissionModeMsg::ConstSharedPtr msg) {onMode(*msg);});

    // On the node clock, so it follows simulation time
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / std::max(rate_hz, 1.0)),
      [this]() {onTimer();});
    RCLCPP_INFO(get_logger(), "Trajectory controller started (%.0f Hz)", rate_hz);
  }

private:
  void onSetpoint(const FlightSetpointMsg & msg)
  {
    control::ControllerSetpoint sp;
    sp.position = Eigen::Vector3d(msg.position.x, msg.position.y, msg.position.z);
    sp.velocity = Eigen::Vector3d(msg.velocity.x, msg.velocity.y, msg.velocity.z);
    sp.yaw = msg.yaw;
    sp.yaw_rate = msg.yaw_rate;
    sp.position_valid = msg.position_valid;
    sp.source = msg.source;
    if (!controller_->setSetpoint(sp, now().seconds())) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Ignoring /setpoint/safe with non-finite values");
    }
  }

  void onOdom(const nav_msgs::msg::Odometry & msg)
  {
    const auto & q = msg.pose.pose.orientation;
    const Eigen::Quaterniond orientation(q.w, q.x, q.y, q.z);
    // nav_msgs/Odometry twist is expressed in the child (body) frame
    const auto & v = msg.twist.twist.linear;
    state_.velocity = orientation.normalized() * Eigen::Vector3d(v.x, v.y, v.z);
    state_.position = Eigen::Vector3d(
      msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
    state_.yaw = flight::yawFromQuaternion(orientation.normalized());
    odom_time_ = now().seconds();
    has_odom_ = true;
  }

  void onMode(const MissionModeMsg & msg)
  {
    if (has_mode_ && (msg.mode != mode_ || msg.armed != armed_)) {
      controller_->resetIntegrator();
    }
    has_mode_ = true;
    mode_ = msg.mode;
    armed_ = msg.armed;
  }

  void onTimer()
  {
    const double t = now().seconds();
    if (!has_odom_) {
      return;  // nothing to control yet
    }
    control::ControllerCommand command;
    if (t - odom_time_ > odom_timeout_) {
      command = controller_->hold();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Stale /odom: commanding zero velocity");
    } else {
      command = controller_->update(state_, t);
    }

    geometry_msgs::msg::Twist twist;
    twist.linear.x = command.velocity.x();
    twist.linear.y = command.velocity.y();
    twist.linear.z = command.velocity.z();
    twist.angular.z = command.yaw_rate;
    cmd_pub_->publish(twist);
  }

  std::unique_ptr<control::TrajectoryController> controller_;
  control::ControllerState state_;
  double odom_time_ = 0.0;
  double odom_timeout_ = 0.3;
  bool has_odom_ = false;
  bool has_mode_ = false;
  uint8_t mode_ = 0;
  bool armed_ = false;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<FlightSetpointMsg>::SharedPtr setpoint_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<MissionModeMsg>::SharedPtr mode_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace interceptor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<interceptor::TrajectoryControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
