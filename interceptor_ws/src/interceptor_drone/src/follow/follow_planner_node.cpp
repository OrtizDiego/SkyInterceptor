/**
 * @file follow_planner_node.cpp
 * @brief The FOLLOW planner node: /target/state + /odom -> /setpoint/raw (plan 4.2).
 *
 * The node frames the target from a preset (BEHIND, SIDE, LEAD or ORBIT) expressed in
 * the target's heading frame, and publishes the velocity setpoint that puts the drone
 * there. It also runs the lost-track machine: a target that disappears makes the
 * drone hover first and go back to the position the follow started from later.
 *
 * All the maths lives in the ROS-free library include/follow/follow_planner.hpp; this
 * file only does the plumbing.
 *
 * Topics
 * ------
 *   in   /target/state   TargetState   the selected track
 *   in   /mission/mode   MissionMode   latched by the mission manager
 *   in   /odom           Odometry      the drone's own state
 *   out  /setpoint/raw   FlightSetpoint the planner's command
 *   out  /drone/arm      Bool          the motors, armed once the follow starts
 *
 * The setpoint goes on /setpoint/raw only. safety_filter_node (plan 4.3) is between
 * the planner and the trajectory controller and has the final say, so it owns
 * /setpoint/safe. Publishing on both would put two publishers on the topic the
 * controller reads, and the controller would sometimes fly an unfiltered setpoint.
 *
 * The mode gate: the planner publishes *nothing* when the mode is not FOLLOW. In
 * INTERCEPT the guidance node owns /setpoint/raw, and a second publisher on that
 * topic would make the safety filter alternate between the two commands, which could
 * abort an engagement. Staying silent is safe: the safety filter brakes to hover on
 * its own when the setpoint goes stale.
 *
 * The drone starts disarmed: the mission manager only arms an INTERCEPT mission (that
 * flag is the engagement authorization, not the motors), and the guidance node arms
 * the drone itself in INTERCEPT. In FOLLOW nobody else does it, so the planner arms
 * the motors once it has the odometry and the mode is FOLLOW. Without this the
 * setpoints would command a drone that never takes off.
 */

#include <Eigen/Geometry>

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>

#include <interceptor_interfaces/msg/flight_setpoint.hpp>
#include <interceptor_interfaces/msg/mission_mode.hpp>
#include <interceptor_interfaces/msg/target_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "common/mission.hpp"
#include "follow/follow_planner.hpp"

namespace interceptor
{

using FlightSetpointMsg = interceptor_interfaces::msg::FlightSetpoint;
using MissionModeMsg = interceptor_interfaces::msg::MissionMode;
using TargetStateMsg = interceptor_interfaces::msg::TargetState;
using BoolMsg = std_msgs::msg::Bool;

class FollowPlannerNode : public rclcpp::Node
{
public:
  explicit FollowPlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("follow_planner_node", options)
  {
    const follow::FollowPlannerParams defaults;

    const std::string preset_name = declare_parameter("follow.preset", "BEHIND");
    const auto preset = follow::parseFramingPreset(preset_name);
    if (!preset) {
      throw std::invalid_argument(
              "follow.preset must be BEHIND, SIDE, LEAD or ORBIT, got '" + preset_name + "'");
    }

    follow::FollowPlannerParams params;
    params.preset = *preset;
    params.offset_behind = declare_parameter("follow.offset.behind", defaults.offset_behind);
    params.offset_side = declare_parameter("follow.offset.side", defaults.offset_side);
    params.offset_height = declare_parameter("follow.offset.height", defaults.offset_height);
    params.orbit_rate = declare_parameter("follow.orbit_rate", defaults.orbit_rate);
    params.heading_tau = declare_parameter("follow.heading_tau", defaults.heading_tau);
    params.kp_pos = declare_parameter("follow.kp_pos", defaults.kp_pos);
    params.max_speed = declare_parameter("follow.max_speed", defaults.max_speed);
    params.max_vertical_speed =
      declare_parameter("follow.max_vertical_speed", defaults.max_vertical_speed);
    params.lost_hold_time = declare_parameter("follow.lost_hold_time", defaults.lost_hold_time);
    params.lost_rtl_time = declare_parameter("follow.lost_rtl_time", defaults.lost_rtl_time);
    params.target_timeout = declare_parameter("follow.target_timeout", defaults.target_timeout);
    params.heading_min_speed =
      declare_parameter("follow.heading_min_speed", defaults.heading_min_speed);

    const double rate_hz = declare_parameter("rate_hz", 50.0);
    if (!(rate_hz > 0.0)) {
      throw std::invalid_argument("rate_hz must be positive");
    }
    odom_timeout_ = declare_parameter("odom_timeout", 0.3);
    // The drone starts disarmed and nobody else arms it in the FOLLOW mode.
    arm_on_start_ = declare_parameter("arm_on_start", true);

    planner_ = std::make_unique<follow::FollowPlanner>(params);

    pub_setpoint_raw_ = create_publisher<FlightSetpointMsg>("/setpoint/raw", 10);
    pub_arm_ = create_publisher<BoolMsg>("/drone/arm", 10);

    target_sub_ = create_subscription<TargetStateMsg>(
      "/target/state", 10,
      [this](TargetStateMsg::ConstSharedPtr msg) {onTarget(*msg);});
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {onOdom(*msg);});
    mode_sub_ = create_subscription<MissionModeMsg>(
      "/mission/mode", rclcpp::QoS(1).reliable().transient_local(),
      [this](MissionModeMsg::ConstSharedPtr msg) {onMode(*msg);});

    // On the node clock, so it follows simulation time
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate_hz),
      [this]() {onTimer();});

    RCLCPP_INFO(
      get_logger(),
      "Follow planner started: preset=%s, offset behind/side/height=%.1f/%.1f/%.1f m, "
      "orbit_rate=%.2f rad/s, heading_tau=%.2f s, kp=%.2f, max speed=%.1f m/s, "
      "lost hold/RTL=%.1f/%.1f s, %.0f Hz",
      follow::toString(params.preset).c_str(), params.offset_behind, params.offset_side,
      params.offset_height, params.orbit_rate, params.heading_tau, params.kp_pos,
      params.max_speed, params.lost_hold_time, params.lost_rtl_time, rate_hz);
  }

private:
  void onTarget(const TargetStateMsg & msg)
  {
    follow::FollowTarget target;
    target.track_id = msg.track_id;
    target.position = Eigen::Vector3d(msg.position.x, msg.position.y, msg.position.z);
    target.velocity = Eigen::Vector3d(msg.velocity.x, msg.velocity.y, msg.velocity.z);
    target.heading = msg.heading;
    target.confirmed = msg.confirmed;
    target.is_valid = msg.is_valid;
    planner_->onTarget(target, now().seconds());
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
    state_.yaw = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    odom_time_ = now().seconds();
    has_odom_ = true;
  }

  void onMode(const MissionModeMsg & msg)
  {
    // The operator-selected track: the planner follows exactly that track, and never
    // another one. The tracker applies the same rule when it selects the target.
    planner_->setOperatorTrackId(msg.track_id);
    planner_->setMode(currentMode(msg.mode), now().seconds());
  }

  static follow::FollowMode currentMode(uint8_t mode)
  {
    switch (static_cast<MissionMode>(mode)) {
      case MissionMode::FOLLOW:
        return follow::FollowMode::FOLLOW;
      case MissionMode::INTERCEPT:
        return follow::FollowMode::INTERCEPT;
      case MissionMode::HOLD:
        break;
    }
    return follow::FollowMode::HOLD;
  }

  void onTimer()
  {
    const double t = now().seconds();
    const follow::FollowCommand command = planner_->update(state_, t);

    if (!command.active) {
      // The mode is not FOLLOW: the planner has nothing to say and publishes nothing.
      return;
    }
    if (!has_odom_ || t - odom_time_ > odom_timeout_) {
      // Stale odometry: the planner does not know where the drone is, so it does not
      // publish. The safety filter brakes to hover on a stale setpoint.
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Stale /odom: the follow planner is silent");
      return;
    }

    if (arm_on_start_ && !arm_sent_) {
      // The drone starts disarmed and the mission manager only arms an INTERCEPT
      // mission, so the planner arms the motors itself, the first time it has the
      // odometry and the mode is FOLLOW. Without this the setpoints would command a
      // drone that never takes off.
      arm_sent_ = true;
      BoolMsg arm;
      arm.data = true;
      pub_arm_->publish(arm);
      RCLCPP_INFO(get_logger(), "Arming the drone");
    }

    FlightSetpointMsg msg;
    msg.header.stamp = now();
    msg.header.frame_id = "map";
    // The safety filter and the controller both fly a velocity: the setpoint must not
    // carry a position, or the controller would add a position term of its own on top
    // of the filtered command.
    msg.position_valid = false;
    msg.velocity.x = command.velocity.x();
    msg.velocity.y = command.velocity.y();
    msg.velocity.z = command.velocity.z();
    msg.acceleration_ff.x = 0.0;
    msg.acceleration_ff.y = 0.0;
    msg.acceleration_ff.z = 0.0;
    msg.yaw = command.yaw;
    msg.yaw_rate = 0.0;
    msg.source = command.source;

    pub_setpoint_raw_->publish(msg);
  }

  std::unique_ptr<follow::FollowPlanner> planner_;
  follow::FollowState state_;

  double odom_time_ = 0.0;
  double odom_timeout_ = 0.3;
  bool has_odom_ = false;
  bool arm_on_start_ = true;
  bool arm_sent_ = false;

  rclcpp::Publisher<FlightSetpointMsg>::SharedPtr pub_setpoint_raw_;
  rclcpp::Publisher<BoolMsg>::SharedPtr pub_arm_;
  rclcpp::Subscription<TargetStateMsg>::SharedPtr target_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<MissionModeMsg>::SharedPtr mode_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace interceptor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<interceptor::FollowPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
