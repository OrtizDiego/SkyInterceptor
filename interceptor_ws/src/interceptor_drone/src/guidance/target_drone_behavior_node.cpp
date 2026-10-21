/**
 * @file target_drone_behavior_node.cpp
 * @brief Flies the target drone (the intruder) with an accelerating, curving profile.
 *
 * The target drone is a kinematic model: this node integrates its trajectory and pushes
 * the pose into Gazebo with /set_entity_state. The guidance never sees this node's
 * output: it only ever reads the tracker's estimate on /target/state, so the whole
 * perception and estimation chain stays in the loop.
 *
 * The profile is the interesting part for the demo: the drone starts nearly stationary,
 * accelerates to its cruise speed while it flies, and turns at a constant rate the whole
 * time. That combination is what pure proportional navigation cannot follow, and what
 * augmented PN compensates.
 */

#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <gazebo_msgs/srv/set_entity_state.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace interceptor
{

using SetEntityState = gazebo_msgs::srv::SetEntityState;

class TargetDroneBehaviorNode : public rclcpp::Node
{
public:
  explicit TargetDroneBehaviorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("target_drone_behavior_node", options)
  {
    entity_name_ = declare_parameter("entity_name", "target_drone");
    start_x_ = declare_parameter("start_x", 20.0);
    start_y_ = declare_parameter("start_y", 0.0);
    start_z_ = declare_parameter("start_z", 12.0);
    start_yaw_ = declare_parameter("start_yaw", 0.0);

    // The acceleration profile
    final_speed_ = declare_parameter("final_speed", 6.0);
    accel_time_ = declare_parameter("accel_time", 6.0);
    turn_rate_ = declare_parameter("turn_rate", 0.35);  // rad/s, constant curve
    // The drone flies level: the interceptor has to match the altitude itself.
    vertical_rate_ = declare_parameter("vertical_rate", 0.0);

    const double rate_hz = declare_parameter("rate_hz", 50.0);
    // The frames of the target drone's own TF tree. The drone is moved by
    // /set_entity_state, so nothing else publishes its pose: this node is the
    // only source of the world -> target_base_link transform, and without it
    // robot_state_publisher has no root to hang the propeller frames on.
    world_frame_ = declare_parameter("world_frame", "map");
    base_frame_ = declare_parameter("base_frame", "target_base_link");

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

    // On the node clock, so the profile follows simulation time.
    timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::duration<double>(1.0 / std::max(rate_hz, 1.0)),
      [this]() {onTimer();});

    RCLCPP_INFO(
      get_logger(),
      "Target drone '%s': accelerate 0 -> %.1f m/s over %.1f s, turn rate %.2f rad/s",
      entity_name_.c_str(), final_speed_, accel_time_, turn_rate_);
  }

private:
  /// The state of the intruder at time t, integrated from the start.
  ///
  /// The manoeuvre: the drone starts nearly stationary and accelerates to
  /// final_speed over accel_time, while it turns at a constant turn_rate the
  /// whole time. The acceleration therefore has two parts: the tangential one
  /// from the speed ramp and the lateral one from the turn. That combination is
  /// what pure proportional navigation cannot follow and what augmented PN
  /// compensates.
  ///
  /// The position is the exact integral of the velocity, so the published
  /// velocity really is d(position)/dt: the tracker's estimate of the target's
  /// velocity and acceleration then matches what the drone actually does.
  void stateAt(
    double t, Eigen::Vector3d & position, Eigen::Vector3d & velocity,
    Eigen::Vector3d & acceleration, double & yaw) const
  {
    const double ramp_time = std::max(accel_time_, 1e-6);
    const double heading0 = start_yaw_;
    const double turn_rate = turn_rate_;

    // The heading turns at a constant rate from the start.
    const double heading = heading0 + turn_rate * t;
    const double cos_h = std::cos(heading);
    const double sin_h = std::sin(heading);

    // Speed: the ramp, then constant.
    const bool ramping = t < ramp_time;
    const double speed = ramping ? final_speed_ * t / ramp_time : final_speed_;
    const double speed_rate = ramping ? final_speed_ / ramp_time : 0.0;

    velocity = Eigen::Vector3d(speed * cos_h, speed * sin_h, vertical_rate_);

    // d(velocity)/dt: the speed ramp along the heading plus the turn rate across it.
    acceleration = Eigen::Vector3d(
      speed_rate * cos_h - speed * turn_rate * sin_h,
      speed_rate * sin_h + speed * turn_rate * cos_h,
      0.0);

    // Position: the exact integral of the velocity.
    double dx = 0.0;
    double dy = 0.0;
    if (std::abs(turn_rate) > 1e-9) {
      // The ramp: int_0^{t1} (v/T) * t * [cos(h0 + w t), sin(h0 + w t)] dt
      const double t1 = std::min(t, ramp_time);
      const double h1 = heading0 + turn_rate * t1;
      const double k = final_speed_ / (ramp_time * turn_rate * turn_rate);
      dx += k * ((h1 - heading0) * std::sin(h1) + std::cos(h1) - std::cos(heading0));
      dy += k * (-(h1 - heading0) * std::cos(h1) + std::sin(h1) - std::sin(heading0));

      // After the ramp the speed is constant, so the path is a circular arc.
      if (t > ramp_time) {
        const double radius = final_speed_ / turn_rate;
        dx += radius * (std::sin(heading) - std::sin(h1));
        dy += -radius * (std::cos(heading) - std::cos(h1));
      }
    } else {
      // No turn: a straight line.
      const double t1 = std::min(t, ramp_time);
      const double distance =
        0.5 * final_speed_ * t1 * t1 / ramp_time + final_speed_ * std::max(0.0, t - ramp_time);
      dx = distance * std::cos(heading0);
      dy = distance * std::sin(heading0);
    }

    position = Eigen::Vector3d(start_x_ + dx, start_y_ + dy, start_z_ + vertical_rate_ * t);
    yaw = heading;
  }

  void onTimer()
  {
    if (!has_start_time_) {
      // The profile starts when the node gets its first tick, so the drone always
      // begins at the configured start pose.
      has_start_time_ = true;
      start_time_ = now().seconds();
    }
    const double t = now().seconds() - start_time_;

    Eigen::Vector3d position, velocity, acceleration;
    double yaw = 0.0;
    stateAt(t, position, velocity, acceleration, yaw);

    sendPose(position, yaw);
    publishOdom(position, velocity, acceleration, yaw);
    publishJoints(t);
    publishTransform(position, yaw);
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
  double start_x_ = 20.0;
  double start_y_ = 0.0;
  double start_z_ = 12.0;
  double start_yaw_ = 0.0;
  double final_speed_ = 6.0;
  double accel_time_ = 6.0;
  double turn_rate_ = 0.35;
  double vertical_rate_ = 0.0;
  bool has_start_time_ = false;
  double start_time_ = 0.0;
  double propeller_speed_ = 2618.0;
  double propeller_slowdown_ = 20.0;
  std::vector<std::string> prop_joint_names_ = {
    "target_prop_fr_joint", "target_prop_fl_joint",
    "target_prop_rl_joint", "target_prop_rr_joint"};

  rclcpp::Client<SetEntityState>::SharedPtr client_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr pub_accel_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joints_;
  rclcpp::TimerBase::SharedPtr timer_;
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
