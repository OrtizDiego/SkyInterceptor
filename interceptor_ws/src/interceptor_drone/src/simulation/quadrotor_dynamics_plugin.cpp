// Gazebo Classic model plugin: quadrotor flight dynamics.
//
// Each physics step (1 kHz):
//   /cmd_vel -> FlightController (onboard autopilot) -> rotor speed commands
//   -> first-order motor model -> rotor thrust, reaction torque and rotor drag,
//      airframe drag, wind and ground effect -> force/torque on the body link.
// Gazebo integrates the rigid body (gravity, inertia, collisions with the world).
// The plugin also publishes /odom, the odom -> base_link TF, /joint_states for the
// spinning propellers, the flight status (DISARMED / LANDED / FLYING) and the wind.

#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Inertial.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include "flight/flight_controller.hpp"
#include "flight/quadrotor_model.hpp"
#include "flight/wind_model.hpp"

namespace interceptor
{

namespace
{

Eigen::Vector3d toEigen(const ignition::math::Vector3d & v)
{
  return Eigen::Vector3d(v.X(), v.Y(), v.Z());
}

ignition::math::Vector3d toIgn(const Eigen::Vector3d & v)
{
  return ignition::math::Vector3d(v.x(), v.y(), v.z());
}

template<typename T>
T sdfParam(const sdf::ElementPtr & sdf, const std::string & name, const T & fallback)
{
  return sdf->Get<T>(name, fallback).first;
}

}  // namespace

class QuadrotorDynamicsPlugin : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    world_ = model->GetWorld();
    ros_node_ = gazebo_ros::Node::Get(sdf);
    const auto logger = ros_node_->get_logger();

    const std::string body_name = sdfParam<std::string>(sdf, "body_link", "base_link");
    body_ = model->GetLink(body_name);
    if (!body_) {
      RCLCPP_ERROR(logger, "Body link '%s' not found, plugin disabled", body_name.c_str());
      return;
    }

    flight::QuadrotorParams vehicle;
    if (!loadRotors(sdf, vehicle)) {
      return;
    }
    loadVehicleParams(sdf, vehicle);
    plant_ = std::make_unique<flight::QuadrotorModel>(vehicle);
    controller_ = std::make_unique<flight::FlightController>(
      vehicle, loadControllerParams(sdf));

    flight::WindParams wind;
    wind.mean = toEigen(sdfParam(sdf, "wind_mean", ignition::math::Vector3d::Zero));
    wind.gust_stddev = sdfParam(sdf, "wind_gust_stddev", 0.0);
    wind.gust_time_constant = sdfParam(sdf, "wind_gust_time_constant", 2.0);
    wind.seed = sdfParam(sdf, "wind_seed", 42u);
    wind_ = std::make_unique<flight::WindModel>(wind);

    command_in_world_frame_ = sdfParam<std::string>(sdf, "command_frame", "heading") == "world";
    command_timeout_ = sdfParam(sdf, "command_timeout", 0.5);
    rotor_visual_slowdown_ = std::max(sdfParam(sdf, "rotor_visual_slowdown", 10.0), 1.0);
    odom_frame_ = sdfParam<std::string>(sdf, "odom_frame", "odom");
    base_frame_ = sdfParam<std::string>(sdf, "base_frame", body_name);
    publish_tf_ = sdfParam(sdf, "publish_tf", true);
    odom_period_ = 1.0 / std::max(sdfParam(sdf, "odom_rate", 100.0), 1.0);
    joint_state_period_ = 1.0 / std::max(sdfParam(sdf, "joint_state_rate", 30.0), 1.0);
    const bool start_armed = sdfParam(sdf, "start_armed", false);

    setupRos();

    last_update_time_ = world_->SimTime().Double();
    if (start_armed) {
      setArmed(true, readState());
    }
    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&QuadrotorDynamicsPlugin::onUpdate, this, std::placeholders::_1));

    RCLCPP_INFO(
      logger, "Quadrotor dynamics loaded: mass %.3f kg, hover %.0f rad/s, wind (%.1f %.1f %.1f) "
      "m/s, gusts %.2f m/s. Publish std_msgs/Bool true on drone/arm to start the motors.",
      vehicle.mass,
      std::sqrt(vehicle.mass * flight::kGravity / (4.0 * vehicle.thrust_coefficient)),
      wind.mean.x(), wind.mean.y(), wind.mean.z(), wind.gust_stddev);
  }

  void Reset() override
  {
    if (!plant_) {
      return;
    }
    armed_ = false;
    plant_->setRotorSpeeds(flight::RotorArray{});
    wind_->reset();
    last_update_time_ = world_->SimTime().Double();
    last_cmd_time_ = last_odom_time_ = last_joint_time_ = -1.0;
    std::lock_guard<std::mutex> lock(mutex_);
    has_pending_cmd_ = has_pending_arm_ = has_pending_wind_ = false;
  }

private:
  bool loadRotors(const sdf::ElementPtr & sdf, flight::QuadrotorParams & vehicle)
  {
    const auto logger = ros_node_->get_logger();
    sdf::ElementPtr rotor = sdf->HasElement("rotor") ? sdf->GetElement("rotor") : nullptr;
    const ignition::math::Pose3d body_pose = body_->WorldPose();
    int count = 0;
    for (; rotor && count < flight::kNumRotors; rotor = rotor->GetNextElement("rotor")) {
      const std::string joint_name = sdfParam<std::string>(rotor, "joint", "");
      gazebo::physics::JointPtr joint = model_->GetJoint(joint_name);
      if (!joint) {
        RCLCPP_ERROR(logger, "Rotor joint '%s' not found, plugin disabled", joint_name.c_str());
        return false;
      }
      // Rotor hub = propeller link origin, expressed in the body frame
      const ignition::math::Vector3d offset = body_pose.Rot().RotateVectorReverse(
        joint->GetChild()->WorldPose().Pos() - body_pose.Pos());
      vehicle.rotors[count].position = toEigen(offset);
      vehicle.rotors[count].direction =
        sdfParam<std::string>(rotor, "direction", "ccw") == "cw" ? -1 : 1;
      rotor_joints_[count] = joint;
      ++count;
    }
    if (count != flight::kNumRotors) {
      RCLCPP_ERROR(logger, "Expected %d <rotor> elements, got %d", flight::kNumRotors, count);
      return false;
    }
    return true;
  }

  void loadVehicleParams(const sdf::ElementPtr & sdf, flight::QuadrotorParams & p)
  {
    // Mass and inertia come from the model itself so they always match the URDF
    p.mass = 0.0;
    for (const auto & link : model_->GetLinks()) {
      p.mass += link->GetInertial()->Mass();
    }
    const ignition::math::Matrix3d moi = body_->GetInertial()->MOI();
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        p.inertia(r, c) = moi(r, c);
      }
    }
    p.thrust_coefficient = sdfParam(sdf, "thrust_coefficient", p.thrust_coefficient);
    p.moment_coefficient = sdfParam(sdf, "moment_coefficient", p.moment_coefficient);
    p.rotor_drag_coefficient = sdfParam(sdf, "rotor_drag_coefficient", p.rotor_drag_coefficient);
    p.rotor_radius = sdfParam(sdf, "rotor_radius", p.rotor_radius);
    p.max_rotor_speed = sdfParam(sdf, "max_rotor_speed", p.max_rotor_speed);
    p.motor_time_constant_up = sdfParam(sdf, "motor_time_constant_up", p.motor_time_constant_up);
    p.motor_time_constant_down =
      sdfParam(sdf, "motor_time_constant_down", p.motor_time_constant_down);
    p.air_density = sdfParam(sdf, "air_density", p.air_density);
    p.drag_area = toEigen(sdfParam(sdf, "drag_area", toIgn(p.drag_area)));
    p.angular_drag = sdfParam(sdf, "angular_drag", p.angular_drag);
    p.enable_ground_effect = sdfParam(sdf, "ground_effect", p.enable_ground_effect);
    p.ground_height = sdfParam(sdf, "ground_height", p.ground_height);
  }

  flight::FlightControllerParams loadControllerParams(const sdf::ElementPtr & sdf)
  {
    flight::FlightControllerParams c;
    c.max_horizontal_speed = sdfParam(sdf, "max_horizontal_speed", c.max_horizontal_speed);
    c.max_vertical_speed = sdfParam(sdf, "max_vertical_speed", c.max_vertical_speed);
    c.max_yaw_rate = sdfParam(sdf, "max_yaw_rate", c.max_yaw_rate);
    c.max_tilt = sdfParam(sdf, "max_tilt", c.max_tilt);
    c.idle_rotor_speed = sdfParam(sdf, "idle_rotor_speed", c.idle_rotor_speed);
    return c;
  }

  void setupRos()
  {
    cmd_sub_ = ros_node_->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        pending_cmd_ = *msg;
        has_pending_cmd_ = true;
      });
    arm_sub_ = ros_node_->create_subscription<std_msgs::msg::Bool>(
      "drone/arm", 10, [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        pending_arm_ = msg->data;
        has_pending_arm_ = true;
      });
    wind_sub_ = ros_node_->create_subscription<geometry_msgs::msg::Vector3>(
      "drone/set_wind", 10, [this](geometry_msgs::msg::Vector3::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        pending_wind_ = Eigen::Vector3d(msg->x, msg->y, msg->z);
        has_pending_wind_ = true;
      });

    odom_pub_ = ros_node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    joint_pub_ = ros_node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    wind_pub_ = ros_node_->create_publisher<geometry_msgs::msg::Vector3Stamped>("drone/wind", 10);
    status_pub_ = ros_node_->create_publisher<std_msgs::msg::String>(
      "drone/status", rclcpp::QoS(1).transient_local());
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(ros_node_);
    }
    publishStatus();
  }

  flight::RigidBodyState readState() const
  {
    const ignition::math::Pose3d pose = body_->WorldPose();
    flight::RigidBodyState s;
    s.position = toEigen(pose.Pos());
    s.orientation = Eigen::Quaterniond(
      pose.Rot().W(), pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z());
    s.velocity = toEigen(body_->WorldLinearVel());
    s.angular_velocity = toEigen(body_->RelativeAngularVel());
    return s;
  }

  void setArmed(bool armed, const flight::RigidBodyState & state)
  {
    if (armed == armed_) {
      return;
    }
    armed_ = armed;
    if (armed_) {
      controller_->reset(state);
      command_ = geometry_msgs::msg::Twist();
    }
    RCLCPP_INFO(ros_node_->get_logger(), armed_ ? "Motors ARMED" : "Motors DISARMED");
  }

  // Publishes the flight status when it changes
  void publishStatus()
  {
    const std::string status =
      !armed_ ? "DISARMED" : (controller_->landed() ? "LANDED" : "FLYING");
    if (status == last_status_) {
      return;
    }
    last_status_ = status;
    std_msgs::msg::String msg;
    msg.data = status;
    status_pub_->publish(msg);
  }

  void onUpdate(const gazebo::common::UpdateInfo & info)
  {
    const double now = info.simTime.Double();
    const double dt = now - last_update_time_;
    if (dt <= 0.0) {
      return;
    }
    last_update_time_ = now;

    const flight::RigidBodyState state = readState();
    consumeRosInputs(now, state);

    // Onboard autopilot -> rotor speed commands
    if (armed_) {
      if (now - last_cmd_time_ > command_timeout_) {
        command_ = geometry_msgs::msg::Twist();  // stale command: hold position
      }
      const Eigen::Vector3d v(command_.linear.x, command_.linear.y, command_.linear.z);
      flight::VelocityCommand cmd;
      cmd.velocity = command_in_world_frame_ ? v :
        flight::headingToWorld(v, flight::yawFromQuaternion(state.orientation));
      cmd.yaw_rate = command_.angular.z;
      plant_->setRotorSpeedCommands(controller_->update(state, cmd, dt));
    } else {
      plant_->setRotorSpeedCommands(flight::RotorArray{});
    }

    // Plant: motors, aerodynamics and wind -> wrench on the body
    plant_->updateMotors(dt);
    const Eigen::Vector3d wind = wind_->update(dt);
    const flight::Wrench wrench = plant_->computeWrench(state, wind);
    body_->AddLinkForce(toIgn(wrench.force));
    body_->AddRelativeTorque(toIgn(wrench.torque));

    // Spin the propeller visuals (slowed down so the rotation stays visible)
    const auto & speeds = plant_->rotorSpeeds();
    for (int i = 0; i < flight::kNumRotors; ++i) {
      const double dir = plant_->params().rotors[i].direction;
      rotor_joints_[i]->SetVelocity(0, dir * speeds[i] / rotor_visual_slowdown_);
    }

    publishStatus();
    publish(info.simTime, now, state, wind);
  }

  void consumeRosInputs(double now, const flight::RigidBodyState & state)
  {
    bool arm_request = false;
    bool has_arm = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (has_pending_cmd_) {
        command_ = pending_cmd_;
        last_cmd_time_ = now;
        has_pending_cmd_ = false;
      }
      if (has_pending_wind_) {
        wind_->setMean(pending_wind_);
        RCLCPP_INFO(
          ros_node_->get_logger(), "Mean wind set to (%.1f %.1f %.1f) m/s",
          pending_wind_.x(), pending_wind_.y(), pending_wind_.z());
        has_pending_wind_ = false;
      }
      has_arm = has_pending_arm_;
      arm_request = pending_arm_;
      has_pending_arm_ = false;
    }
    if (has_arm) {
      setArmed(arm_request, state);
    }
  }

  void publish(
    const gazebo::common::Time & stamp_time, double now,
    const flight::RigidBodyState & state, const Eigen::Vector3d & wind)
  {
    const auto stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(stamp_time);

    if (now - last_odom_time_ >= odom_period_ || last_odom_time_ < 0.0) {
      last_odom_time_ = now;
      nav_msgs::msg::Odometry odom;
      odom.header.stamp = stamp;
      odom.header.frame_id = odom_frame_;
      odom.child_frame_id = base_frame_;
      odom.pose.pose.position.x = state.position.x();
      odom.pose.pose.position.y = state.position.y();
      odom.pose.pose.position.z = state.position.z();
      odom.pose.pose.orientation.w = state.orientation.w();
      odom.pose.pose.orientation.x = state.orientation.x();
      odom.pose.pose.orientation.y = state.orientation.y();
      odom.pose.pose.orientation.z = state.orientation.z();
      // nav_msgs/Odometry twist is expressed in the child (body) frame
      const Eigen::Vector3d v_body = state.orientation.conjugate() * state.velocity;
      odom.twist.twist.linear.x = v_body.x();
      odom.twist.twist.linear.y = v_body.y();
      odom.twist.twist.linear.z = v_body.z();
      odom.twist.twist.angular.x = state.angular_velocity.x();
      odom.twist.twist.angular.y = state.angular_velocity.y();
      odom.twist.twist.angular.z = state.angular_velocity.z();
      odom_pub_->publish(odom);

      if (tf_broadcaster_) {
        geometry_msgs::msg::TransformStamped tf;
        tf.header = odom.header;
        tf.child_frame_id = base_frame_;
        tf.transform.translation.x = state.position.x();
        tf.transform.translation.y = state.position.y();
        tf.transform.translation.z = state.position.z();
        tf.transform.rotation = odom.pose.pose.orientation;
        tf_broadcaster_->sendTransform(tf);
      }

      geometry_msgs::msg::Vector3Stamped wind_msg;
      wind_msg.header.stamp = stamp;
      wind_msg.header.frame_id = odom_frame_;
      wind_msg.vector.x = wind.x();
      wind_msg.vector.y = wind.y();
      wind_msg.vector.z = wind.z();
      wind_pub_->publish(wind_msg);
    }

    if (now - last_joint_time_ >= joint_state_period_ || last_joint_time_ < 0.0) {
      last_joint_time_ = now;
      sensor_msgs::msg::JointState joints;
      joints.header.stamp = stamp;
      const auto & speeds = plant_->rotorSpeeds();
      for (int i = 0; i < flight::kNumRotors; ++i) {
        joints.name.push_back(rotor_joints_[i]->GetName());
        joints.position.push_back(rotor_joints_[i]->Position(0));
        // Physical rotor speed [rad/s], not the slowed-down visual one
        joints.velocity.push_back(plant_->params().rotors[i].direction * speeds[i]);
      }
      joint_pub_->publish(joints);
    }
  }

  // Gazebo
  gazebo::physics::ModelPtr model_;
  gazebo::physics::WorldPtr world_;
  gazebo::physics::LinkPtr body_;
  std::array<gazebo::physics::JointPtr, flight::kNumRotors> rotor_joints_;
  gazebo::event::ConnectionPtr update_connection_;

  // Flight model (simulation thread only)
  std::unique_ptr<flight::QuadrotorModel> plant_;
  std::unique_ptr<flight::FlightController> controller_;
  std::unique_ptr<flight::WindModel> wind_;
  geometry_msgs::msg::Twist command_;
  bool armed_ = false;
  bool command_in_world_frame_ = false;
  double command_timeout_ = 0.5;
  double rotor_visual_slowdown_ = 10.0;
  double last_update_time_ = 0.0;
  double last_cmd_time_ = -1.0;
  double last_odom_time_ = -1.0;
  double last_joint_time_ = -1.0;
  double odom_period_ = 0.01;
  double joint_state_period_ = 1.0 / 30.0;

  // ROS
  gazebo_ros::Node::SharedPtr ros_node_;
  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_ = true;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr wind_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr wind_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  std::string last_status_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Inputs written by ROS callbacks, consumed by the simulation thread
  std::mutex mutex_;
  geometry_msgs::msg::Twist pending_cmd_;
  bool has_pending_cmd_ = false;
  bool pending_arm_ = false;
  bool has_pending_arm_ = false;
  Eigen::Vector3d pending_wind_ = Eigen::Vector3d::Zero();
  bool has_pending_wind_ = false;
};

GZ_REGISTER_MODEL_PLUGIN(QuadrotorDynamicsPlugin)

}  // namespace interceptor
