/**
 * @file intercept_guidance_node.cpp
 * @brief INTERCEPT guidance: PN / augmented PN on the line of sight, with the
 *        terminal capture law that flies the last few metres.
 *
 * Consumes the tracker's target state (/target/state) and the drone odometry (/odom),
 * runs the phase machine of plan section 4.4
 *
 *     SEARCH -> PURSUE (PN or APN) -> CAPTURE -> DONE
 *
 * and publishes the resulting setpoint on /setpoint/raw, plus GuidanceCommand telemetry.
 *
 * The guidance law itself lives in the ROS-free library include/guidance/pn_guidance.hpp.
 * This node only does the plumbing: the phase logic, the setpoint, the telemetry and
 * the arming of the drone.
 *
 * The setpoint goes on /setpoint/raw only. safety_filter_node (plan 2.2) is between the
 * planner and the trajectory controller and has the final say, so it owns /setpoint/safe.
 * Publishing on both would put two publishers on the topic the controller reads, and the
 * controller would sometimes fly an unfiltered setpoint.
 *
 * The abort (plan 4.4) is owned by the safety filter too: it is the one that knows
 * about the bystanders and about the engagement gate. The filter brakes the drone
 * itself, and it says so on /mission/status (abort_active). This node follows that
 * signal and stops commanding a closure: the vehicle holds position and the phase
 * machine goes to ABORT.
 */

#include <Eigen/Geometry>

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/vector3.hpp>
#include <interceptor_interfaces/msg/flight_setpoint.hpp>
#include <interceptor_interfaces/msg/guidance_command.hpp>
#include <interceptor_interfaces/msg/mission_mode.hpp>
#include <interceptor_interfaces/msg/mission_status.hpp>
#include <interceptor_interfaces/msg/target_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "common/math_utils.hpp"
#include "common/mission.hpp"
#include "guidance/pn_guidance.hpp"

namespace interceptor
{

using FlightSetpointMsg = interceptor_interfaces::msg::FlightSetpoint;
using GuidanceCommandMsg = interceptor_interfaces::msg::GuidanceCommand;
using MissionModeMsg = interceptor_interfaces::msg::MissionMode;
using MissionStatusMsg = interceptor_interfaces::msg::MissionStatus;
using TargetStateMsg = interceptor_interfaces::msg::TargetState;
using BoolMsg = std_msgs::msg::Bool;

// Closer than this the bearing rate is not used as a yaw feed-forward: at the
// capture the bearing swings wildly and its rate is unbounded.
constexpr double kMinYawFeedforwardRange = 3.0;  // m

class InterceptGuidanceNode : public rclcpp::Node
{
public:
  explicit InterceptGuidanceNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("intercept_guidance_node", options)
  {
    const guidance::GuidanceParams defaults;

    const std::string law_name = declare_parameter("guidance.law", "apn");
    const auto law = guidance::parseGuidanceLaw(law_name);
    if (!law) {
      throw std::invalid_argument("guidance.law must be 'pn' or 'apn', got '" + law_name + "'");
    }
    guidance::GuidanceParams guidance_params;
    guidance_params.law = *law;
    guidance_params.nav_constant =
      declare_parameter("guidance.nav_constant", defaults.nav_constant);
    guidance_params.max_accel = declare_parameter("guidance.max_accel", defaults.max_accel);
    guidance_params.target_accel_gain =
      declare_parameter("guidance.target_accel_gain", defaults.target_accel_gain);
    guidance_params.project_target_accel =
      declare_parameter("guidance.project_target_accel", defaults.project_target_accel);
    // The terminal capture law: the closure profile and the range the interceptor
    // settles at.
    guidance_params.capture.hold_radius =
      declare_parameter("guidance.hold_radius", defaults.capture.hold_radius);
    guidance_params.capture.approach_decel =
      declare_parameter("guidance.approach_decel", defaults.capture.approach_decel);
    guidance_params.capture.final_closing_speed =
      declare_parameter("guidance.final_closing_speed", defaults.capture.final_closing_speed);
    guidance_params.capture.max_closing_speed =
      declare_parameter("guidance.max_closing_speed", defaults.capture.max_closing_speed);
    guidance_params.capture.closing_speed_feedback =
      declare_parameter("guidance.closing_speed_feedback", defaults.capture.closing_speed_feedback);
    guidance_ = std::make_unique<guidance::PnGuidance>(guidance_params);

    capture_start_range_ = declare_parameter("intercept.capture_start_range", 10.0);
    capture_radius_ = declare_parameter("intercept.capture_radius", 1.5);
    capture_max_rel_speed_ = declare_parameter("intercept.capture_max_rel_speed", 3.0);
    // The range at which the target counts as having escaped the capture: well
    // above the capture start range, because the interceptor drifts in and out of
    // it while it closes.
    capture_escape_range_ = declare_parameter("intercept.capture_escape_range", 25.0);
    min_engage_altitude_agl_ = declare_parameter("intercept.min_engage_altitude_agl", 10.0);
    require_armed_ = declare_parameter("intercept.require_armed", true);

    // The hold radius has to be inside the capture radius: the range converges to
    // it, so aiming at the envelope boundary would approach the boundary
    // asymptotically and never enter the envelope.
    if (guidance_params.capture.hold_radius >= capture_radius_) {
      RCLCPP_WARN(
        get_logger(),
        "guidance.hold_radius (%.2f m) is not inside intercept.capture_radius (%.2f m): "
        "the capture envelope would never be entered", guidance_params.capture.hold_radius,
        capture_radius_);
    }

    // How the acceleration law becomes a velocity setpoint: the guidance acceleration
    // is scaled by the effective time constant of the velocity loop.
    accel_to_velocity_gain_ = declare_parameter("guidance.accel_to_velocity_gain", 0.2);
    max_speed_ = declare_parameter("guidance.max_speed", 12.0);
    max_vertical_speed_ = declare_parameter("guidance.max_vertical_speed", 3.0);
    // The drone has to take off before it can intercept: it climbs to this altitude
    // and holds it until the engagement is allowed.
    cruise_altitude_ = declare_parameter("guidance.cruise_altitude", 12.0);
    arm_on_start_ = declare_parameter("guidance.arm_on_start", true);

    const double rate_hz = declare_parameter("rate_hz", 50.0);

    pub_setpoint_raw_ = create_publisher<FlightSetpointMsg>("/setpoint/raw", 10);
    pub_command_ = create_publisher<GuidanceCommandMsg>("/guidance/command", 10);
    pub_arm_ = create_publisher<BoolMsg>("/drone/arm", 10);
    pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/guidance/vectors", 10);

    target_sub_ = create_subscription<TargetStateMsg>(
      "/target/state", 10,
      [this](TargetStateMsg::ConstSharedPtr msg) {onTarget(*msg);});
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {onOdom(*msg);});
    mode_sub_ = create_subscription<MissionModeMsg>(
      "/mission/mode", rclcpp::QoS(1).reliable().transient_local(),
      [this](MissionModeMsg::ConstSharedPtr msg) {onMode(*msg);});
    // The abort comes from the safety filter: it is the node that knows about the
    // bystanders and about the engagement gate.
    status_sub_ = create_subscription<MissionStatusMsg>(
      "/mission/status", 10,
      [this](MissionStatusMsg::ConstSharedPtr msg) {onStatus(*msg);});

    // On the node clock, so it follows simulation time
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / std::max(rate_hz, 1.0)),
      [this]() {onTimer();});

    RCLCPP_INFO(
      get_logger(),
      "PN guidance started: law=%s, N=%.1f, max_accel=%.1f m/s^2, capture_radius=%.1f m, "
      "hold_radius=%.2f m, approach_decel=%.1f m/s^2",
      guidance::toString(guidance_params.law).c_str(), guidance_params.nav_constant,
      guidance_params.max_accel, capture_radius_, guidance_params.capture.hold_radius,
      guidance_params.capture.approach_decel);
  }

private:
  // Phases of the engagement (plan 4.4)
  enum class Phase
  {
    SEARCH,   // no valid target: climb and hold
    PURSUE,   // PN / APN on the line of sight
    CAPTURE,  // inside the capture range: the capture is armed
    DONE,     // captured
    ABORT,    // the safety filter aborted the engagement: hold position
  };

  static constexpr const char * phaseName(Phase phase)
  {
    switch (phase) {
      case Phase::SEARCH: return "SEARCH";
      case Phase::PURSUE: return "PURSUE";
      case Phase::CAPTURE: return "CAPTURE";
      case Phase::DONE: return "DONE";
      case Phase::ABORT: return "ABORT";
    }
    return "UNKNOWN";
  }

  void onTarget(const TargetStateMsg & msg)
  {
    target_valid_ = msg.is_valid;
    target_confirmed_ = msg.confirmed;
    target_class_ = msg.class_id;
    track_id_ = msg.track_id;
    target_position_ = Eigen::Vector3d(msg.position.x, msg.position.y, msg.position.z);
    target_velocity_ = Eigen::Vector3d(msg.velocity.x, msg.velocity.y, msg.velocity.z);
    target_acceleration_ =
      Eigen::Vector3d(msg.acceleration.x, msg.acceleration.y, msg.acceleration.z);
    has_target_ = true;
  }

  void onOdom(const nav_msgs::msg::Odometry & msg)
  {
    const auto & q = msg.pose.pose.orientation;
    const Eigen::Quaterniond orientation(q.w, q.x, q.y, q.z);
    // nav_msgs/Odometry twist is expressed in the child (body) frame
    const auto & v = msg.twist.twist.linear;
    interceptor_velocity_ = orientation.normalized() * Eigen::Vector3d(v.x, v.y, v.z);
    interceptor_position_ = Eigen::Vector3d(
      msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
    odom_orientation_ = orientation.normalized();
    has_odom_ = true;
  }

  void onMode(const MissionModeMsg & msg)
  {
    if (has_mode_ && (msg.mode != mode_ || msg.armed != armed_)) {
      // A new mode or a new arming state restarts the engagement.
      phase_ = Phase::SEARCH;
      capture_confirmed_ = false;
      capture_reported_ = false;
      aborted_ = false;
    }
    has_mode_ = true;
    mode_ = msg.mode;
    armed_ = msg.armed;
  }

  /// The safety filter's status: the abort comes from there.
  ///
  /// The abort is latched here as well, so a single dropped message does not
  /// resume the closure.
  void onStatus(const MissionStatusMsg & msg)
  {
    if (msg.abort_active && !aborted_) {
      aborted_ = true;
      phase_ = Phase::ABORT;
      RCLCPP_WARN(
        get_logger(), "Engagement aborted by the safety filter: %s",
        msg.reason.c_str());
    }
  }

  /// The engagement gate of plan 4.4: the target must be a uav, confirmed, high
  /// enough, and the operator must have armed the system.
  bool engagementAllowed() const
  {
    if (!has_target_ || !target_valid_) {
      return false;
    }
    if (target_class_ != static_cast<int32_t>(TargetClass::UAV)) {
      return false;
    }
    if (!target_confirmed_) {
      return false;
    }
    if (target_position_.z() < min_engage_altitude_agl_) {
      return false;
    }
    if (require_armed_ && !armed_) {
      return false;
    }
    return true;
  }

  void onTimer()
  {
    if (!has_odom_) {
      return;  // nothing to fly yet
    }

    // Arm the drone once, after the first odometry.
    if (arm_on_start_ && !arm_sent_) {
      arm_sent_ = true;
      BoolMsg msg;
      msg.data = true;
      pub_arm_->publish(msg);
      RCLCPP_INFO(get_logger(), "Arming the drone");
    }

    updatePhase();
    publishSetpoint();
    publishTelemetry();
    publishMarkers();
  }

  void updatePhase()
  {
    switch (phase_) {
      case Phase::SEARCH:
        if (engagementAllowed()) {
          phase_ = Phase::PURSUE;
          RCLCPP_INFO(
            get_logger(), "Engagement started: target confirmed, range %.1f m",
            (target_position_ - interceptor_position_).norm());
        }
        break;

      case Phase::PURSUE: {
          if (!engagementAllowed()) {
            phase_ = Phase::SEARCH;
            RCLCPP_WARN(get_logger(), "Engagement aborted: target lost or gate closed");
            break;
          }
          const double range = (target_position_ - interceptor_position_).norm();
          if (range < capture_start_range_) {
            phase_ = Phase::CAPTURE;
            RCLCPP_INFO(get_logger(), "Capture phase at %.1f m", range);
          }
          break;
        }

      case Phase::CAPTURE: {
          if (!engagementAllowed()) {
            phase_ = Phase::SEARCH;
            RCLCPP_WARN(get_logger(), "Capture aborted: target lost or gate closed");
            break;
          }
          const double range = (target_position_ - interceptor_position_).norm();
          const double relative_speed = (target_velocity_ - interceptor_velocity_).norm();
          if (range <= capture_radius_ && relative_speed <= capture_max_rel_speed_) {
            phase_ = Phase::DONE;
            capture_confirmed_ = true;
            RCLCPP_INFO(
              get_logger(), "Target captured at %.2f m, relative speed %.2f m/s", range,
              relative_speed);
          } else if (range <= capture_radius_ && !capture_reported_) {
            // Inside the capture radius but too fast: the interceptor is about to
            // overfly the target. Report it once so the operator can see how close
            // the engagement got, then keep flying the guidance law.
            capture_reported_ = true;
            RCLCPP_WARN(
              get_logger(), "Inside the capture radius (%.2f m) but the relative speed "
              "is %.2f m/s (limit %.2f): overflying the target",
              range, relative_speed, capture_max_rel_speed_);
          } else if (range > capture_escape_range_) {
            // The target pulled away for good: go back to the pursuit law. The
            // threshold is well above the capture start range, because the
            // interceptor naturally drifts in and out of it while it closes.
            phase_ = Phase::PURSUE;
            RCLCPP_INFO(get_logger(), "Target escaped, back to PURSUE");
          }
          break;
        }

      case Phase::DONE:
        // Hold the position until the operator resets the mission.
        break;

      case Phase::ABORT:
        // The safety filter aborted the engagement: hold position. The filter
        // brakes the drone anyway, but the guidance must not keep commanding a
        // closure.
        break;
    }
  }

  /// The velocity setpoint for the current phase.
  Eigen::Vector3d computeVelocity()
  {
    if (aborted_) {
      // The engagement was aborted: hold position.
      return Eigen::Vector3d::Zero();
    }
    switch (phase_) {
      case Phase::SEARCH: {
          // Climb to the cruise altitude and hold it.
          const double altitude_error = cruise_altitude_ - interceptor_position_.z();
          Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
          velocity.z() =
            std::clamp(altitude_error * 0.5, -max_vertical_speed_, max_vertical_speed_);
          return velocity;
        }

      case Phase::PURSUE:
      case Phase::CAPTURE:
      case Phase::DONE: {
          // The same law in all three phases: the terminal capture law. Its closure
          // profile slows the interceptor down as the range closes, so the capture
          // is flown rather than detected, and in DONE it holds the offset inside
          // the capture envelope.
          //
          // The phases differ in what they mean, not in how the vehicle is flown:
          // PURSUE is the approach, CAPTURE is the range at which the capture is
          // armed, and DONE is the captured state.
          guidance::EngagementState state;
          state.interceptor_position = interceptor_position_;
          state.interceptor_velocity = interceptor_velocity_;
          state.target_position = target_position_;
          state.target_velocity = target_velocity_;
          state.target_acceleration = target_acceleration_;

          const guidance::GuidanceResult result = guidance_->update(state);
          last_result_ = result;
          return clampSpeed(
            guidance_->velocitySetpoint(state, result, accel_to_velocity_gain_));
        }

      case Phase::ABORT:
        // The safety filter aborted the engagement: hold position.
        return Eigen::Vector3d::Zero();
    }
    return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d clampSpeed(const Eigen::Vector3d & velocity) const
  {
    Eigen::Vector3d out = velocity;
    const double horizontal = out.head<2>().norm();
    if (horizontal > max_speed_) {
      out.head<2>() *= max_speed_ / horizontal;
    }
    out.z() = std::clamp(out.z(), -max_vertical_speed_, max_vertical_speed_);
    return out;
  }

  void publishSetpoint()
  {
    const Eigen::Vector3d velocity = computeVelocity();

    FlightSetpointMsg msg;
    msg.header.stamp = now();
    msg.header.frame_id = "map";
    msg.position_valid = false;  // velocity tracking: the controller has no position goal
    msg.velocity.x = velocity.x();
    msg.velocity.y = velocity.y();
    msg.velocity.z = velocity.z();
    // The guidance acceleration is a feed-forward for the velocity loop.
    msg.acceleration_ff.x = last_result_.acceleration.x();
    msg.acceleration_ff.y = last_result_.acceleration.y();
    msg.acceleration_ff.z = last_result_.acceleration.z();
    msg.yaw = targetYaw();
    msg.yaw_rate = targetYawRate();
    msg.source = FlightSetpointMsg::SOURCE_INTERCEPT;

    // Only the raw setpoint: safety_filter_node owns /setpoint/safe.
    pub_setpoint_raw_->publish(msg);
  }

  /// The heading that points the interceptor's nose at the target.
  ///
  /// The stereo camera hangs off the front of the airframe, so the nose has to
  /// follow the target for the cameras to see it: without this the drone flies
  /// the guidance law side-on and the camera stares into the sky. The heading is
  /// the bearing of the target in the world frame, which is the same convention
  /// the trajectory controller's yaw loop expects.
  ///
  /// Before the engagement there is no target yet, so the drone holds the
  /// heading it was spawned with rather than swinging to an arbitrary one.
  double targetYaw() const
  {
    if (!has_target_ || !target_valid_) {
      return currentYaw();
    }
    const Eigen::Vector3d offset = target_position_ - interceptor_position_;
    if (offset.head<2>().norm() < 1e-3) {
      return currentYaw();
    }
    return std::atan2(offset.y(), offset.x());
  }

  /// The rate the bearing to the target turns: the yaw-rate feed-forward of the
  /// heading in targetYaw(), so the nose keeps up with a target that crosses the
  /// line of sight without a heading gain high enough to make the yaw ring. Zero
  /// within kMinYawFeedforwardRange, where the bearing swings wildly at the capture.
  double targetYawRate() const
  {
    if (!has_target_ || !target_valid_ || !has_odom_) {
      return 0.0;
    }
    return bearingRate(
      interceptor_position_, interceptor_velocity_, target_position_, target_velocity_,
      kMinYawFeedforwardRange);
  }

  /// The interceptor's current heading, from its odometry.
  double currentYaw() const
  {
    if (!has_odom_) {
      return 0.0;
    }
    const auto & q = odom_orientation_;
    return std::atan2(
      2.0 * (q.w() * q.z() + q.x() * q.y()),
      1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
  }

  void publishTelemetry()
  {
    GuidanceCommandMsg msg;
    msg.header.stamp = now();
    msg.header.frame_id = "map";
    msg.track_id = track_id_;
    msg.acceleration_cmd.x = last_result_.acceleration.x();
    msg.acceleration_cmd.y = last_result_.acceleration.y();
    msg.acceleration_cmd.z = last_result_.acceleration.z();
    msg.navigation_constant = guidance_->params().nav_constant;
    msg.closing_velocity = last_result_.los.closing_velocity;
    msg.time_to_go = last_result_.los.time_to_go;
    msg.miss_distance = last_result_.zero_effort_miss;
    if (phase_ == Phase::CAPTURE || phase_ == Phase::DONE) {
      // The terminal law is running: the closure is governed by the capture radius.
      msg.mode = GuidanceCommandMsg::MODE_TERMINAL;
    } else {
      msg.mode = guidance_->params().law == guidance::GuidanceLaw::PURE_PN ?
        GuidanceCommandMsg::MODE_PN : GuidanceCommandMsg::MODE_APN;
    }
    msg.is_feasible = engagementAllowed() && !aborted_;
    msg.status_message = phaseName(phase_);

    pub_command_->publish(msg);
  }

  /// Draws the line of sight and the commanded acceleration in RViz.
  void publishMarkers()
  {
    if (phase_ == Phase::SEARCH) {
      return;
    }

    visualization_msgs::msg::MarkerArray array;

    // The line of sight from the interceptor to the target.
    visualization_msgs::msg::Marker los;
    los.header.stamp = now();
    los.header.frame_id = "map";
    los.ns = "los";
    los.id = 0;
    los.type = visualization_msgs::msg::Marker::LINE_STRIP;
    los.action = visualization_msgs::msg::Marker::ADD;
    los.pose.orientation.w = 1.0;
    los.scale.x = 0.08;
    los.color.r = 0.2f;
    los.color.g = 0.6f;
    los.color.b = 1.0f;
    los.color.a = 0.9f;
    los.points.push_back(point(interceptor_position_));
    los.points.push_back(point(target_position_));
    array.markers.push_back(los);

    // The commanded acceleration, drawn from the interceptor.
    visualization_msgs::msg::Marker accel;
    accel.header.stamp = now();
    accel.header.frame_id = "map";
    accel.ns = "acceleration";
    accel.id = 0;
    accel.type = visualization_msgs::msg::Marker::ARROW;
    accel.action = visualization_msgs::msg::Marker::ADD;
    accel.pose.position = point(interceptor_position_);
    accel.pose.orientation.w = 1.0;
    accel.scale.x = 0.12;
    accel.scale.y = 0.25;
    accel.scale.z = 0.25;
    accel.color.r = 1.0f;
    accel.color.g = 0.4f;
    accel.color.b = 0.1f;
    accel.color.a = 1.0f;
    // The arrow points along the acceleration: its default direction is +x.
    const Eigen::Vector3d direction = last_result_.acceleration.normalized();
    const Eigen::Vector3d reference = std::abs(direction.x()) < 0.9 ?
      Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
    const Eigen::Vector3d axis = reference.cross(direction).normalized();
    const double angle = std::acos(std::clamp(reference.dot(direction), -1.0, 1.0));
    const Eigen::Quaterniond orientation(Eigen::AngleAxisd(angle, axis));
    accel.pose.orientation.x = orientation.x();
    accel.pose.orientation.y = orientation.y();
    accel.pose.orientation.z = orientation.z();
    accel.pose.orientation.w = orientation.w();
    array.markers.push_back(accel);

    pub_markers_->publish(array);
  }

  static geometry_msgs::msg::Point point(const Eigen::Vector3d & v)
  {
    geometry_msgs::msg::Point p;
    p.x = v.x();
    p.y = v.y();
    p.z = v.z();
    return p;
  }

  // Guidance
  std::unique_ptr<guidance::PnGuidance> guidance_;
  guidance::GuidanceResult last_result_;

  // Phases and gates
  Phase phase_ = Phase::SEARCH;
  bool capture_confirmed_ = false;
  bool capture_reported_ = false;
  bool target_valid_ = false;
  bool target_confirmed_ = false;
  int32_t target_class_ = -1;
  int32_t track_id_ = -1;
  bool has_target_ = false;
  bool has_odom_ = false;
  bool has_mode_ = false;
  bool armed_ = false;
  uint8_t mode_ = 0;
  bool arm_sent_ = false;
  /// The safety filter aborted the engagement.
  bool aborted_ = false;

  // Target and interceptor state (world frame)
  Eigen::Vector3d target_position_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d target_velocity_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d target_acceleration_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d interceptor_position_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d interceptor_velocity_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond odom_orientation_ = Eigen::Quaterniond::Identity();

  // Parameters
  double capture_start_range_ = 10.0;
  double capture_radius_ = 1.5;
  double capture_max_rel_speed_ = 3.0;
  double capture_escape_range_ = 25.0;
  double min_engage_altitude_agl_ = 10.0;
  bool require_armed_ = true;
  double accel_to_velocity_gain_ = 0.2;
  double max_speed_ = 12.0;
  double max_vertical_speed_ = 3.0;
  double cruise_altitude_ = 12.0;
  bool arm_on_start_ = true;

  rclcpp::Publisher<FlightSetpointMsg>::SharedPtr pub_setpoint_raw_;
  rclcpp::Publisher<GuidanceCommandMsg>::SharedPtr pub_command_;
  rclcpp::Publisher<BoolMsg>::SharedPtr pub_arm_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Subscription<TargetStateMsg>::SharedPtr target_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<MissionModeMsg>::SharedPtr mode_sub_;
  rclcpp::Subscription<MissionStatusMsg>::SharedPtr status_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace interceptor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<interceptor::InterceptGuidanceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
