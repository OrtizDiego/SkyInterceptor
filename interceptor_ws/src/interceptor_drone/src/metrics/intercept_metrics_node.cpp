/**
 * @file intercept_metrics_node.cpp
 * @brief Records the metrics of one INTERCEPT engagement (plan section 5).
 *
 * The node watches the live topics of one engagement and writes a row of CSV
 * per run: the capture, the envelope, the abort and the class of the engaged
 * track. The scoring itself is in the ROS-free library
 * (include/metrics/intercept_metrics.hpp), so it can be unit tested on
 * synthetic records; this node only turns the topics into samples.
 *
 * The geometry comes from the same two sources the guidance uses: the
 * tracker's target state (/target/state) and the drone's odometry (/odom).
 * The range and the relative speed are therefore the ones the guidance's own
 * capture condition is evaluated on.
 *
 * The abort comes from the safety filter (/mission/status), and the phase from
 * the guidance (/guidance/command).
 *
 * The node writes the row when the engagement ends (the guidance reports a
 * terminal phase) or when the time limit is reached, and it appends to the
 * file the batch runner reads.
 */

#include <Eigen/Geometry>

#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <interceptor_interfaces/msg/guidance_command.hpp>
#include <interceptor_interfaces/msg/mission_status.hpp>
#include <interceptor_interfaces/msg/target_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "metrics/intercept_metrics.hpp"

namespace interceptor
{

using GuidanceCommandMsg = interceptor_interfaces::msg::GuidanceCommand;
using MissionStatusMsg = interceptor_interfaces::msg::MissionStatus;
using TargetStateMsg = interceptor_interfaces::msg::TargetState;
using OdometryMsg = nav_msgs::msg::Odometry;

class InterceptMetricsNode : public rclcpp::Node
{
public:
  explicit InterceptMetricsNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("intercept_metrics_node", options)
  {
    // The capture thresholds are the guidance's own (config/intercept_params.yaml),
    // so the scorer and the guidance can never disagree on what a capture is.
    metrics::MetricsParams params;
    params.capture_radius_m =
      declare_parameter("intercept.capture_radius", params.capture_radius_m);
    params.capture_max_rel_speed_mps =
      declare_parameter("intercept.capture_max_rel_speed", params.capture_max_rel_speed_mps);
    scorer_ = std::make_unique<metrics::EngagementScorer>(params);

    scenario_ = declare_parameter("scenario", std::string("unknown"));
    seed_ = declare_parameter("seed", 0);
    output_path_ = declare_parameter("output_path", std::string("intercept_metrics.csv"));
    // How long the engagement may take before the node gives up on it [s]. It is
    // counted from the first odometry: the simulator can take a minute of its
    // own time to spawn the drone, and that is not part of the engagement.
    timeout_s_ = declare_parameter("timeout", 120.0);

    sub_target_ = create_subscription<TargetStateMsg>(
      "/target/state", 10,
      [this](TargetStateMsg::ConstSharedPtr msg) {
        target_ = *msg;
        have_target_ = true;
      });
    sub_odom_ = create_subscription<OdometryMsg>(
      "/odom", 10,
      [this](OdometryMsg::ConstSharedPtr msg) {
        odom_ = *msg;
        have_odom_ = true;
      });
    sub_guidance_ = create_subscription<GuidanceCommandMsg>(
      "/guidance/command", 10,
      [this](GuidanceCommandMsg::ConstSharedPtr msg) {guidance_ = *msg;});
    sub_status_ = create_subscription<MissionStatusMsg>(
      "/mission/status", 10,
      [this](MissionStatusMsg::ConstSharedPtr msg) {status_ = *msg;});

    // On the node clock, so the samples follow the simulation time.
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(0.05),
      [this]() {onTimer();});

    RCLCPP_INFO(
      get_logger(),
      "Intercept metrics: scenario '%s', seed %d, capture radius %.2f m, "
      "max relative speed %.2f m/s, output %s",
      scenario_.c_str(), seed_, params.capture_radius_m,
      params.capture_max_rel_speed_mps, output_path_.c_str());
  }

private:
  void onTimer()
  {
    if (!have_odom_) {
      return;  // the drone is not in the world yet
    }
    const double now_s = now().seconds();
    if (start_time_ < 0.0) {
      start_time_ = now_s;
    }
    const double elapsed = now_s - start_time_;

    if (have_target_) {
      sample(elapsed);
    }

    // The engagement ends when the guidance reaches a terminal phase, or when
    // the time limit is reached.
    const bool terminal = guidance_.status_message == "DONE" ||
      guidance_.status_message == "ABORT";
    if (terminal || elapsed > timeout_s_) {
      writeRow();
      rclcpp::shutdown();
    }
  }

  /// One sample of the engagement, from the tracker's state and the odometry.
  void sample(double elapsed)
  {
    const Eigen::Vector3d target_position(
      target_.position.x, target_.position.y, target_.position.z);
    const Eigen::Vector3d target_velocity(
      target_.velocity.x, target_.velocity.y, target_.velocity.z);

    const Eigen::Vector3d interceptor_position(
      odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z);

    // The odometry twist is in the body frame, so it has to be rotated into
    // the world frame before it can be compared with the target's velocity.
    const auto & q = odom_.pose.pose.orientation;
    const Eigen::Quaterniond orientation(q.w, q.x, q.y, q.z);
    const Eigen::Vector3d body_velocity(
      odom_.twist.twist.linear.x, odom_.twist.twist.linear.y, odom_.twist.twist.linear.z);
    const Eigen::Vector3d interceptor_velocity = orientation.normalized() * body_velocity;

    metrics::EngagementSample sample;
    sample.time_s = elapsed;
    sample.range_m = (target_position - interceptor_position).norm();
    sample.relative_speed_mps = (target_velocity - interceptor_velocity).norm();
    sample.target_speed_mps = target_velocity.norm();
    sample.miss_distance_m = guidance_.miss_distance;
    sample.target_class_id = target_.class_id;
    sample.guidance_engaging = guidance_.status_message == "PURSUE" ||
      guidance_.status_message == "CAPTURE" || guidance_.status_message == "DONE";
    sample.abort_active = status_.abort_active;
    sample.abort_reason = status_.reason;
    scorer_->addSample(sample);
  }

  /// Writes the row of the run and appends it to the output file.
  void writeRow()
  {
    if (written_) {
      return;
    }
    written_ = true;

    metrics::RunRecord record;
    record.scenario = scenario_;
    record.seed = seed_;
    record.result = scorer_->result();
    record.result.end_phase = guidance_.status_message;

    std::ofstream file(output_path_, std::ios::app);
    if (!file.is_open()) {
      RCLCPP_ERROR(get_logger(), "Cannot write %s", output_path_.c_str());
      return;
    }
    // The header goes on top of an empty file, so the file is readable on its
    // own as well as appended to.
    if (file.tellp() == 0) {
      file << metrics::csvHeader() << '\n';
    }
    file << metrics::csvRow(record) << '\n';
    file.flush();

    const metrics::EngagementResult & result = record.result;
    RCLCPP_INFO(
      get_logger(),
      "Run '%s' seed %d: capture %s (range %.2f m, relative speed %.2f m/s), "
      "envelope %s, closest approach %.2f m, abort %s%s, non-uav engagements %d, "
      "phase %s, %.1f s",
      scenario_.c_str(), seed_, result.capture_confirmed ? "yes" : "no",
      result.capture_range_m, result.capture_relative_speed_mps,
      result.envelope_reached ? "yes" : "no", result.closest_approach_m,
      result.abort_fired ? "yes" : "no",
      result.abort_reason.empty() ? "" : (" (" + result.abort_reason + ")").c_str(),
      result.non_uav_engagements, result.end_phase.c_str(), result.duration_s);
  }

  std::unique_ptr<metrics::EngagementScorer> scorer_;
  TargetStateMsg target_;
  OdometryMsg odom_;
  GuidanceCommandMsg guidance_;
  MissionStatusMsg status_;
  bool have_target_ = false;
  bool have_odom_ = false;
  double start_time_ = -1.0;
  bool written_ = false;
  std::string scenario_;
  int seed_ = 0;
  std::string output_path_;
  double timeout_s_ = 120.0;

  rclcpp::Subscription<TargetStateMsg>::SharedPtr sub_target_;
  rclcpp::Subscription<OdometryMsg>::SharedPtr sub_odom_;
  rclcpp::Subscription<GuidanceCommandMsg>::SharedPtr sub_guidance_;
  rclcpp::Subscription<MissionStatusMsg>::SharedPtr sub_status_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace interceptor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<interceptor::InterceptMetricsNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
