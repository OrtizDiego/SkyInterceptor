/**
 * @file metrics_recorder_node.cpp
 * @brief Watches a FOLLOW run and reports the metrics of the phase's exit criteria:
 *        the minimum distance to every person, vehicle and obstacle, the framing
 *        error against the boresight and the retention of the target track.
 *
 * Topics
 * ------
 *   in  /odom             Odometry   the drone's own state
 *   in  /tracks           TargetStateArray  every track the tracker reports
 *   in  /target/state     TargetState the selected track
 *   in  /follow/target_truth PointStamped  the ground truth of the scenario's target
 *
 * Outputs
 * -------
 * A JSON summary (the numbers of the run and the verdict against the criteria), a CSV
 * with the time series and a human readable summary in the log.
 *
 * The distances are measured against the obstacles of safety_params.yaml, the same
 * list the safety filter keeps the drone out of, so the two never disagree about
 * where the trees are.
 */

#include <Eigen/Geometry>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <interceptor_interfaces/msg/mission_status.hpp>
#include <interceptor_interfaces/msg/target_state.hpp>
#include <interceptor_interfaces/msg/target_state_array.hpp>

#include "metrics/metrics_recorder.hpp"

namespace interceptor
{

using PointStampedMsg = geometry_msgs::msg::PointStamped;
using OdometryMsg = nav_msgs::msg::Odometry;
using TargetStateMsg = interceptor_interfaces::msg::TargetState;
using TargetStateArrayMsg = interceptor_interfaces::msg::TargetStateArray;
using MissionStatusMsg = interceptor_interfaces::msg::MissionStatus;

class MetricsRecorderNode : public rclcpp::Node
{
public:
  explicit MetricsRecorderNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("metrics_recorder_node", options)
  {
    metrics::RecorderParams params;
    // The thresholds come from the safety parameters: the recorder has to measure
    // the distances the filter promises, not a copy of them.
    params.d_min_3d = declare_parameter("safety.d_min_3d", params.d_min_3d);
    params.d_min_horizontal = declare_parameter("safety.d_min_horizontal", params.d_min_horizontal);
    params.keepout_horizontal_altitude =
      declare_parameter("safety.keepout_horizontal_altitude", params.keepout_horizontal_altitude);
    params.keepout_horizontal_taper =
      declare_parameter("safety.keepout_horizontal_taper", params.keepout_horizontal_taper);
    params.d_obstacle_min = declare_parameter("safety.d_obstacle_min", params.d_obstacle_min);
    params.drone_radius = declare_parameter("safety.drone_radius", params.drone_radius);
    params.uncertainty_gain =
      declare_parameter("safety.uncertainty_gain", params.uncertainty_gain);
    // The static obstacles, the same list the safety filter keeps the drone out
    // of. It is declared here, so the value of the parameter file is picked up.
    declare_parameter("safety.obstacles", std::vector<double>{});

    const double rate_hz = declare_parameter("metrics.rate_hz", 20.0);
    if (!(rate_hz > 0.0)) {
      throw std::invalid_argument("metrics.rate_hz must be positive");
    }
    params.warmup_s = declare_parameter("metrics.warmup_s", params.warmup_s);
    params.framing_tolerance_deg =
      declare_parameter("metrics.framing_tolerance_deg", params.framing_tolerance_deg);
    params.min_framing_score =
      declare_parameter("metrics.min_framing_score", params.min_framing_score);
    params.framing_over_tracked_time_scenarios = declare_parameter(
      "metrics.framing_over_tracked_time_scenarios", std::vector<std::string>{});
    params.reacquire_loss_time_s =
      declare_parameter("metrics.reacquire_loss_time_s", params.reacquire_loss_time_s);
    params.scenario = declare_parameter("metrics.scenario", params.scenario);
    params.seed = declare_parameter("metrics.seed", params.seed);

    output_path_ = declare_parameter("metrics.output_path", std::string(""));
    csv_path_ = declare_parameter("metrics.csv_path", std::string(""));
    run_duration_s_ = declare_parameter("metrics.run_duration_s", 0.0);

    recorder_.setParams(params);
    recorder_.setObstacles(readObstacles());

    sub_odom_ = create_subscription<OdometryMsg>(
      "/odom", rclcpp::QoS(10),
      [this](OdometryMsg::ConstSharedPtr msg) {onOdom(*msg);});
    sub_tracks_ = create_subscription<TargetStateArrayMsg>(
      "/tracks", rclcpp::QoS(10),
      [this](TargetStateArrayMsg::ConstSharedPtr msg) {onTracks(*msg);});
    sub_target_ = create_subscription<TargetStateMsg>(
      "/target/state", rclcpp::QoS(10),
      [this](TargetStateMsg::ConstSharedPtr msg) {onTarget(*msg);});
    sub_status_ = create_subscription<MissionStatusMsg>(
      "/mission/status", rclcpp::QoS(10),
      [this](MissionStatusMsg::ConstSharedPtr msg) {onStatus(*msg);});
    sub_truth_ = create_subscription<PointStampedMsg>(
      "/follow/target_truth", rclcpp::QoS(10),
      [this](PointStampedMsg::ConstSharedPtr msg) {onTruth(*msg);});

    // On the node clock, so the samples follow the simulation time.
    timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::duration<double>(1.0 / rate_hz),
      [this]() {onTimer();});

    RCLCPP_INFO(
      get_logger(),
      "Metrics recorder: scenario '%s', d_min_3d %.1f m, d_min_horizontal %.1f m, "
      "d_obstacle_min %.1f m, %zu obstacles, warm-up %.1f s, %.0f Hz",
      params.scenario.c_str(), params.d_min_3d, params.d_min_horizontal, params.d_obstacle_min,
      recorder_.obstacles().size(), params.warmup_s, rate_hz);
  }

  /// Writes the summary when the node goes down, so a run that is stopped early
  /// still leaves its numbers behind.
  ~MetricsRecorderNode() override
  {
    write();
  }

private:
  /// The static obstacles of safety_params.yaml: a flat list of x, y, radius, height.
  std::vector<metrics::Obstacle> readObstacles() const
  {
    std::vector<metrics::Obstacle> obstacles;
    // The parameter is declared, so the value of the file is picked up: a value
    // that comes from a parameter file and was never declared stays invisible.
    const auto values = get_parameter("safety.obstacles").as_double_array();
    if (values.empty()) {
      RCLCPP_ERROR(
        get_logger(),
        "The obstacle list is empty: load config/safety_params.yaml with this "
        "node, or the distances to the obstacles cannot be measured");
      return obstacles;
    }
    for (std::size_t i = 0; i + 3 < values.size(); i += 4) {
      metrics::Obstacle obstacle;
      obstacle.x = values[i];
      obstacle.y = values[i + 1];
      obstacle.radius = values[i + 2];
      obstacle.height = values[i + 3];
      obstacle.name = "obstacle_" + std::to_string(obstacles.size());
      obstacles.push_back(obstacle);
    }
    return obstacles;
  }

  void onOdom(const OdometryMsg & msg)
  {
    const auto & q = msg.pose.pose.orientation;
    const Eigen::Quaterniond orientation(q.w, q.x, q.y, q.z);
    odom_position_ = Eigen::Vector3d(
      msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
    odom_yaw_ = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    // The boresight is the +x axis of the body: the body frame is FLU and the
    // stereo camera hangs level on the nose of the airframe.
    boresight_ = orientation.normalized() * Eigen::Vector3d(1.0, 0.0, 0.0);
    has_odom_ = true;
  }

  void onTracks(const TargetStateArrayMsg & msg)
  {
    tracks_.clear();
    tracks_.reserve(msg.tracks.size());
    for (const TargetStateMsg & track : msg.tracks) {
      metrics::TrackSample sample;
      sample.track_id = track.track_id;
      sample.class_id = track.class_id;
      sample.position =
        Eigen::Vector3d(track.position.x, track.position.y, track.position.z);
      const double pxx = track.position_covariance[0];
      const double pyy = track.position_covariance[4];
      sample.position_sigma = std::sqrt(std::max(0.0, 0.5 * (pxx + pyy)));
      sample.confirmed = track.confirmed;
      sample.is_valid = track.is_valid;
      tracks_.push_back(sample);
    }
  }

  void onTarget(const TargetStateMsg & msg)
  {
    selected_track_id_ = msg.track_id;
    selected_position_ = Eigen::Vector3d(msg.position.x, msg.position.y, msg.position.z);
    selected_confirmed_ = msg.confirmed;
    selected_valid_ = msg.is_valid;
    has_target_ = true;
  }

  void onStatus(const MissionStatusMsg & msg)
  {
    safety_active_ = msg.safety_active;
    reason_ = msg.reason;
  }

  void onTruth(const PointStampedMsg & msg)
  {
    target_truth_ = Eigen::Vector3d(msg.point.x, msg.point.y, msg.point.z);
    has_truth_ = true;
  }

  void onTimer()
  {
    if (!has_odom_ || !has_target_) {
      return;
    }
    metrics::Sample sample;
    sample.time_s = now().seconds();
    if (!has_start_time_) {
      has_start_time_ = true;
      start_time_ = sample.time_s;
    }
    sample.drone_position = odom_position_;
    sample.drone_yaw = odom_yaw_;
    sample.boresight = boresight_;
    sample.selected_track_id = selected_track_id_;
    sample.selected_position = selected_position_;
    sample.selected_usable = selected_confirmed_ && selected_valid_;
    sample.tracks = tracks_;
    sample.has_target_truth = has_truth_;
    sample.target_truth = target_truth_;
    sample.safety_active = safety_active_;
    sample.reason = reason_;
    recorder_.addSample(sample);

    if (run_duration_s_ > 0.0 && sample.time_s - start_time_ >= run_duration_s_) {
      write();
      // The run is over: the launch ends with the node.
      rclcpp::shutdown();
    }
  }

  /// Writes the summary and the time series, and stops the node.
  void write()
  {
    if (written_) {
      return;
    }
    written_ = true;
    const metrics::Summary summary = recorder_.summarize();
    const std::string text =
      metrics::MetricsRecorder::toText(summary, recorder_.params());
    RCLCPP_INFO(get_logger(), "\n%s", text.c_str());

    if (!output_path_.empty()) {
      const std::filesystem::path path(output_path_);
      if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path());
      }
      std::ofstream out(output_path_);
      if (out) {
        out << metrics::MetricsRecorder::toJson(summary, recorder_.params());
        RCLCPP_INFO(get_logger(), "Summary written to %s", output_path_.c_str());
      } else {
        RCLCPP_ERROR(get_logger(), "Cannot write %s", output_path_.c_str());
      }
    }
    if (!csv_path_.empty()) {
      const std::filesystem::path path(csv_path_);
      if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path());
      }
      std::ofstream out(csv_path_);
      if (out) {
        out << metrics::MetricsRecorder::toCsv(recorder_.timeSeries());
        RCLCPP_INFO(get_logger(), "Time series written to %s", csv_path_.c_str());
      } else {
        RCLCPP_ERROR(get_logger(), "Cannot write %s", csv_path_.c_str());
      }
    }
  }

  metrics::MetricsRecorder recorder_;

  Eigen::Vector3d odom_position_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d boresight_ = Eigen::Vector3d(1.0, 0.0, 0.0);
  double odom_yaw_ = 0.0;
  bool has_odom_ = false;

  std::vector<metrics::TrackSample> tracks_;
  int selected_track_id_ = -1;
  Eigen::Vector3d selected_position_ = Eigen::Vector3d::Zero();
  bool selected_confirmed_ = false;
  bool selected_valid_ = false;
  bool has_target_ = false;

  Eigen::Vector3d target_truth_ = Eigen::Vector3d::Zero();
  bool has_truth_ = false;

  bool safety_active_ = false;
  std::string reason_;

  std::string output_path_;
  std::string csv_path_;
  double run_duration_s_ = 0.0;
  double start_time_ = 0.0;
  bool has_start_time_ = false;
  bool written_ = false;

  rclcpp::Subscription<OdometryMsg>::SharedPtr sub_odom_;
  rclcpp::Subscription<TargetStateArrayMsg>::SharedPtr sub_tracks_;
  rclcpp::Subscription<TargetStateMsg>::SharedPtr sub_target_;
  rclcpp::Subscription<PointStampedMsg>::SharedPtr sub_truth_;
  rclcpp::Subscription<MissionStatusMsg>::SharedPtr sub_status_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace interceptor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<interceptor::MetricsRecorderNode>();
  rclcpp::spin(node);
  // The node is destroyed here, which writes the summary of the run.
  node.reset();
  rclcpp::shutdown();
  return 0;
}
