/**
 * @file metrics_recorder.cpp
 * @brief The metrics of a FOLLOW run: the minimum distances, the framing error and
 *        the retention of the target track. Plain C++ and Eigen, no ROS.
 */

#include "metrics/metrics_recorder.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

#include "safety/safety_filter.hpp"

namespace interceptor
{
namespace metrics
{

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegPerRad = 180.0 / kPi;

/// The ground classes: the ones the keep-out barrier protects. Same rule as the
/// safety filter's own isGroundClass: person, car, truck and bicycle.
bool isGroundClass(int class_id)
{
  return class_id >= 0 && class_id <= 3;
}

/// Wraps an angle into (-pi, pi].
double wrapAngle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle <= -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

/// The percentile of a sorted vector, with the linear interpolation between the two
/// neighbours. The vector is sorted in place.
double percentile(std::vector<double> & values, double fraction)
{
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  if (values.size() == 1) {
    return values.front();
  }
  const double position = fraction * static_cast<double>(values.size() - 1);
  const auto lower = static_cast<size_t>(std::floor(position));
  const auto upper = static_cast<size_t>(std::ceil(position));
  const double weight = position - static_cast<double>(lower);
  return values[lower] * (1.0 - weight) + values[upper] * weight;
}

std::string jsonNumber(double value)
{
  if (!std::isfinite(value)) {
    return "null";
  }
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(6) << value;
  std::string text = stream.str();
  // Trim the trailing zeros, but keep at least one decimal.
  const std::size_t dot = text.find('.');
  if (dot != std::string::npos) {
    std::size_t last = text.size();
    while (last > dot + 2 && text[last - 1] == '0') {
      --last;
    }
    text.erase(last);
  }
  return text;
}

std::string jsonString(const std::string & value)
{
  std::string out = "\"";
  for (const char c : value) {
    if (c == '"' || c == '\\') {
      out.push_back('\\');
    }
    out.push_back(c);
  }
  out.push_back('"');
  return out;
}

}  // namespace

MetricsRecorder::MetricsRecorder(const RecorderParams & params)
: params_(params)
{
}

double MetricsRecorder::horizontalDistance(
  const Eigen::Vector3d & a, const Eigen::Vector3d & b)
{
  return std::hypot(a.x() - b.x(), a.y() - b.y());
}

double MetricsRecorder::distance(const Eigen::Vector3d & a, const Eigen::Vector3d & b)
{
  return (a - b).norm();
}

double MetricsRecorder::horizontalStandoff(
  double vertical_separation, double position_sigma) const
{
  // The same formula as the safety filter's own horizontalStandoff: the plan's
  // flat standoff, given up with the height above the person. Above the relief
  // altitude the rule is not in force at all, and the 3D rule governs alone.
  const double relief =
    std::max(0.0, vertical_separation - params_.keepout_horizontal_altitude);
  const double nominal = params_.d_min_horizontal - relief * params_.keepout_horizontal_taper;
  if (nominal <= 0.0) {
    return 0.0;
  }
  return nominal + params_.uncertainty_gain * position_sigma;
}

double MetricsRecorder::nominalHorizontalStandoff(double vertical_separation) const
{
  return horizontalStandoff(vertical_separation, 0.0);
}

double MetricsRecorder::azimuthError(
  double yaw, const Eigen::Vector3d & from, const Eigen::Vector3d & to)
{
  const double dx = to.x() - from.x();
  const double dy = to.y() - from.y();
  if (std::hypot(dx, dy) < 1e-6) {
    // The drone is on top of the target: the bearing is meaningless.
    return 0.0;
  }
  const double bearing = std::atan2(dy, dx);
  return std::abs(wrapAngle(bearing - yaw));
}

double MetricsRecorder::coneAngle(
  const Eigen::Vector3d & boresight, const Eigen::Vector3d & from, const Eigen::Vector3d & to)
{
  const Eigen::Vector3d direction = to - from;
  const double length = direction.norm();
  if (length < 1e-6) {
    return 0.0;
  }
  const double boresight_length = boresight.norm();
  if (boresight_length < 1e-6) {
    return 0.0;
  }
  const double cosine = direction.dot(boresight) / (length * boresight_length);
  return std::acos(std::clamp(cosine, -1.0, 1.0));
}

void MetricsRecorder::addSample(const Sample & sample)
{
  const bool first = samples_.empty();
  const double run_time = first ? 0.0 : sample.time_s - samples_.front().time_s;
  const bool scored = run_time >= params_.warmup_s;

  // --- The distances of this sample ----------------------------------------
  double min_person_distance = std::numeric_limits<double>::infinity();
  for (const TrackSample & track : sample.tracks) {
    if (!isGroundClass(track.class_id)) {
      continue;
    }
    min_person_distance = std::min(
      min_person_distance, horizontalDistance(sample.drone_position, track.position));
  }
  // The 3D distance is the hard rule; the horizontal one is the plan's standoff.
  double min_person_distance_3d = std::numeric_limits<double>::infinity();
  for (const TrackSample & track : sample.tracks) {
    if (!isGroundClass(track.class_id)) {
      continue;
    }
    min_person_distance_3d = std::min(
      min_person_distance_3d, distance(sample.drone_position, track.position));
  }

  double min_obstacle_distance = std::numeric_limits<double>::infinity();
  std::string min_obstacle_name;
  for (const Obstacle & obstacle : obstacles_) {
    // The same rule as the filter's: a short obstacle is not a hazard for a drone
    // that flies above it.
    if (obstacle.height > 0.0 && sample.drone_position.z() > obstacle.height) {
      continue;
    }
    const double distance = horizontalDistance(
      sample.drone_position, Eigen::Vector3d(obstacle.x, obstacle.y, 0.0)) - obstacle.radius;
    if (distance < min_obstacle_distance) {
      min_obstacle_distance = distance;
      min_obstacle_name = obstacle.name;
    }
  }

  // --- The framing of this sample ------------------------------------------
  const bool has_selected = sample.selected_track_id >= 0 && sample.selected_usable;
  const double framing_error = has_selected ?
    azimuthError(sample.drone_yaw, sample.drone_position, sample.selected_position) : 0.0;
  const double cone = has_selected ?
    coneAngle(sample.boresight, sample.drone_position, sample.selected_position) : 0.0;
  const bool has_truth = sample.has_target_truth;
  const double framing_target_error = has_truth ?
    azimuthError(sample.drone_yaw, sample.drone_position, sample.target_truth) : 0.0;

  TimeSeriesPoint point;
  point.time_s = sample.time_s;
  point.drone_x = sample.drone_position.x();
  point.drone_y = sample.drone_position.y();
  point.drone_z = sample.drone_position.z();
  point.target_x = sample.target_truth.x();
  point.target_y = sample.target_truth.y();
  point.target_z = sample.target_truth.z();
  point.selected_track_id = sample.selected_track_id;
  point.track_count = static_cast<int>(sample.tracks.size());
  point.safety_active = sample.safety_active;
  point.reason = sample.reason;
  point.framing_error_deg = framing_error * kDegPerRad;
  point.cone_angle_deg = cone * kDegPerRad;
  point.framing_target_error_deg = framing_target_error * kDegPerRad;
  point.min_person_distance_horizontal =
    std::isfinite(min_person_distance) ? min_person_distance : 0.0;
  point.min_person_distance_3d =
    std::isfinite(min_person_distance_3d) ? min_person_distance_3d : 0.0;
  point.min_obstacle_distance = std::isfinite(min_obstacle_distance) ? min_obstacle_distance : 0.0;
  point.target_present = sample.selected_usable;
  point.scored = scored;
  series_.push_back(point);

  samples_.push_back(sample);
}

Summary MetricsRecorder::summarize() const
{
  Summary summary;
  summary.sample_count = samples_.size();
  if (samples_.empty()) {
    return summary;
  }

  const double t0 = samples_.front().time_s;
  const double t1 = samples_.back().time_s;
  summary.duration_s = t1 - t0;

  double min_person_distance = std::numeric_limits<double>::infinity();
  double min_person_distance_3d = std::numeric_limits<double>::infinity();
  double min_person_margin_horizontal = std::numeric_limits<double>::infinity();
  double min_person_margin_horizontal_effective = std::numeric_limits<double>::infinity();
  double min_person_margin_3d = std::numeric_limits<double>::infinity();
  double min_person_margin_3d_effective = std::numeric_limits<double>::infinity();
  int min_person_track_id = -1;
  double min_obstacle_distance = std::numeric_limits<double>::infinity();
  std::string min_obstacle_name;

  std::vector<double> framing_errors;
  std::vector<double> framing_errors_present;
  std::vector<double> framing_target_errors;
  double present_time = 0.0;
  double lost_time = 0.0;
  bool previous_present = false;
  bool has_previous = false;
  double loss_start = t0;

  for (size_t i = 0; i < samples_.size(); ++i) {
    const Sample & sample = samples_[i];
    const double run_time = sample.time_s - t0;
    const bool scored = run_time >= params_.warmup_s;
    const double dt = (i + 1 < samples_.size()) ?
      std::max(0.0, samples_[i + 1].time_s - sample.time_s) : 0.0;

    // The distances
    for (const TrackSample & track : sample.tracks) {
      if (!isGroundClass(track.class_id)) {
        continue;
      }
      const double horizontal = horizontalDistance(sample.drone_position, track.position);
      if (horizontal < min_person_distance) {
        min_person_distance = horizontal;
        min_person_track_id = track.track_id;
      }
      // The 3D rule: the hard minimum distance, at any altitude.
      const double range_3d = distance(sample.drone_position, track.position);
      min_person_distance_3d = std::min(min_person_distance_3d, range_3d);
      const double surface_3d = range_3d - params_.drone_radius;
      min_person_margin_3d = std::min(min_person_margin_3d, surface_3d - params_.d_min_3d);
      min_person_margin_3d_effective = std::min(
        min_person_margin_3d_effective,
        surface_3d - (params_.d_min_3d + params_.uncertainty_gain * track.position_sigma));
      // The horizontal rule: the plan's standoff, given up with the height.
      // The criterion is against the nominal standoff, like the 3D one; the
      // filter's uncertainty cushion is reported on its own.
      const double vertical = sample.drone_position.z() - track.position.z();
      const double standoff = nominalHorizontalStandoff(vertical);
      if (standoff > 0.0) {
        min_person_margin_horizontal = std::min(
          min_person_margin_horizontal, horizontal - params_.drone_radius - standoff);
        min_person_margin_horizontal_effective = std::min(
          min_person_margin_horizontal_effective,
          horizontal - params_.drone_radius - horizontalStandoff(vertical, track.position_sigma));
      }
    }
    for (const Obstacle & obstacle : obstacles_) {
      // The safety filter's own distance: the 3D distance to the obstacle's
      // cylinder, beside it, above it or over its rim (safety::obstacleDistance).
      const safety::Obstacle cylinder{
        obstacle.name, obstacle.x, obstacle.y, obstacle.radius, obstacle.height};
      const double distance =
        safety::obstacleDistance(cylinder, sample.drone_position).distance;
      if (distance < min_obstacle_distance) {
        min_obstacle_distance = distance;
        min_obstacle_name = obstacle.name;
      }
    }

    // The retention
    if (scored) {
      // The state of the interval that starts at this sample is the state of this
      // sample: the presence changes at the stamp of the sample that reports it.
      if (sample.selected_usable) {
        present_time += dt;
      } else {
        lost_time += dt;
      }
      summary.scored_time_s += dt;
    }
    if (has_previous) {
      if (!previous_present && sample.selected_usable) {
        // The target came back: the dropout that was running ends here.
        const double duration = sample.time_s - loss_start;
        summary.dropout_count++;
        summary.max_loss_duration = std::max(summary.max_loss_duration, duration);
        if (duration >= params_.reacquire_loss_time_s) {
          summary.long_loss_count++;
          summary.reacquired_count++;
          summary.reacquire_delay_mean += duration;
        }
      } else if (previous_present && !sample.selected_usable) {
        // The target went missing.
        loss_start = sample.time_s;
      }
    }
    previous_present = sample.selected_usable;
    has_previous = true;

    // The framing
    if (scored) {
      if (sample.selected_track_id >= 0 && sample.selected_usable) {
        const double error = azimuthError(
          sample.drone_yaw, sample.drone_position, sample.selected_position) * kDegPerRad;
        framing_errors.push_back(error);
        framing_errors_present.push_back(error);
      } else {
        // The target was lost: it was not in the frame either.
        framing_errors.push_back(std::numeric_limits<double>::infinity());
      }
      if (sample.has_target_truth) {
        framing_target_errors.push_back(
          azimuthError(sample.drone_yaw, sample.drone_position, sample.target_truth) * kDegPerRad);
      }
    }
  }

  // A run that ends with the target still missing: the last dropout never closed.
  if (has_previous && !previous_present) {
    const double duration = t1 - loss_start;
    summary.dropout_count++;
    summary.max_loss_duration = std::max(summary.max_loss_duration, duration);
    if (duration >= params_.reacquire_loss_time_s) {
      summary.long_loss_count++;
    }
  }

  summary.min_person_distance_3d = min_person_distance_3d;
  summary.min_person_surface_distance_3d = min_person_distance_3d - params_.drone_radius;
  summary.min_person_margin_3d = min_person_margin_3d;
  summary.min_person_margin_3d_effective = min_person_margin_3d_effective;
  summary.min_person_distance_horizontal = min_person_distance;
  summary.min_person_margin_horizontal = min_person_margin_horizontal;
  summary.min_person_margin_horizontal_effective = min_person_margin_horizontal_effective;
  summary.min_person_track_id = min_person_track_id;
  summary.min_obstacle_distance = min_obstacle_distance;
  summary.min_obstacle_margin = min_obstacle_distance - params_.d_obstacle_min;
  summary.min_obstacle_name = min_obstacle_name;

  summary.target_present_time = present_time;
  summary.target_lost_time = lost_time;

  // The framing score: the fraction of the scored time the target was within the
  // tolerance. The time the target was lost counts against the score.
  double in_tolerance = 0.0;
  for (const double error : framing_errors) {
    if (error <= params_.framing_tolerance_deg) {
      in_tolerance += 1.0;
    }
  }
  summary.framing_score = framing_errors.empty() ?
    0.0 : in_tolerance / static_cast<double>(framing_errors.size());
  double present_in_tolerance = 0.0;
  for (const double error : framing_errors_present) {
    if (error <= params_.framing_tolerance_deg) {
      present_in_tolerance += 1.0;
    }
  }
  summary.framing_score_conditional = framing_errors_present.empty() ?
    0.0 : present_in_tolerance / static_cast<double>(framing_errors_present.size());

  double sum = 0.0;
  for (const double error : framing_errors_present) {
    sum += error;
  }
  summary.framing_mean_deg = framing_errors_present.empty() ?
    0.0 : sum / static_cast<double>(framing_errors_present.size());
  summary.framing_p95_deg = percentile(framing_errors_present, 0.95);
  summary.framing_max_deg = framing_errors_present.empty() ?
    0.0 : *std::max_element(framing_errors_present.begin(), framing_errors_present.end());

  double target_sum = 0.0;
  size_t target_in_tolerance = 0;
  for (const double error : framing_target_errors) {
    target_sum += error;
    if (error <= params_.framing_tolerance_deg) {
      target_in_tolerance++;
    }
  }
  summary.framing_target_score = framing_target_errors.empty() ?
    0.0 : static_cast<double>(target_in_tolerance) /
    static_cast<double>(framing_target_errors.size());
  summary.framing_target_mean_deg = framing_target_errors.empty() ?
    0.0 : target_sum / static_cast<double>(framing_target_errors.size());

  if (summary.long_loss_count > 0) {
    summary.reacquire_delay_mean /= static_cast<double>(summary.long_loss_count);
  }

  // --- The verdicts ---------------------------------------------------------
  // The two rules of the keep-out are both in force, and the stricter of the two
  // governs: both margins have to hold.
  summary.distance_ok = std::isfinite(min_person_distance_3d) &&
    summary.min_person_margin_3d >= 0.0 && summary.min_person_margin_horizontal >= 0.0 &&
    summary.min_obstacle_margin >= 0.0;
  // The framing verdict: over the whole scored time, or - for the scenarios that
  // hide the target by design - over the time the target is tracked. Both scores
  // are in the summary either way.
  const auto & tracked_only = params_.framing_over_tracked_time_scenarios;
  summary.framing_over_tracked_time =
    std::find(tracked_only.begin(), tracked_only.end(), params_.scenario) != tracked_only.end();
  summary.framing_ok = summary.framing_over_tracked_time ?
    (!framing_errors_present.empty() &&
    summary.framing_score_conditional >= params_.min_framing_score) :
    (!framing_errors.empty() && summary.framing_score >= params_.min_framing_score);
  summary.reacquire_ok = summary.long_loss_count == 0 ||
    summary.reacquired_count == summary.long_loss_count;
  summary.all_ok = summary.distance_ok && summary.framing_ok && summary.reacquire_ok;

  return summary;
}

std::string MetricsRecorder::toJson(const Summary & summary, const RecorderParams & params)
{
  std::ostringstream out;
  out << "{\n";
  out << "  \"scenario\": " << jsonString(params.scenario) << ",\n";
  out << "  \"seed\": " << params.seed << ",\n";
  out << "  \"d_min_3d\": " << jsonNumber(params.d_min_3d) << ",\n";
  out << "  \"d_min_horizontal\": " << jsonNumber(params.d_min_horizontal) << ",\n";
  out << "  \"d_obstacle_min\": " << jsonNumber(params.d_obstacle_min) << ",\n";
  out << "  \"framing_tolerance_deg\": " << jsonNumber(params.framing_tolerance_deg) << ",\n";
  out << "  \"warmup_s\": " << jsonNumber(params.warmup_s) << ",\n";
  out << "  \"sample_count\": " << summary.sample_count << ",\n";
  out << "  \"duration_s\": " << jsonNumber(summary.duration_s) << ",\n";
  out << "  \"scored_time_s\": " << jsonNumber(summary.scored_time_s) << ",\n";
  out << "  \"min_person_distance_3d\": " << jsonNumber(summary.min_person_distance_3d) << ",\n";
  out << "  \"min_person_surface_distance_3d\": " <<
    jsonNumber(summary.min_person_surface_distance_3d) << ",\n";
  out << "  \"min_person_margin_3d\": " << jsonNumber(summary.min_person_margin_3d) << ",\n";
  out << "  \"min_person_margin_3d_effective\": " <<
    jsonNumber(summary.min_person_margin_3d_effective) << ",\n";
  out << "  \"min_person_distance_horizontal\": " <<
    jsonNumber(summary.min_person_distance_horizontal) << ",\n";
  out << "  \"min_person_margin_horizontal\": " <<
    jsonNumber(summary.min_person_margin_horizontal) << ",\n";
  out << "  \"min_person_margin_horizontal_effective\": " <<
    jsonNumber(summary.min_person_margin_horizontal_effective) << ",\n";
  out << "  \"min_person_track_id\": " << summary.min_person_track_id << ",\n";
  out << "  \"min_obstacle_distance\": " << jsonNumber(summary.min_obstacle_distance) << ",\n";
  out << "  \"min_obstacle_margin\": " << jsonNumber(summary.min_obstacle_margin) << ",\n";
  out << "  \"min_obstacle_name\": " << jsonString(summary.min_obstacle_name) << ",\n";
  out << "  \"framing_score\": " << jsonNumber(summary.framing_score) << ",\n";
  out << "  \"framing_score_conditional\": " << jsonNumber(summary.framing_score_conditional) <<
    ",\n";
  out << "  \"framing_mean_deg\": " << jsonNumber(summary.framing_mean_deg) << ",\n";
  out << "  \"framing_p95_deg\": " << jsonNumber(summary.framing_p95_deg) << ",\n";
  out << "  \"framing_max_deg\": " << jsonNumber(summary.framing_max_deg) << ",\n";
  out << "  \"framing_target_score\": " << jsonNumber(summary.framing_target_score) << ",\n";
  out << "  \"framing_target_mean_deg\": " << jsonNumber(summary.framing_target_mean_deg) << ",\n";
  out << "  \"target_present_time\": " << jsonNumber(summary.target_present_time) << ",\n";
  out << "  \"target_lost_time\": " << jsonNumber(summary.target_lost_time) << ",\n";
  out << "  \"dropout_count\": " << summary.dropout_count << ",\n";
  out << "  \"max_loss_duration\": " << jsonNumber(summary.max_loss_duration) << ",\n";
  out << "  \"long_loss_count\": " << summary.long_loss_count << ",\n";
  out << "  \"reacquired_count\": " << summary.reacquired_count << ",\n";
  out << "  \"reacquire_delay_mean\": " << jsonNumber(summary.reacquire_delay_mean) << ",\n";
  out << "  \"distance_ok\": " << (summary.distance_ok ? "true" : "false") << ",\n";
  out << "  \"framing_ok\": " << (summary.framing_ok ? "true" : "false") << ",\n";
  out << "  \"framing_basis\": \"" <<
    (summary.framing_over_tracked_time ? "tracked" : "all") << "\",\n";
  out << "  \"reacquire_ok\": " << (summary.reacquire_ok ? "true" : "false") << ",\n";
  out << "  \"all_ok\": " << (summary.all_ok ? "true" : "false") << "\n";
  out << "}\n";
  return out.str();
}

std::string MetricsRecorder::toCsv(const std::vector<TimeSeriesPoint> & series)
{
  std::ostringstream out;
  out << "time_s,drone_x,drone_y,drone_z,target_x,target_y,target_z,"
    "framing_error_deg,cone_angle_deg,framing_target_error_deg,"
    "min_person_distance_3d,min_person_distance_horizontal,min_obstacle_distance,"
    "target_present,scored,"
    "selected_track_id,track_count,safety_active,reason\n";
  out << std::fixed << std::setprecision(4);
  for (const TimeSeriesPoint & point : series) {
    out << point.time_s << "," << point.drone_x << "," << point.drone_y << "," <<
      point.drone_z << "," << point.target_x << "," << point.target_y << "," <<
      point.target_z << "," << point.framing_error_deg << "," << point.cone_angle_deg << "," <<
      point.framing_target_error_deg << "," << point.min_person_distance_3d << "," <<
      point.min_person_distance_horizontal << "," <<
      point.min_obstacle_distance << "," << (point.target_present ? 1 : 0) << "," <<
      (point.scored ? 1 : 0) << "," << point.selected_track_id << "," << point.track_count <<
      "," << (point.safety_active ? 1 : 0) << "," << point.reason << "\n";
  }
  return out.str();
}

std::string MetricsRecorder::toText(const Summary & summary, const RecorderParams & params)
{
  std::ostringstream out;
  out << std::fixed << std::setprecision(2);
  out << "=== FOLLOW run: scenario '" << params.scenario << "'" <<
    (params.seed != 0 ? " (seed " + std::to_string(params.seed) + ")" : "") << " ===\n";
  out << "samples " << summary.sample_count << " over " << summary.duration_s <<
    " s, scored " << summary.scored_time_s << " s\n";
  out << "min distance to a person/vehicle: " << summary.min_person_distance_3d <<
    " m in 3D (surface " << summary.min_person_surface_distance_3d <<
    " m, margin " << summary.min_person_margin_3d << " m), " <<
    summary.min_person_distance_horizontal << " m horizontal (margin " <<
    summary.min_person_margin_horizontal << " m), track " <<
    summary.min_person_track_id << ")\n";
  out << "min distance to an obstacle: " << summary.min_obstacle_distance <<
    " m (margin " << summary.min_obstacle_margin << " m, " << summary.min_obstacle_name << ")\n";
  out << "framing: " << summary.framing_score * 100.0 << " % within +-" <<
    params.framing_tolerance_deg << " deg, mean " << summary.framing_mean_deg <<
    " deg, p95 " << summary.framing_p95_deg << " deg, max " << summary.framing_max_deg << " deg\n";
  out << "framing against the ground truth: " << summary.framing_target_score * 100.0 <<
    " %, mean " << summary.framing_target_mean_deg << " deg\n";
  out << "target present " << summary.target_present_time << " s, lost " <<
    summary.target_lost_time << " s, " << summary.dropout_count << " dropouts, longest " <<
    summary.max_loss_duration << " s\n";
  out << "long losses: " << summary.long_loss_count << ", reacquired " <<
    summary.reacquired_count << ", mean delay " << summary.reacquire_delay_mean << " s\n";
  out << "VERDICT: distance " << (summary.distance_ok ? "PASS" : "FAIL") <<
    ", framing " << (summary.framing_ok ? "PASS" : "FAIL") <<
    ", reacquisition " << (summary.reacquire_ok ? "PASS" : "FAIL") <<
    " -> " << (summary.all_ok ? "PASS" : "FAIL") << "\n";
  return out.str();
}

}  // namespace metrics
}  // namespace interceptor
