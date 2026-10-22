/**
 * @file metrics_recorder.hpp
 * @brief The metrics of a FOLLOW run: the minimum distances to the people, the
 *        vehicles and the obstacles, the framing error against the boresight and the
 *        retention of the target track. Plain C++ and Eigen, no ROS, so the scoring
 *        can be unit tested on synthetic trajectories.
 *
 * What it measures
 * ----------------
 * The exit criteria of Phase 2 of the plan (section 5) are:
 *
 *   * the minimum distance to every person or vehicle is >= d_min and to every
 *     obstacle >= d_obstacle_min, in 100 % of the runs;
 *   * the target is within +-15 degrees of the boresight >= 95 % of the time;
 *   * the track is reacquired after a 2 s occlusion.
 *
 * This class computes exactly those three numbers from a time series of samples and
 * turns them into per-run verdicts, so a batch of runs can be scored automatically.
 *
 * The distances
 * -------------
 * The safety rule is about *every* person and vehicle, not only the one that is
 * being followed, so the recorder looks at every track the tracker reports. The
 * distances are the ones the safety filter enforces (section 4.3), which after
 * P3.4 are two rules at once, and the stricter of the two governs:
 *
 *   * the 3D rule: the distance between the centres, in three dimensions, minus
 *     the drone's own radius, has to be at least d_min_3d, at any altitude. The
 *     filter inflates it with the track's position uncertainty
 *     (d_min_3d + k * sigma), so the recorder reports the margin against that
 *     effective threshold as well.
 *   * the horizontal rule: the plan's flat standoff d_min_horizontal, in force
 *     while the drone is at most keepout_horizontal_altitude above the person.
 *     Above that the margin is given up at keepout_horizontal_taper metres per
 *     metre of climb, and the 3D rule governs alone.
 *   * an obstacle: the horizontal distance between the centres, minus the
 *     obstacle's own radius, has to be at least d_obstacle_min. A short obstacle
 *     is not a hazard for a drone that flies above it, so the same height rule
 *     as the filter's applies: an obstacle the drone is above is not counted.
 *
 * The framing error
 * -----------------
 * The camera is mounted level on the nose of the airframe (the body frame is FLU, so
 * the boresight is the +x axis of the body), and the drone flies above the target,
 * so the *azimuthal* angle is what decides whether the target is in the frame: the
 * bearing to the target against the drone's heading. The full three-dimensional
 * cone angle is reported as well, but it is dominated by the height offset the
 * planner is asked for (5 m up, 8 m back is 32 degrees), so it cannot be the
 * criterion.
 *
 * The retention
 * -------------
 * The selected target counts as *present* while the tracker reports it as usable
 * (a valid, confirmed track). Every gap is a dropout; a gap of at least
 * `reacquire_loss_time_s` is a long one, and the run passes the reacquisition
 * criterion only if every long gap ends with the target coming back.
 */

#pragma once

#include <Eigen/Dense>

#include <cstdint>
#include <string>
#include <vector>

namespace interceptor
{
namespace metrics
{

/// A static obstacle: a cylinder standing on the ground. Same as safety::Obstacle,
/// repeated here so the library has no dependency on the safety filter.
struct Obstacle
{
  std::string name;
  double x = 0.0;       // m, world frame
  double y = 0.0;       // m, world frame
  double radius = 0.0;  // m, the obstacle's own radius
  double height = 0.0;  // m, its top; the keep-out only applies below this
};

/// A person or vehicle track, as the tracker reports it.
struct TrackSample
{
  int track_id = -1;
  int class_id = 0;  // the TargetClass value: 0=person, 1=car, 2=truck, 3=bicycle
  Eigen::Vector3d position = Eigen::Vector3d::Zero();  // m, world frame
  /// The horizontal position uncertainty [m]: the standard deviation of the
  /// horizontal position, as the safety filter computes it.
  double position_sigma = 0.0;
  bool confirmed = false;
  bool is_valid = false;
};

/// One cycle of the recorder: everything that is known at one instant.
struct Sample
{
  double time_s = 0.0;
  Eigen::Vector3d drone_position = Eigen::Vector3d::Zero();  // m, world frame
  /// The drone's heading [rad], the yaw of the body frame in the world frame.
  double drone_yaw = 0.0;
  /// The boresight: the unit vector of the +x axis of the body, in the world frame.
  Eigen::Vector3d boresight = Eigen::Vector3d(1.0, 0.0, 0.0);
  /// The track the tracker selected, -1 when there is none.
  int selected_track_id = -1;
  /// The position of the selected track [m]. It is the same track the tracker
  /// publishes on /target/state.
  Eigen::Vector3d selected_position = Eigen::Vector3d::Zero();
  /// True when the selected track is usable (confirmed and valid).
  bool selected_usable = false;
  /// Every track the tracker reports.
  std::vector<TrackSample> tracks;
  /// The ground truth of the scenario's target, when the scenario publishes it.
  bool has_target_truth = false;
  Eigen::Vector3d target_truth = Eigen::Vector3d::Zero();
  /// The state of the safety filter, from /mission/status.
  bool safety_active = false;
  std::string reason;
};

struct RecorderParams
{
  // --- The thresholds of the exit criteria ---------------------------------
  // The hard 3D distance to any person or vehicle track [m].
  double d_min_3d = 5.0;
  // The plan's horizontal standoff [m], given up above keepout_horizontal_altitude.
  double d_min_horizontal = 5.0;
  double keepout_horizontal_altitude = 5.0;  // m, the height of the horizontal keep-out
  double keepout_horizontal_taper = 1.0;     // m of margin per m of climb
  double d_obstacle_min = 3.0;   // m, to the surface of any static obstacle
  double drone_radius = 0.3;     // m, the drone's own radius
  double uncertainty_gain = 2.0;  // k in d_min + k * sigma, the filter's inflation
  double framing_tolerance_deg = 15.0;  // deg, the half-width of the frame
  double min_framing_score = 0.95;     // the criterion: 95 % of the time
  double reacquire_loss_time_s = 2.0;   // s, a loss at least this long is a long one

  // --- The run -------------------------------------------------------------
  /// The time before the run is scored [s]: the drone needs a moment to reach the
  /// framing point after it takes off.
  double warmup_s = 0.0;
  std::string scenario = "unknown";
  int64_t seed = 0;
};

/// One point of the time series, for the CSV output.
struct TimeSeriesPoint
{
  double time_s = 0.0;
  double drone_x = 0.0;   // m, the drone's position
  double drone_y = 0.0;
  double drone_z = 0.0;
  double target_x = 0.0;  // m, the ground truth of the scenario's target
  double target_y = 0.0;
  double target_z = 0.0;
  int selected_track_id = -1;  // the track the tracker selected
  int track_count = 0;         // the number of tracks the tracker reports
  bool safety_active = false;  // the safety filter changed the command
  std::string reason;          // why the safety filter did (empty when nominal)
  double framing_error_deg = 0.0;      // the azimuthal error to the selected track
  double cone_angle_deg = 0.0;          // the three-dimensional angle to it
  double framing_target_error_deg = 0.0;  // the azimuthal error to the ground truth
  double min_person_distance = 0.0;     // m, the closest track of this sample
  double min_obstacle_distance = 0.0;   // m, the closest obstacle of this sample
  bool target_present = false;
  bool scored = false;                 // false during the warm-up
};

/// Everything the recorder knows about a run.
struct Summary
{
  // --- The distances --------------------------------------------------------
  /// The smallest 3D distance to the centre of any person or vehicle track over
  /// the whole run [m]. This is the distance the hard rule is about.
  double min_person_distance_3d = 0.0;
  /// The same, between the surfaces (the drone's own radius comes off) [m].
  double min_person_surface_distance_3d = 0.0;
  /// The smallest margin against d_min_3d: the surface distance - d_min_3d [m].
  /// Negative means the criterion was broken.
  double min_person_margin_3d = 0.0;
  /// The margin against the uncertainty-inflated threshold the filter enforces,
  /// d_min_3d + k * sigma.
  double min_person_margin_3d_effective = 0.0;
  /// The smallest horizontal distance to the centre of any person or vehicle
  /// track [m].
  double min_person_distance_horizontal = 0.0;
  /// The smallest margin against the horizontal standoff the filter enforces at
  /// that height: the surface distance - the effective standoff [m].
  double min_person_margin_horizontal = 0.0;
  int min_person_track_id = -1;
  /// The smallest distance to the surface of any obstacle over the whole run [m].
  double min_obstacle_distance = 0.0;
  /// The margin against d_obstacle_min: surface distance - d_obstacle_min [m].
  double min_obstacle_margin = 0.0;
  std::string min_obstacle_name;

  // --- The framing ----------------------------------------------------------
  /// The fraction of the scored time the target was within the tolerance of the
  /// boresight. The time the target was lost counts against the score.
  double framing_score = 0.0;
  /// The same, over the time the target was present only.
  double framing_score_conditional = 0.0;
  double framing_mean_deg = 0.0;
  double framing_p95_deg = 0.0;
  double framing_max_deg = 0.0;
  /// The same three numbers for the ground truth of the scenario's target: they
  /// differ from the ones above when the drone framed the wrong track.
  double framing_target_score = 0.0;
  double framing_target_mean_deg = 0.0;

  // --- The retention --------------------------------------------------------
  double target_present_time = 0.0;  // s
  double target_lost_time = 0.0;     // s
  int dropout_count = 0;             // the number of gaps
  double max_loss_duration = 0.0;    // s, the longest gap
  int long_loss_count = 0;           // the gaps of at least reacquire_loss_time_s
  int reacquired_count = 0;          // of those, the ones that ended with the target back
  double reacquire_delay_mean = 0.0;  // s, the mean length of the reacquired long gaps

  // --- The run --------------------------------------------------------------
  size_t sample_count = 0;
  double duration_s = 0.0;
  double scored_time_s = 0.0;

  // --- The verdicts ---------------------------------------------------------
  bool distance_ok = false;
  bool framing_ok = false;
  bool reacquire_ok = false;
  bool all_ok = false;
};

/// The metrics of a FOLLOW run.
///
/// The class is not thread safe and expects to be driven from one timer. The node
/// around it only does the plumbing: the subscriptions, the timer and the output.
class MetricsRecorder
{
public:
  explicit MetricsRecorder(const RecorderParams & params = RecorderParams());

  const RecorderParams & params() const {return params_;}
  void setParams(const RecorderParams & params) {params_ = params;}

  /// The static obstacles the distances are measured against. They come from
  /// safety_params.yaml, the same list the safety filter keeps out of.
  void setObstacles(const std::vector<Obstacle> & obstacles) {obstacles_ = obstacles;}
  const std::vector<Obstacle> & obstacles() const {return obstacles_;}

  /// Adds one sample. The samples are expected to arrive in time order.
  void addSample(const Sample & sample);

  /// The metrics of the run so far.
  Summary summarize() const;

  /// The time series, for the CSV output.
  const std::vector<TimeSeriesPoint> & timeSeries() const {return series_;}

  /// The summary as JSON, so a batch of runs can be aggregated by a script.
  static std::string toJson(const Summary & summary, const RecorderParams & params);
  /// The time series as CSV.
  static std::string toCsv(const std::vector<TimeSeriesPoint> & series);
  /// A human readable summary, for the log.
  static std::string toText(const Summary & summary, const RecorderParams & params);

  // --- The geometry, exposed so it can be tested on its own -----------------

  /// The azimuthal angle between the drone's heading and the bearing to a point
  /// [rad]. Always positive and never larger than pi.
  static double azimuthError(
    double yaw, const Eigen::Vector3d & from, const Eigen::Vector3d & to);

  /// The angle between the boresight and the direction to a point [rad].
  static double coneAngle(
    const Eigen::Vector3d & boresight, const Eigen::Vector3d & from, const Eigen::Vector3d & to);

  /// The horizontal distance between two points [m].
  static double horizontalDistance(const Eigen::Vector3d & a, const Eigen::Vector3d & b);

  /// The 3D distance between two points [m].
  static double distance(const Eigen::Vector3d & a, const Eigen::Vector3d & b);

  /// The horizontal standoff the filter enforces at a height above the person,
  /// the same formula as the filter's own: the plan's flat standoff, given up
  /// with the height. Zero when the rule is not in force.
  double horizontalStandoff(double vertical_separation, double position_sigma) const;

private:
  RecorderParams params_;
  std::vector<Obstacle> obstacles_;

  // The samples, in time order
  std::vector<Sample> samples_;
  std::vector<TimeSeriesPoint> series_;
};

}  // namespace metrics
}  // namespace interceptor
