/**
 * @file intercept_metrics.hpp
 * @brief The scoring of an INTERCEPT engagement: the metrics of one run and the
 *        aggregate over a batch of runs (plan section 5, phase 3 exit criteria).
 *
 * Plain C++ and no ROS, so the scoring can be unit tested on synthetic
 * engagement records and reused by whatever drives the runs. The node
 * (src/metrics/intercept_metrics_node.cpp) is the thin layer on top: it turns
 * the live topics into samples and hands them to the scorer.
 *
 * What is measured
 * ----------------
 * The capture is the guidance's own condition, reproduced here from the same
 * two inputs the guidance uses (the tracker's target state and the drone's
 * odometry): the range is inside the capture radius and the relative speed is
 * under the limit. The *envelope* is weaker: the range was inside the capture
 * radius at all, whatever the relative speed was. The plan's exit criteria are
 * stated over the envelope, so both are reported and the caller says which one
 * it compared against the threshold.
 *
 * The abort is the safety filter's: it latches, so the first sample that
 * reports it carries the reason, and the reason is kept as the run's abort
 * reason.
 *
 * The class of the engaged track is checked on every sample of an engagement:
 * a non-uav track that is flown at is a violation of the class rule, and the
 * count of such samples is the metric (the criteria demand zero).
 *
 * The records are CSV: one row per run, a fixed set of columns. That is the
 * format the batch reads back and aggregates, and it is what the report tool
 * prints the table from.
 */

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace interceptor
{
namespace metrics
{

/// The TargetClass values the messages use (person=0, car=1, truck=2,
/// bicycle=3, uav=4).
constexpr int kPersonClass = 0;
constexpr int kUavClass = 4;

/// The thresholds the scorer applies.
struct MetricsParams
{
  /// The capture envelope [m]: the range under which a capture counts.
  double capture_radius_m = 1.5;
  /// The largest relative speed a capture may have [m/s].
  double capture_max_rel_speed_mps = 5.0;
  /// The only class that may be engaged.
  int uav_class_id = kUavClass;
};

/// One observation of an engagement.
struct EngagementSample
{
  /// The time of the observation [s], on the engagement's own clock.
  double time_s = 0.0;
  /// The range between the interceptor and the target [m].
  double range_m = 0.0;
  /// The relative speed between the two [m/s].
  double relative_speed_mps = 0.0;
  /// The target's own speed [m/s]: a capture of a fast target is worth more
  /// than a capture of a hovering one, and the report says which it was.
  double target_speed_mps = 0.0;
  /// The zero-effort miss the guidance reports [m].
  double miss_distance_m = 0.0;
  /// The class of the target the interceptor is flying at.
  int target_class_id = -1;
  /// True while the guidance is engaged (PURSUE, CAPTURE or DONE).
  bool guidance_engaging = false;
  /// True when the safety filter has aborted the engagement.
  bool abort_active = false;
  /// Why it aborted.
  std::string abort_reason;
};

/// The scored outcome of one engagement.
struct EngagementResult
{
  /// The guidance's own capture condition was met.
  bool capture_confirmed = false;
  /// The range at the capture [m].
  double capture_range_m = 0.0;
  /// The relative speed at the capture [m/s].
  double capture_relative_speed_mps = 0.0;
  /// The time of the capture [s].
  double capture_time_s = 0.0;
  /// The target's own speed at the capture [m/s].
  double capture_target_speed_mps = 0.0;
  /// The capture envelope was entered at all, whatever the relative speed was.
  bool envelope_reached = false;
  /// The closest approach [m].
  double closest_approach_m = 0.0;
  /// The smallest zero-effort miss the guidance reported [m].
  double min_miss_distance_m = 0.0;
  /// The safety filter aborted the engagement.
  bool abort_fired = false;
  /// Why it aborted.
  std::string abort_reason;
  /// The number of samples in which a non-uav track was being flown at.
  int non_uav_engagements = 0;
  /// The phase the engagement ended in.
  std::string end_phase;
  /// How long the engagement lasted [s].
  double duration_s = 0.0;
  /// The number of samples the score is based on.
  int sample_count = 0;
};

/// The scores of one run, with the run's identity.
struct RunRecord
{
  /// The name of the scenario.
  std::string scenario;
  /// The seed of the run.
  int seed = 0;
  /// The scores.
  EngagementResult result;
};

/// The rates of one scenario over a batch of runs.
struct ScenarioSummary
{
  /// The name of the scenario.
  std::string scenario;
  /// The number of runs.
  int runs = 0;
  /// The runs in which the capture was confirmed.
  int captures = 0;
  /// The runs in which the capture envelope was reached.
  int envelopes = 0;
  /// The runs in which the abort fired.
  int aborts = 0;
  /// The runs in which a non-uav track was engaged.
  int non_uav_runs = 0;
  /// The total number of non-uav engagement samples over the runs.
  int non_uav_samples = 0;
  /// The rate of confirmed captures over the runs [-].
  double capture_rate = 0.0;
  /// The rate of runs that reached the envelope over the runs [-].
  double envelope_rate = 0.0;
  /// The rate of runs that aborted over the runs [-].
  double abort_rate = 0.0;
  /// The median range at the capture over the runs that captured [m].
  double median_capture_range_m = 0.0;
  /// The largest relative speed at the capture over the runs that captured [m/s].
  double worst_capture_relative_speed_mps = 0.0;
  /// The median closest approach over the runs [m].
  double median_closest_approach_m = 0.0;
  /// The median duration of the runs [s].
  double median_duration_s = 0.0;
};

/// The plan's exit criteria, as rates over the runs of a scenario.
struct ExitCriteria
{
  /// The rate of runs that reached the capture envelope the scenario demands.
  double min_envelope_rate = 0.0;
  /// The rate of runs that *confirmed* a capture the scenario demands.
  ///
  /// The plan's exit criteria are stated over the envelope, which is the weaker
  /// measure: the envelope is reached even when the interceptor flies through
  /// the target too fast for the capture to be confirmed. The confirmed capture
  /// is the stronger one, and it is the number an operator wants, so it is a
  /// criterion of its own.
  double min_capture_rate = 0.0;
  /// The rate of runs that aborted the scenario demands (the bystander scenario
  /// demands an abort in every run).
  double min_abort_rate = 0.0;
  /// The number of non-uav engagements the scenario allows (always zero).
  int max_non_uav_engagements = 0;
};

/// Scores an engagement from its samples.
///
/// The scorer is fed the samples in the order they arrive and keeps the
/// numbers the report needs: the first capture it sees, the closest approach,
/// the abort and its reason, and the count of the samples in which a non-uav
/// track was engaged.
class EngagementScorer
{
public:
  explicit EngagementScorer(MetricsParams params = MetricsParams());

  /// Feeds one sample of the engagement.
  void addSample(const EngagementSample & sample);

  /// The scores of the engagement so far.
  const EngagementResult & result() const {return result_;}

  /// The parameters the scorer applies.
  const MetricsParams & params() const {return params_;}

private:
  MetricsParams params_;
  EngagementResult result_;
  bool have_sample_ = false;
};

/// The rates of a batch of runs, per scenario.
class MetricsAggregator
{
public:
  /// Adds the scores of one run.
  void add(const RunRecord & record);

  /// The summary of every scenario that has runs.
  std::vector<ScenarioSummary> summarize() const;

  /// The summary of one scenario.
  ScenarioSummary summarize(const std::string & scenario) const;

  /// The number of runs that were added.
  int runs() const {return static_cast<int>(records_.size());}

  /// The scores of every run that was added.
  const std::vector<RunRecord> & records() const {return records_;}

  /// True when the scenario meets the criteria.
  static bool meetsCriteria(
    const ScenarioSummary & summary, const ExitCriteria & criteria);

private:
  std::vector<RunRecord> records_;
};

/// The header of the CSV the batch writes: one row per run.
std::string csvHeader();

/// The CSV row of one run. The columns are the header's, in the same order.
std::string csvRow(const RunRecord & record);

/// Reads the rows of a CSV written by csvRow(). Rows that do not parse are
/// skipped, so a truncated file (a run that was killed) does not break the
/// report.
std::vector<RunRecord> parseCsv(const std::string & text);

/// The summary of a batch as CSV: one row per scenario.
std::string summaryCsv(const std::vector<ScenarioSummary> & summaries);

}  // namespace metrics
}  // namespace interceptor
