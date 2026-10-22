/**
 * @file test_metrics_recorder.cpp
 * @brief Unit tests for the metrics of a FOLLOW run: the geometry of the framing
 *        error, the minimum distances to the people and the obstacles, the
 *        retention of the target track and the verdicts against the exit criteria.
 *
 * The recorder is ROS-free, so everything is driven with synthetic samples on a
 * synthetic clock: no simulator, no messages, no timing surprises.
 */

#include <gtest/gtest.h>
#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "metrics/metrics_recorder.hpp"

namespace interceptor
{
namespace metrics
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

/// The recorder with the thresholds of the plan.
RecorderParams testParams()
{
  RecorderParams params;
  params.d_min_3d = 5.0;
  params.d_min_horizontal = 5.0;
  params.keepout_horizontal_altitude = 5.0;
  params.keepout_horizontal_taper = 1.0;
  params.d_obstacle_min = 3.0;
  params.drone_radius = 0.3;
  params.uncertainty_gain = 2.0;
  params.framing_tolerance_deg = 15.0;
  params.min_framing_score = 0.95;
  params.reacquire_loss_time_s = 2.0;
  params.scenario = "walker";
  return params;
}

/// A person track at a position.
TrackSample personAt(int id, const Eigen::Vector3d & position, bool confirmed = true)
{
  TrackSample track;
  track.track_id = id;
  track.class_id = 0;  // person
  track.position = position;
  track.position_sigma = 0.2;
  track.confirmed = confirmed;
  track.is_valid = true;
  return track;
}

/// A sample of a run: the drone at the origin with its nose along +x, a person
/// straight ahead.
Sample sampleAt(double t, bool target_present = true)
{
  Sample sample;
  sample.time_s = t;
  sample.drone_position = Eigen::Vector3d::Zero();
  sample.drone_yaw = 0.0;
  sample.boresight = Eigen::Vector3d(1.0, 0.0, 0.0);
  sample.selected_track_id = target_present ? 7 : -1;
  sample.selected_position = Eigen::Vector3d(10.0, 0.0, 1.0);
  sample.selected_usable = target_present;
  sample.tracks.push_back(personAt(7, sample.selected_position));
  return sample;
}

}  // namespace

// --- The geometry ------------------------------------------------------------

TEST(MetricsRecorder, AzimuthErrorIsZeroStraightAhead)
{
  const double error = MetricsRecorder::azimuthError(
    0.0, Eigen::Vector3d::Zero(), Eigen::Vector3d(10.0, 0.0, 0.0));
  EXPECT_NEAR(error, 0.0, 1e-9);
}

TEST(MetricsRecorder, AzimuthErrorIsTheAngleToTheBearing)
{
  // The nose along +x, the target 90 degrees to the left.
  const double error = MetricsRecorder::azimuthError(
    0.0, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 10.0, 0.0));
  EXPECT_NEAR(error, 0.5 * kPi, 1e-9);
  // The nose along +x, the target behind: 180 degrees.
  const double behind = MetricsRecorder::azimuthError(
    0.0, Eigen::Vector3d::Zero(), Eigen::Vector3d(-10.0, 0.0, 0.0));
  EXPECT_NEAR(behind, kPi, 1e-9);
}

TEST(MetricsRecorder, AzimuthErrorIgnoresTheHeight)
{
  // The camera is level, so a target straight ahead but far below is still in the
  // middle of the frame horizontally.
  const double error = MetricsRecorder::azimuthError(
    0.0, Eigen::Vector3d(0.0, 0.0, 6.0), Eigen::Vector3d(8.0, 0.0, 1.0));
  EXPECT_NEAR(error, 0.0, 1e-9);
}

TEST(MetricsRecorder, AzimuthErrorWrapsAround)
{
  // The nose at 350 degrees, the target at 10 degrees: 20 degrees, not 340.
  const double error = MetricsRecorder::azimuthError(
    350.0 * kPi / 180.0, Eigen::Vector3d::Zero(),
    Eigen::Vector3d(
      10.0 * std::cos(10.0 * kPi / 180.0),
      10.0 * std::sin(10.0 * kPi / 180.0), 0.0));
  EXPECT_NEAR(error, 20.0 * kPi / 180.0, 1e-9);
}

TEST(MetricsRecorder, ConeAngleIsTheThreeDimensionalAngle)
{
  // The boresight along +x, the target straight ahead: no angle.
  EXPECT_NEAR(
    MetricsRecorder::coneAngle(
      Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d::Zero(),
      Eigen::Vector3d(10.0, 0.0, 0.0)),
    0.0, 1e-9);
  // The boresight along +x, the target 5 m below and 8 m ahead: 32 degrees.
  const double angle = MetricsRecorder::coneAngle(
    Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 6.0),
    Eigen::Vector3d(8.0, 0.0, 1.0));
  EXPECT_NEAR(angle, std::atan2(5.0, 8.0), 1e-9);
}

// --- The distances -----------------------------------------------------------

TEST(MetricsRecorder, MinimumDistanceToEveryTrack)
{
  MetricsRecorder recorder(testParams());
  Sample sample = sampleAt(0.0);
  // A second person, closer than the target.
  sample.tracks.push_back(personAt(8, Eigen::Vector3d(3.0, 4.0, 1.0)));
  recorder.addSample(sample);

  const Summary summary = recorder.summarize();
  // The hard rule is the 3D distance: the person is at (3, 4, 1) and the drone
  // at the origin, so 5 m away in the horizontal plane and a bit more in 3D.
  EXPECT_NEAR(summary.min_person_distance_horizontal, 5.0, 1e-9);
  EXPECT_EQ(summary.min_person_track_id, 8);
  EXPECT_NEAR(summary.min_person_distance_3d, std::sqrt(26.0), 1e-9);
  // The distances are between the surfaces.
  EXPECT_NEAR(summary.min_person_surface_distance_3d, std::sqrt(26.0) - 0.3, 1e-9);
  // The 3D margin is negative here: the person is sqrt(26) = 5.10 m away in 3D,
  // so between the surfaces it is 4.80 m, which is inside d_min_3d = 5.0. The
  // horizontal distance happens to be exactly 5 m, so the two rules disagree and
  // the stricter one - the 3D - is what the criterion is about.
  EXPECT_NEAR(summary.min_person_margin_3d, std::sqrt(26.0) - 0.3 - 5.0, 1e-9);
  EXPECT_LT(summary.min_person_margin_3d, 0.0);
  // The horizontal rule is in force: the drone is at the ground, so it is at most
  // keepout_horizontal_altitude above the person. Its threshold is the flat
  // standoff inflated by the track's uncertainty, d_min_horizontal + k * sigma
  // = 5.0 + 2 * 0.2 = 5.4 m, the same inflation the filter applies.
  EXPECT_NEAR(summary.min_person_margin_horizontal, 5.0 - 0.3 - 5.4, 1e-9);
  EXPECT_FALSE(summary.distance_ok);
}

TEST(MetricsRecorder, DistanceToEveryTrackNotOnlyTheSelectedOne)
{
  MetricsRecorder recorder(testParams());
  Sample sample = sampleAt(0.0);
  sample.tracks.push_back(personAt(8, Eigen::Vector3d(2.0, 0.0, 1.0)));
  recorder.addSample(sample);

  const Summary summary = recorder.summarize();
  EXPECT_NEAR(summary.min_person_distance_horizontal, 2.0, 1e-9);
  EXPECT_NEAR(summary.min_person_distance_3d, std::sqrt(5.0), 1e-9);
  EXPECT_EQ(summary.min_person_track_id, 8);
}

TEST(MetricsRecorder, TheUncertaintyIsPartOfTheEffectiveMargin)
{
  MetricsRecorder recorder(testParams());
  Sample sample = sampleAt(0.0);
  TrackSample noisy = personAt(7, Eigen::Vector3d(5.4, 0.0, 1.0));
  noisy.position_sigma = 0.5;
  sample.tracks = {noisy};
  recorder.addSample(sample);

  const Summary summary = recorder.summarize();
  // The person is at (5.4, 0, 1) and the drone at the origin, so it is
  // sqrt(5.4^2 + 1^2) = 5.492 m away in 3D, 5.192 m between the surfaces. The
  // margin against d_min_3d = 5.0 is therefore 0.192 m, but the filter enforces
  // d_min_3d + k * sigma = 5.0 + 2 * 0.5 = 6.0 m, so the margin against that is
  // negative: the filter would have pushed the drone away.
  const double range_3d = std::sqrt(5.4 * 5.4 + 1.0);
  EXPECT_NEAR(summary.min_person_surface_distance_3d, range_3d - 0.3, 1e-9);
  EXPECT_NEAR(summary.min_person_margin_3d, range_3d - 0.3 - 5.0, 1e-9);
  EXPECT_NEAR(summary.min_person_margin_3d_effective, range_3d - 0.3 - 6.0, 1e-9);
}

TEST(MetricsRecorder, TheMinimumIsOverTheWholeRun)
{
  MetricsRecorder recorder(testParams());
  recorder.addSample(sampleAt(0.0));
  Sample close = sampleAt(1.0);
  close.drone_position = Eigen::Vector3d(4.0, 0.0, 0.0);
  close.selected_position = Eigen::Vector3d(10.0, 0.0, 1.0);
  close.tracks[0].position = close.selected_position;
  recorder.addSample(close);
  recorder.addSample(sampleAt(2.0));

  const Summary summary = recorder.summarize();
  EXPECT_NEAR(summary.min_person_distance_horizontal, 6.0, 1e-9);
  EXPECT_NEAR(summary.min_person_distance_3d, std::sqrt(37.0), 1e-9);
}

TEST(MetricsRecorder, ObstaclesAreMeasuredToTheirSurface)
{
  MetricsRecorder recorder(testParams());
  Obstacle tree;
  tree.name = "oak_1";
  tree.x = 10.0;
  tree.y = 0.0;
  tree.radius = 4.0;
  tree.height = 6.5;
  recorder.setObstacles({tree});

  Sample sample = sampleAt(0.0);
  sample.drone_position = Eigen::Vector3d(0.0, 0.0, 1.0);
  recorder.addSample(sample);

  const Summary summary = recorder.summarize();
  EXPECT_NEAR(summary.min_obstacle_distance, 6.0, 1e-9);
  EXPECT_NEAR(summary.min_obstacle_margin, 3.0, 1e-9);
  EXPECT_EQ(summary.min_obstacle_name, "oak_1");
  EXPECT_TRUE(summary.distance_ok);
}

TEST(MetricsRecorder, AnObstacleTheDroneIsAboveIsNotAHazard)
{
  MetricsRecorder recorder(testParams());
  Obstacle bench;
  bench.name = "bench_1";
  bench.x = 1.0;
  bench.y = 0.0;
  bench.radius = 0.75;
  bench.height = 0.5;
  recorder.setObstacles({bench});

  // The drone flies over the bench: the keep-out of a 0.5 m bench does not apply.
  Sample sample = sampleAt(0.0);
  sample.drone_position = Eigen::Vector3d(0.0, 0.0, 6.0);
  recorder.addSample(sample);

  const Summary summary = recorder.summarize();
  // The bench is not counted at all: the drone is above it, so it is not a hazard.
  EXPECT_TRUE(std::isinf(summary.min_obstacle_distance));
  EXPECT_TRUE(summary.distance_ok);
}

// --- The framing -------------------------------------------------------------

TEST(MetricsRecorder, TheFramingScoreIsTheFractionWithinTheTolerance)
{
  MetricsRecorder recorder(testParams());
  // Nine samples within 15 degrees, one outside.
  for (int i = 0; i < 9; ++i) {
    recorder.addSample(sampleAt(0.1 * i));
  }
  Sample outside = sampleAt(1.0);
  outside.selected_position = Eigen::Vector3d(0.0, 10.0, 1.0);
  outside.tracks[0].position = outside.selected_position;
  recorder.addSample(outside);

  const Summary summary = recorder.summarize();
  EXPECT_NEAR(summary.framing_score, 0.9, 1e-9);
  EXPECT_FALSE(summary.framing_ok);
}

TEST(MetricsRecorder, TheTimeTheTargetIsLostCountsAgainstTheFraming)
{
  MetricsRecorder recorder(testParams());
  recorder.addSample(sampleAt(0.0));
  recorder.addSample(sampleAt(0.5));
  recorder.addSample(sampleAt(1.0, false));  // the target is lost
  recorder.addSample(sampleAt(1.5));

  const Summary summary = recorder.summarize();
  // Three of the four samples had the target in the frame.
  EXPECT_NEAR(summary.framing_score, 0.75, 1e-9);
  // The conditional score only looks at the samples that had a target.
  EXPECT_NEAR(summary.framing_score_conditional, 1.0, 1e-9);
  EXPECT_GT(summary.target_lost_time, 0.0);
}

TEST(MetricsRecorder, TheMeanAndThePercentileOfTheFramingError)
{
  MetricsRecorder recorder(testParams());
  // The target at 0, 1, 2, 3, 4 degrees to the left of the nose.
  for (int i = 0; i < 5; ++i) {
    Sample sample = sampleAt(0.1 * i);
    const double angle = i * kPi / 180.0;
    sample.selected_position = Eigen::Vector3d(
      10.0 * std::cos(angle), 10.0 * std::sin(angle), 1.0);
    sample.tracks[0].position = sample.selected_position;
    recorder.addSample(sample);
  }

  const Summary summary = recorder.summarize();
  EXPECT_NEAR(summary.framing_mean_deg, 2.0, 1e-6);
  EXPECT_NEAR(summary.framing_p95_deg, 3.8, 1e-6);
  EXPECT_NEAR(summary.framing_max_deg, 4.0, 1e-6);
  EXPECT_TRUE(summary.framing_ok);
}

TEST(MetricsRecorder, TheFramingAgainstTheGroundTruthIsReportedSeparately)
{
  MetricsRecorder recorder(testParams());
  Sample sample = sampleAt(0.0);
  // The drone frames the selected track perfectly, but the ground truth of the
  // target is 90 degrees to the side: the drone is framing the wrong person.
  sample.has_target_truth = true;
  sample.target_truth = Eigen::Vector3d(0.0, 10.0, 1.0);
  recorder.addSample(sample);

  const Summary summary = recorder.summarize();
  EXPECT_NEAR(summary.framing_score, 1.0, 1e-9);
  EXPECT_NEAR(summary.framing_target_score, 0.0, 1e-9);
  EXPECT_NEAR(summary.framing_target_mean_deg, 90.0, 1e-6);
}

// --- The retention ------------------------------------------------------------

TEST(MetricsRecorder, ALongLossThatEndsIsAReacquisition)
{
  MetricsRecorder recorder(testParams());
  recorder.addSample(sampleAt(0.0));
  recorder.addSample(sampleAt(1.0, false));
  recorder.addSample(sampleAt(2.0, false));
  recorder.addSample(sampleAt(3.0, false));
  recorder.addSample(sampleAt(4.0));  // the target is back

  const Summary summary = recorder.summarize();
  EXPECT_EQ(summary.dropout_count, 1);
  EXPECT_EQ(summary.long_loss_count, 1);
  EXPECT_EQ(summary.reacquired_count, 1);
  EXPECT_NEAR(summary.max_loss_duration, 3.0, 1e-9);
  EXPECT_TRUE(summary.reacquire_ok);
}

TEST(MetricsRecorder, AShortLossIsNotALongOne)
{
  MetricsRecorder recorder(testParams());
  recorder.addSample(sampleAt(0.0));
  recorder.addSample(sampleAt(1.0, false));
  recorder.addSample(sampleAt(2.0));

  const Summary summary = recorder.summarize();
  EXPECT_EQ(summary.dropout_count, 1);
  EXPECT_EQ(summary.long_loss_count, 0);
  EXPECT_TRUE(summary.reacquire_ok);
}

TEST(MetricsRecorder, ALongLossThatNeverEndsIsAFailure)
{
  MetricsRecorder recorder(testParams());
  recorder.addSample(sampleAt(0.0));
  recorder.addSample(sampleAt(1.0, false));
  recorder.addSample(sampleAt(4.0, false));

  const Summary summary = recorder.summarize();
  EXPECT_EQ(summary.long_loss_count, 1);
  EXPECT_EQ(summary.reacquired_count, 0);
  EXPECT_FALSE(summary.reacquire_ok);
}

TEST(MetricsRecorder, ThePresentAndLostTimeIsCounted)
{
  MetricsRecorder recorder(testParams());
  recorder.addSample(sampleAt(0.0));
  recorder.addSample(sampleAt(2.0));
  recorder.addSample(sampleAt(3.0, false));
  recorder.addSample(sampleAt(5.0));

  const Summary summary = recorder.summarize();
  EXPECT_NEAR(summary.target_present_time, 3.0, 1e-9);
  EXPECT_NEAR(summary.target_lost_time, 2.0, 1e-9);
}

// --- The warm-up and the verdicts --------------------------------------------

TEST(MetricsRecorder, TheWarmUpIsNotScored)
{
  RecorderParams params = testParams();
  params.warmup_s = 10.0;
  MetricsRecorder recorder(params);
  // Ten bad samples during the warm-up: the target is far off the nose.
  for (int i = 0; i < 10; ++i) {
    Sample sample = sampleAt(0.5 * i);
    sample.selected_position = Eigen::Vector3d(0.0, 10.0, 1.0);
    sample.tracks[0].position = sample.selected_position;
    recorder.addSample(sample);
  }
  recorder.addSample(sampleAt(10.5));

  const Summary summary = recorder.summarize();
  EXPECT_NEAR(summary.framing_score, 1.0, 1e-9);
  EXPECT_TRUE(summary.framing_ok);
}

TEST(MetricsRecorder, AnEmptyRunFailsEverything)
{
  MetricsRecorder recorder(testParams());
  const Summary summary = recorder.summarize();
  EXPECT_EQ(summary.sample_count, 0u);
  EXPECT_FALSE(summary.all_ok);
  EXPECT_FALSE(summary.framing_ok);
}

TEST(MetricsRecorder, TheVerdictsNeedEveryCriterion)
{
  MetricsRecorder recorder(testParams());
  recorder.addSample(sampleAt(0.0));
  recorder.addSample(sampleAt(1.0));
  const Summary summary = recorder.summarize();
  EXPECT_TRUE(summary.distance_ok);
  EXPECT_TRUE(summary.framing_ok);
  EXPECT_TRUE(summary.reacquire_ok);
  EXPECT_TRUE(summary.all_ok);
}

// --- The output ---------------------------------------------------------------

TEST(MetricsRecorder, TheJsonHasTheNumbersAndTheVerdicts)
{
  MetricsRecorder recorder(testParams());
  recorder.addSample(sampleAt(0.0));
  recorder.addSample(sampleAt(1.0));
  const std::string json = MetricsRecorder::toJson(recorder.summarize(), recorder.params());
  EXPECT_NE(json.find("\"framing_score\""), std::string::npos);
  EXPECT_NE(json.find("\"min_person_distance_3d\""), std::string::npos);
  EXPECT_NE(json.find("\"min_person_margin_horizontal\""), std::string::npos);
  EXPECT_NE(json.find("\"all_ok\": true"), std::string::npos);
  EXPECT_NE(json.find("\"scenario\": \"walker\""), std::string::npos);
}

TEST(MetricsRecorder, TheCsvHasOneRowPerSample)
{
  MetricsRecorder recorder(testParams());
  recorder.addSample(sampleAt(0.0));
  recorder.addSample(sampleAt(1.0));
  const std::string csv = MetricsRecorder::toCsv(recorder.timeSeries());
  // A header and two rows.
  EXPECT_EQ(std::count(csv.begin(), csv.end(), '\n'), 3);
  EXPECT_NE(csv.find("time_s,drone_x,drone_y,drone_z"), std::string::npos);
  // The time series carries both distances, because the two rules of the
  // keep-out disagree above the horizontal standoff's altitude.
  EXPECT_NE(csv.find("min_person_distance_3d,min_person_distance_horizontal"), std::string::npos);
}

}  // namespace metrics
}  // namespace interceptor
