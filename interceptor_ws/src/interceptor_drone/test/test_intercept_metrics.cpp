/**
 * @file test_intercept_metrics.cpp
 * @brief Unit tests for the INTERCEPT engagement scoring.
 *
 * The tests run on synthetic engagement records, so the scoring is checked
 * without a simulator: a capture, an envelope that was reached but not flown
 * (the overfly), an abort with its reason, and a non-uav engagement.
 */

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "metrics/intercept_metrics.hpp"

using namespace interceptor::metrics;  // NOLINT(build/namespaces)

namespace
{

/// A sample of an engagement at a given range and relative speed.
EngagementSample sample(
  double time_s, double range_m, double relative_speed_mps, double target_speed_mps = 0.0)
{
  EngagementSample s;
  s.time_s = time_s;
  s.range_m = range_m;
  s.relative_speed_mps = relative_speed_mps;
  s.target_speed_mps = target_speed_mps;
  s.miss_distance_m = range_m;
  s.target_class_id = kUavClass;
  s.guidance_engaging = true;
  return s;
}

/// The scores of a hover engagement that ends in a capture.
RunRecord capturedRun()
{
  EngagementScorer scorer;
  scorer.addSample(sample(0.0, 20.0, 5.0));
  scorer.addSample(sample(1.0, 10.0, 4.0));
  scorer.addSample(sample(2.0, 3.0, 3.0));
  scorer.addSample(sample(2.5, 1.2, 1.5));
  scorer.addSample(sample(3.0, 1.1, 0.8));

  RunRecord record;
  record.scenario = "hover";
  record.seed = 1;
  record.result = scorer.result();
  record.result.end_phase = "DONE";
  return record;
}

/// The scores of an engagement that reached the envelope but overflew it.
RunRecord overflownRun()
{
  EngagementScorer scorer;
  scorer.addSample(sample(0.0, 20.0, 5.0));
  scorer.addSample(sample(1.0, 10.0, 8.0));
  scorer.addSample(sample(2.0, 1.2, 9.0));  // inside the envelope, but too fast
  scorer.addSample(sample(3.0, 6.0, 9.0));  // and gone again

  RunRecord record;
  record.scenario = "hover";
  record.seed = 2;
  record.result = scorer.result();
  record.result.end_phase = "CAPTURE";
  return record;
}

/// The scores of an engagement the safety filter aborted.
RunRecord abortedRun()
{
  EngagementScorer scorer;
  scorer.addSample(sample(0.0, 20.0, 5.0));

  EngagementSample abort = sample(1.0, 15.0, 5.0);
  abort.abort_active = true;
  abort.abort_reason = "a person (track 3) is 12.0 m from the capture point";
  scorer.addSample(abort);

  // The abort latches, so a later sample without it does not clear it.
  scorer.addSample(sample(2.0, 14.0, 4.0));

  RunRecord record;
  record.scenario = "bystander";
  record.seed = 3;
  record.result = scorer.result();
  record.result.end_phase = "ABORT";
  return record;
}

}  // namespace

// ---------------------------------------------------------------------------
// The scorer
// ---------------------------------------------------------------------------

TEST(EngagementScorerTest, ConfirmsACaptureInsideTheEnvelope)
{
  EngagementScorer scorer;
  scorer.addSample(sample(0.0, 20.0, 5.0));
  scorer.addSample(sample(1.0, 1.2, 1.5));

  const EngagementResult & result = scorer.result();
  EXPECT_TRUE(result.capture_confirmed);
  EXPECT_TRUE(result.envelope_reached);
  EXPECT_NEAR(result.capture_range_m, 1.2, 1e-9);
  EXPECT_NEAR(result.capture_relative_speed_mps, 1.5, 1e-9);
  EXPECT_NEAR(result.capture_time_s, 1.0, 1e-9);
}

TEST(EngagementScorerTest, DoesNotConfirmACaptureThatIsTooFast)
{
  EngagementScorer scorer;
  scorer.addSample(sample(0.0, 1.2, 9.0));

  const EngagementResult & result = scorer.result();
  EXPECT_FALSE(result.capture_confirmed);
  // The envelope was reached though: the criteria are stated over it.
  EXPECT_TRUE(result.envelope_reached);
}

TEST(EngagementScorerTest, KeepsTheFirstCaptureAndNotALaterOne)
{
  EngagementScorer scorer;
  scorer.addSample(sample(0.0, 1.0, 1.0));
  scorer.addSample(sample(1.0, 0.5, 0.2));

  const EngagementResult & result = scorer.result();
  EXPECT_TRUE(result.capture_confirmed);
  EXPECT_NEAR(result.capture_range_m, 1.0, 1e-9);
  EXPECT_NEAR(result.capture_time_s, 0.0, 1e-9);
}

TEST(EngagementScorerTest, TracksTheClosestApproach)
{
  EngagementScorer scorer;
  scorer.addSample(sample(0.0, 20.0, 5.0));
  scorer.addSample(sample(1.0, 4.0, 3.0));
  scorer.addSample(sample(2.0, 0.8, 1.0));
  scorer.addSample(sample(3.0, 6.0, 3.0));

  EXPECT_NEAR(scorer.result().closest_approach_m, 0.8, 1e-9);
}

TEST(EngagementScorerTest, KeepsTheAbortAndItsReason)
{
  EngagementScorer scorer;
  scorer.addSample(sample(0.0, 20.0, 5.0));

  EngagementSample abort = sample(1.0, 15.0, 5.0);
  abort.abort_active = true;
  abort.abort_reason = "a person is 12.0 m from the capture point";
  scorer.addSample(abort);

  scorer.addSample(sample(2.0, 14.0, 4.0));

  const EngagementResult & result = scorer.result();
  EXPECT_TRUE(result.abort_fired);
  EXPECT_EQ(result.abort_reason, "a person is 12.0 m from the capture point");
}

TEST(EngagementScorerTest, CountsTheNonUavEngagements)
{
  EngagementScorer scorer;

  EngagementSample person = sample(0.0, 20.0, 5.0);
  person.target_class_id = kPersonClass;
  scorer.addSample(person);
  scorer.addSample(person);

  EngagementSample uav = sample(1.0, 10.0, 4.0);
  uav.target_class_id = kUavClass;
  scorer.addSample(uav);

  EXPECT_EQ(scorer.result().non_uav_engagements, 2);
}

TEST(EngagementScorerTest, IgnoresANonUavTrackThatIsNotEngaged)
{
  EngagementScorer scorer;

  // A person in the scene, but the guidance is not flying at it.
  EngagementSample bystander = sample(0.0, 20.0, 5.0);
  bystander.target_class_id = kPersonClass;
  bystander.guidance_engaging = false;
  scorer.addSample(bystander);

  EXPECT_EQ(scorer.result().non_uav_engagements, 0);
}

TEST(EngagementScorerTest, RejectsAnInvalidCaptureRadius)
{
  // The parameters are built up first, because the preprocessor splits a macro
  // argument on its commas and braces do not protect them.
  MetricsParams bad_radius;
  bad_radius.capture_radius_m = 0.0;
  EXPECT_THROW(EngagementScorer scorer{bad_radius}, std::invalid_argument);

  MetricsParams bad_speed;
  bad_speed.capture_max_rel_speed_mps = -1.0;
  EXPECT_THROW(EngagementScorer scorer{bad_speed}, std::invalid_argument);
}

// ---------------------------------------------------------------------------
// The aggregator
// ---------------------------------------------------------------------------

TEST(MetricsAggregatorTest, CountsTheRatesPerScenario)
{
  MetricsAggregator aggregator;
  aggregator.add(capturedRun());
  aggregator.add(overflownRun());
  aggregator.add(abortedRun());

  const ScenarioSummary hover = aggregator.summarize("hover");
  EXPECT_EQ(hover.runs, 2);
  EXPECT_EQ(hover.captures, 1);
  EXPECT_EQ(hover.envelopes, 2);
  EXPECT_NEAR(hover.capture_rate, 0.5, 1e-9);
  EXPECT_NEAR(hover.envelope_rate, 1.0, 1e-9);
  EXPECT_EQ(hover.aborts, 0);

  const ScenarioSummary bystander = aggregator.summarize("bystander");
  EXPECT_EQ(bystander.runs, 1);
  EXPECT_EQ(bystander.aborts, 1);
  EXPECT_NEAR(bystander.abort_rate, 1.0, 1e-9);
}

TEST(MetricsAggregatorTest, AggregatesEveryScenario)
{
  MetricsAggregator aggregator;
  aggregator.add(capturedRun());
  aggregator.add(abortedRun());

  const std::vector<ScenarioSummary> summaries = aggregator.summarize();
  ASSERT_EQ(summaries.size(), 2u);
  EXPECT_EQ(summaries[0].scenario, "hover");
  EXPECT_EQ(summaries[1].scenario, "bystander");
}

TEST(MetricsAggregatorTest, ReportsTheMedianAndTheWorstCapture)
{
  MetricsAggregator aggregator;
  aggregator.add(capturedRun());

  RunRecord slow = capturedRun();
  slow.seed = 2;
  slow.result.capture_range_m = 1.4;
  slow.result.capture_relative_speed_mps = 4.5;
  aggregator.add(slow);

  const ScenarioSummary hover = aggregator.summarize("hover");
  EXPECT_NEAR(hover.median_capture_range_m, 1.3, 1e-9);
  EXPECT_NEAR(hover.worst_capture_relative_speed_mps, 4.5, 1e-9);
}

TEST(MetricsAggregatorTest, ChecksTheExitCriteria)
{
  MetricsAggregator aggregator;
  aggregator.add(capturedRun());
  aggregator.add(overflownRun());

  const ScenarioSummary hover = aggregator.summarize("hover");

  // The plan's criterion is over the envelope: both runs reached it.
  ExitCriteria criteria;
  criteria.min_envelope_rate = 0.9;
  EXPECT_TRUE(MetricsAggregator::meetsCriteria(hover, criteria));

  // Over the confirmed capture it would fail: only one of the two captured.
  criteria.min_envelope_rate = 0.0;
  criteria.min_capture_rate = 1.0;
  criteria.max_non_uav_engagements = 0;
  EXPECT_FALSE(MetricsAggregator::meetsCriteria(hover, criteria));
}

TEST(MetricsAggregatorTest, FailsTheCriteriaOnANonUavEngagement)
{
  MetricsAggregator aggregator;
  RunRecord violation = capturedRun();
  violation.result.non_uav_engagements = 1;
  aggregator.add(violation);

  ExitCriteria criteria;
  criteria.min_envelope_rate = 0.0;
  criteria.max_non_uav_engagements = 0;
  EXPECT_FALSE(MetricsAggregator::meetsCriteria(aggregator.summarize("hover"), criteria));
}

TEST(MetricsAggregatorTest, FailsTheCriteriaWithoutAnyRun)
{
  MetricsAggregator aggregator;
  ExitCriteria criteria;
  criteria.min_envelope_rate = 0.0;
  EXPECT_FALSE(MetricsAggregator::meetsCriteria(aggregator.summarize("hover"), criteria));
}

// ---------------------------------------------------------------------------
// The CSV
// ---------------------------------------------------------------------------

TEST(InterceptMetricsCsvTest, RoundTripsARun)
{
  const RunRecord original = capturedRun();
  const std::string text = csvHeader() + "\n" + csvRow(original) + "\n";

  const std::vector<RunRecord> parsed = parseCsv(text);
  ASSERT_EQ(parsed.size(), 1u);
  EXPECT_EQ(parsed[0].scenario, original.scenario);
  EXPECT_EQ(parsed[0].seed, original.seed);
  EXPECT_EQ(parsed[0].result.capture_confirmed, original.result.capture_confirmed);
  EXPECT_NEAR(parsed[0].result.capture_range_m, original.result.capture_range_m, 1e-9);
  EXPECT_NEAR(
    parsed[0].result.capture_relative_speed_mps, original.result.capture_relative_speed_mps, 1e-9);
  EXPECT_EQ(parsed[0].result.envelope_reached, original.result.envelope_reached);
  EXPECT_NEAR(parsed[0].result.closest_approach_m, original.result.closest_approach_m, 1e-9);
  EXPECT_EQ(parsed[0].result.end_phase, original.result.end_phase);
}

TEST(InterceptMetricsCsvTest, KeepsAnAbortReasonWithCommas)
{
  RunRecord record = abortedRun();
  const std::vector<RunRecord> parsed = parseCsv(csvHeader() + "\n" + csvRow(record) + "\n");

  ASSERT_EQ(parsed.size(), 1u);
  EXPECT_TRUE(parsed[0].result.abort_fired);
  EXPECT_EQ(
    parsed[0].result.abort_reason,
    "a person (track 3) is 12.0 m from the capture point");
}

TEST(InterceptMetricsCsvTest, SkipsATruncatedRow)
{
  const std::string text = csvHeader() + "\n" + "hover,1,1,1.2\n";
  EXPECT_TRUE(parseCsv(text).empty());
}

TEST(InterceptMetricsCriteriaTest, AreThePlansNumbers)
{
  EXPECT_DOUBLE_EQ(criteriaFor("hover").min_envelope_rate, 0.9);
  EXPECT_DOUBLE_EQ(criteriaFor("straight").min_envelope_rate, 0.8);
  EXPECT_DOUBLE_EQ(criteriaFor("weave").min_envelope_rate, 0.6);
  EXPECT_DOUBLE_EQ(criteriaFor("bystander").min_abort_rate, 1.0);
  // The bystander run is not expected to capture.
  EXPECT_DOUBLE_EQ(criteriaFor("bystander").min_envelope_rate, 0.0);
  for (const char * name : {"hover", "straight", "weave", "bystander", "other"}) {
    EXPECT_EQ(criteriaFor(name).max_non_uav_engagements, 0) << name;
  }
}
