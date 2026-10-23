/**
 * @file intercept_metrics_report.cpp
 * @brief Reads the CSV of a batch of INTERCEPT runs and scores it against the
 *        plan's exit criteria (section 5, phase 3).
 *
 *     intercept_metrics_report <runs.csv> [<summary.csv>]
 *
 * Prints one line per scenario (the rates and the verdict) and, when a second
 * path is given, writes the per-scenario summary as CSV. The exit code is 0
 * when every scenario that has runs meets its criteria.
 *
 * The aggregation and the criteria are the ROS-free library's
 * (include/metrics/intercept_metrics.hpp), so the batch is scored by the same
 * code the unit tests check.
 */

#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "metrics/intercept_metrics.hpp"

int main(int argc, char ** argv)
{
  using interceptor::metrics::MetricsAggregator;
  using interceptor::metrics::ScenarioSummary;

  if (argc < 2) {
    std::cerr << "usage: intercept_metrics_report <runs.csv> [<summary.csv>]\n";
    return 2;
  }
  std::ifstream input(argv[1]);
  if (!input.is_open()) {
    std::cerr << "cannot read " << argv[1] << "\n";
    return 2;
  }
  std::stringstream text;
  text << input.rdbuf();

  MetricsAggregator aggregator;
  for (const auto & record : interceptor::metrics::parseCsv(text.str())) {
    aggregator.add(record);
  }
  const std::vector<ScenarioSummary> summaries = aggregator.summarize();

  bool all_met = !summaries.empty();
  std::printf("=== INTERCEPT batch: %d runs ===\n", aggregator.runs());
  for (const ScenarioSummary & summary : summaries) {
    const auto criteria = interceptor::metrics::criteriaFor(summary.scenario);
    const bool met = MetricsAggregator::meetsCriteria(summary, criteria);
    all_met = all_met && met;
    std::printf(
      "%-10s runs %3d | envelope %3d (%5.1f %%, need %3.0f %%) | capture %3d (%5.1f %%) | "
      "abort %3d (%5.1f %%, need %3.0f %%) | non-uav %d | capture range %.2f m, "
      "worst rel. speed %.2f m/s, closest %.2f m, %.1f s | %s\n",
      summary.scenario.c_str(), summary.runs,
      summary.envelopes, 100.0 * summary.envelope_rate, 100.0 * criteria.min_envelope_rate,
      summary.captures, 100.0 * summary.capture_rate,
      summary.aborts, 100.0 * summary.abort_rate, 100.0 * criteria.min_abort_rate,
      summary.non_uav_samples, summary.median_capture_range_m,
      summary.worst_capture_relative_speed_mps, summary.median_closest_approach_m,
      summary.median_duration_s, met ? "PASS" : "FAIL");
  }

  if (argc > 2) {
    std::ofstream output(argv[2]);
    output << interceptor::metrics::summaryCsv(summaries);
  }
  return all_met ? 0 : 1;
}
