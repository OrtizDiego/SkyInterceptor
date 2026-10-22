#include "metrics/intercept_metrics.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace interceptor
{
namespace metrics
{

namespace
{

/// The value a number that was never measured is written as.
constexpr double kNoValue = 0.0;

/// Escapes a field for the CSV: a comma, a quote or a newline makes the field
/// quoted, and a quote inside it is doubled.
std::string csvField(const std::string & value)
{
  const bool needs_quotes =
    value.find_first_of(",\"\n\r") != std::string::npos;
  if (!needs_quotes) {
    return value;
  }
  std::string out = "\"";
  for (const char c : value) {
    if (c == '"') {
      out += "\"\"";
    } else {
      out += c;
    }
  }
  out += "\"";
  return out;
}

/// Splits a CSV line into its fields, honouring the quoting csvField writes.
std::vector<std::string> splitCsvLine(const std::string & line)
{
  std::vector<std::string> fields;
  std::string current;
  bool in_quotes = false;
  for (std::size_t i = 0; i < line.size(); ++i) {
    const char c = line[i];
    if (in_quotes) {
      if (c == '"') {
        if (i + 1 < line.size() && line[i + 1] == '"') {
          current += '"';
          ++i;
        } else {
          in_quotes = false;
        }
      } else {
        current += c;
      }
    } else if (c == '"') {
      in_quotes = true;
    } else if (c == ',') {
      fields.push_back(current);
      current.clear();
    } else {
      current += c;
    }
  }
  fields.push_back(current);
  return fields;
}

/// The median of a list of values. The list is copied and sorted.
double medianOf(std::vector<double> values)
{
  if (values.empty()) {
    return kNoValue;
  }
  std::sort(values.begin(), values.end());
  const std::size_t middle = values.size() / 2;
  if (values.size() % 2 == 1) {
    return values[middle];
  }
  return 0.5 * (values[middle - 1] + values[middle]);
}

/// A number from a CSV field, or the fallback when it does not parse.
double numberOr(const std::string & field, double fallback)
{
  try {
    std::size_t used = 0;
    const double value = std::stod(field, &used);
    if (used == 0) {
      return fallback;
    }
    return value;
  } catch (const std::exception &) {
    return fallback;
  }
}

/// A flag from a CSV field: "1" and "true" are true.
bool flagOr(const std::string & field, bool fallback)
{
  if (field == "1" || field == "true") {
    return true;
  }
  if (field == "0" || field == "false") {
    return false;
  }
  return fallback;
}

}  // namespace

EngagementScorer::EngagementScorer(MetricsParams params)
: params_(params)
{
  if (!(params_.capture_radius_m > 0.0)) {
    throw std::invalid_argument("capture_radius_m must be positive");
  }
  if (params_.capture_max_rel_speed_mps < 0.0) {
    throw std::invalid_argument("capture_max_rel_speed_mps must not be negative");
  }
}

void EngagementScorer::addSample(const EngagementSample & sample)
{
  if (!have_sample_) {
    have_sample_ = true;
    result_.closest_approach_m = sample.range_m;
    result_.min_miss_distance_m = sample.miss_distance_m;
  } else {
    result_.closest_approach_m = std::min(result_.closest_approach_m, sample.range_m);
    result_.min_miss_distance_m = std::min(result_.min_miss_distance_m, sample.miss_distance_m);
  }
  ++result_.sample_count;
  result_.duration_s = sample.time_s;

  // The envelope: the range was inside the capture radius, whatever the
  // relative speed was.
  if (sample.range_m <= params_.capture_radius_m) {
    result_.envelope_reached = true;
  }

  // The capture: the guidance's own condition, on the same two inputs.
  if (!result_.capture_confirmed &&
    sample.range_m <= params_.capture_radius_m &&
    sample.relative_speed_mps <= params_.capture_max_rel_speed_mps)
  {
    result_.capture_confirmed = true;
    result_.capture_range_m = sample.range_m;
    result_.capture_relative_speed_mps = sample.relative_speed_mps;
    result_.capture_time_s = sample.time_s;
    result_.capture_target_speed_mps = sample.target_speed_mps;
  }

  // The abort latches, so the first sample that reports it carries the reason.
  if (sample.abort_active && !result_.abort_fired) {
    result_.abort_fired = true;
    result_.abort_reason = sample.abort_reason;
  }

  // A non-uav track that is flown at is a violation of the class rule.
  if (sample.guidance_engaging && sample.target_class_id >= 0 &&
    sample.target_class_id != params_.uav_class_id)
  {
    ++result_.non_uav_engagements;
  }
}

void MetricsAggregator::add(const RunRecord & record)
{
  records_.push_back(record);
}

ScenarioSummary MetricsAggregator::summarize(const std::string & scenario) const
{
  ScenarioSummary summary;
  summary.scenario = scenario;

  std::vector<double> capture_ranges;
  std::vector<double> closest_approaches;
  std::vector<double> durations;

  for (const RunRecord & record : records_) {
    if (record.scenario != scenario) {
      continue;
    }
    ++summary.runs;
    const EngagementResult & result = record.result;
    if (result.capture_confirmed) {
      ++summary.captures;
      capture_ranges.push_back(result.capture_range_m);
      summary.worst_capture_relative_speed_mps = std::max(
        summary.worst_capture_relative_speed_mps, result.capture_relative_speed_mps);
    }
    if (result.envelope_reached) {
      ++summary.envelopes;
    }
    if (result.abort_fired) {
      ++summary.aborts;
    }
    if (result.non_uav_engagements > 0) {
      ++summary.non_uav_runs;
    }
    summary.non_uav_samples += result.non_uav_engagements;
    closest_approaches.push_back(result.closest_approach_m);
    durations.push_back(result.duration_s);
  }

  if (summary.runs > 0) {
    summary.capture_rate = static_cast<double>(summary.captures) / summary.runs;
    summary.envelope_rate = static_cast<double>(summary.envelopes) / summary.runs;
    summary.abort_rate = static_cast<double>(summary.aborts) / summary.runs;
  }
  summary.median_capture_range_m = medianOf(capture_ranges);
  summary.median_closest_approach_m = medianOf(closest_approaches);
  summary.median_duration_s = medianOf(durations);
  return summary;
}

std::vector<ScenarioSummary> MetricsAggregator::summarize() const
{
  // The scenarios in the order they were added, so the report is stable.
  std::vector<std::string> scenarios;
  for (const RunRecord & record : records_) {
    if (std::find(scenarios.begin(), scenarios.end(), record.scenario) == scenarios.end()) {
      scenarios.push_back(record.scenario);
    }
  }

  std::vector<ScenarioSummary> summaries;
  summaries.reserve(scenarios.size());
  for (const std::string & scenario : scenarios) {
    summaries.push_back(summarize(scenario));
  }
  return summaries;
}

bool MetricsAggregator::meetsCriteria(
  const ScenarioSummary & summary, const ExitCriteria & criteria)
{
  if (summary.runs == 0) {
    return false;
  }
  if (summary.envelope_rate < criteria.min_envelope_rate) {
    return false;
  }
  if (summary.capture_rate < criteria.min_capture_rate) {
    return false;
  }
  if (summary.abort_rate < criteria.min_abort_rate) {
    return false;
  }
  return summary.non_uav_samples <= criteria.max_non_uav_engagements;
}

std::string csvHeader()
{
  return "scenario,seed,capture_confirmed,capture_range_m,capture_relative_speed_mps,"
         "capture_target_speed_mps,envelope_reached,closest_approach_m,min_miss_distance_m,"
         "abort_fired,abort_reason,non_uav_engagements,end_phase,duration_s,sample_count";
}

std::string csvRow(const RunRecord & record)
{
  const EngagementResult & result = record.result;
  std::ostringstream out;
  out << csvField(record.scenario) << ',' << record.seed << ','
      << (result.capture_confirmed ? 1 : 0) << ','
      << result.capture_range_m << ','
      << result.capture_relative_speed_mps << ','
      << result.capture_target_speed_mps << ','
      << (result.envelope_reached ? 1 : 0) << ','
      << result.closest_approach_m << ','
      << result.min_miss_distance_m << ','
      << (result.abort_fired ? 1 : 0) << ','
      << csvField(result.abort_reason) << ','
      << result.non_uav_engagements << ','
      << csvField(result.end_phase) << ','
      << result.duration_s << ','
      << result.sample_count;
  return out.str();
}

std::vector<RunRecord> parseCsv(const std::string & text)
{
  std::vector<RunRecord> records;
  std::istringstream stream(text);
  std::string line;
  bool first = true;
  while (std::getline(stream, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (line.empty()) {
      continue;
    }
    if (first) {
      // The header: the column order is the one csvHeader writes.
      first = false;
      continue;
    }
    const std::vector<std::string> fields = splitCsvLine(line);
    if (fields.size() < 15) {
      continue;  // a truncated row: skip it
    }
    RunRecord record;
    record.scenario = fields[0];
    record.seed = static_cast<int>(numberOr(fields[1], 0.0));
    EngagementResult & result = record.result;
    result.capture_confirmed = flagOr(fields[2], false);
    result.capture_range_m = numberOr(fields[3], 0.0);
    result.capture_relative_speed_mps = numberOr(fields[4], 0.0);
    result.capture_target_speed_mps = numberOr(fields[5], 0.0);
    result.envelope_reached = flagOr(fields[6], false);
    result.closest_approach_m = numberOr(fields[7], 0.0);
    result.min_miss_distance_m = numberOr(fields[8], 0.0);
    result.abort_fired = flagOr(fields[9], false);
    result.abort_reason = fields[10];
    result.non_uav_engagements = static_cast<int>(numberOr(fields[11], 0.0));
    result.end_phase = fields[12];
    result.duration_s = numberOr(fields[13], 0.0);
    result.sample_count = static_cast<int>(numberOr(fields[14], 0.0));
    records.push_back(record);
  }
  return records;
}

std::string summaryCsv(const std::vector<ScenarioSummary> & summaries)
{
  std::ostringstream out;
  out << "scenario,runs,captures,capture_rate,envelopes,envelope_rate,aborts,abort_rate,"
    "non_uav_runs,non_uav_samples,median_capture_range_m,"
    "worst_capture_relative_speed_mps,median_closest_approach_m,median_duration_s\n";
  for (const ScenarioSummary & summary : summaries) {
    out << csvField(summary.scenario) << ','
        << summary.runs << ','
        << summary.captures << ','
        << summary.capture_rate << ','
        << summary.envelopes << ','
        << summary.envelope_rate << ','
        << summary.aborts << ','
        << summary.abort_rate << ','
        << summary.non_uav_runs << ','
        << summary.non_uav_samples << ','
        << summary.median_capture_range_m << ','
        << summary.worst_capture_relative_speed_mps << ','
        << summary.median_closest_approach_m << ','
        << summary.median_duration_s << '\n';
  }
  return out.str();
}

}  // namespace metrics
}  // namespace interceptor
