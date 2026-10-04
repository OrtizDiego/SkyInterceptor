#include "estimation/target_selector.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <vector>

namespace interceptor
{
namespace estimation
{
namespace
{

bool eligible(const TrackEstimate & track, const SelectionRequest & request)
{
  return std::find(
    request.eligible_classes.begin(), request.eligible_classes.end(),
    track.class_id) != request.eligible_classes.end();
}

bool candidate(const TrackEstimate & track, const SelectionRequest & request)
{
  return eligible(track, request) && track.confirmed && track.is_valid;
}

// Ranking key of the automatic selection: the distance from the reference, or
// 0 without one (ties go to the lowest id)
double distance(const TrackEstimate & track, const SelectionRequest & request)
{
  return request.reference ? (track.position - *request.reference).norm() : 0.0;
}

const TrackEstimate * findTrack(const std::vector<TrackEstimate> & tracks, int id)
{
  for (const auto & track : tracks) {
    if (track.id == id) {
      return &track;
    }
  }
  return nullptr;
}

// Closest candidate other than exclude_id
const TrackEstimate * bestCandidate(
  const std::vector<TrackEstimate> & tracks, const SelectionRequest & request, int exclude_id)
{
  const TrackEstimate * best = nullptr;
  double best_distance = std::numeric_limits<double>::infinity();
  for (const auto & track : tracks) {
    if (track.id == exclude_id || !candidate(track, request)) {
      continue;
    }
    const double d = distance(track, request);
    if (!best || d < best_distance || (d == best_distance && track.id < best->id)) {
      best = &track;
      best_distance = d;
    }
  }
  return best;
}

}  // namespace

TargetSelector::TargetSelector(const TargetSelectorParams & params)
: params_(params)
{
  if (!(params_.switch_margin >= 0.0)) {
    throw std::invalid_argument("switch_margin must be non-negative");
  }
}

std::optional<int> TargetSelector::select(
  const std::vector<TrackEstimate> & tracks, const SelectionRequest & request)
{
  if (request.operator_track_id >= 0) {
    const TrackEstimate * track = findTrack(tracks, request.operator_track_id);
    if (track && eligible(*track, request) && track->confirmed) {
      selected_ = track->id;
    } else {
      selected_.reset();
    }
    return selected_;
  }

  const TrackEstimate * current = selected_ ? findTrack(tracks, *selected_) : nullptr;
  if (current && !eligible(*current, request)) {
    current = nullptr;  // e.g. the mission mode changed
  }

  const TrackEstimate * best = bestCandidate(tracks, request, current ? current->id : -1);
  const bool clearly_closer = best && current &&
    distance(*best, request) < distance(*current, request) - params_.switch_margin;
  if (!current) {
    selected_ = best ? std::optional<int>(best->id) : std::nullopt;
  } else if (best && !candidate(*current, request)) {
    selected_ = best->id;  // The selected track is lost, another one is in view
  } else if (clearly_closer) {
    selected_ = best->id;
  } else {
    selected_ = current->id;
  }
  return selected_;
}

}  // namespace estimation
}  // namespace interceptor
