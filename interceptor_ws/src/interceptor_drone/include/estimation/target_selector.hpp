#pragma once

#include <Eigen/Dense>

#include <optional>
#include <vector>

#include "estimation/track_manager.hpp"

namespace interceptor
{
namespace estimation
{

struct TargetSelectorParams
{
  // Automatic selection only moves to another track that is this much closer
  // than the selected one [m], so it doesn't flip between two targets at a
  // similar distance
  double switch_margin = 3.0;
};

struct SelectionRequest
{
  // Classes of the current mission mode (TargetClass values). Empty: nothing
  // can be selected (HOLD, or the mode is not known yet).
  std::vector<int> eligible_classes;
  // Operator-selected track, -1 for automatic selection
  int operator_track_id = -1;
  // Position the automatic selection measures distances from (the drone). If
  // unknown, the selection prefers the oldest track (lowest id).
  std::optional<Eigen::Vector3d> reference;
};

// Picks the track the planners work on from the tracker output.
//
// Only tracks of an eligible class can be selected, whatever else is asked.
// - Operator mode (operator_track_id >= 0): that track if it exists, is
//   eligible and confirmed; otherwise nothing. It never falls back to another
//   track.
// - Automatic mode: candidates are confirmed, valid (not coasting past
//   coast_timeout) eligible tracks. The selected track is kept while it is a
//   candidate unless another candidate is closer by more than switch_margin.
//   Once it coasts past coast_timeout the closest candidate takes over; with no
//   candidate it stays selected (and is_valid = false tells the planners it is
//   lost) until the track is deleted.
class TargetSelector
{
public:
  explicit TargetSelector(const TargetSelectorParams & params = TargetSelectorParams());

  // Updates and returns the selected track id
  std::optional<int> select(
    const std::vector<TrackEstimate> & tracks, const SelectionRequest & request);

  std::optional<int> selected() const {return selected_;}
  const TargetSelectorParams & params() const {return params_;}
  void reset() {selected_.reset();}

private:
  TargetSelectorParams params_;
  std::optional<int> selected_;
};

}  // namespace estimation
}  // namespace interceptor
