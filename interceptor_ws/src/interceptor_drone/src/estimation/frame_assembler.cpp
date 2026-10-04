#include "estimation/frame_assembler.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <stdexcept>
#include <utility>
#include <vector>

namespace interceptor
{
namespace estimation
{

FrameAssembler::FrameAssembler(const FrameAssemblerParams & params)
: params_(params)
{
  if (!(params_.stamp_tolerance >= 0.0) || !(params_.frame_timeout >= 0.0)) {
    throw std::invalid_argument("stamp_tolerance and frame_timeout must be non-negative");
  }
}

bool FrameAssembler::add(double stamp, const Measurement & detection, double arrival)
{
  if (last_stamp_ && stamp - *last_stamp_ <= params_.stamp_tolerance) {
    ++dropped_late_;
    return false;
  }

  // Join the pending frame with the closest stamp within the tolerance. Pending
  // frames are more than stamp_tolerance apart, so there is at most one.
  for (auto & pending : pending_) {
    if (std::abs(pending.frame.stamp - stamp) <= params_.stamp_tolerance) {
      pending.frame.detections.push_back(detection);
      return true;
    }
  }

  PendingFrame pending;
  pending.frame.stamp = stamp;
  pending.frame.detections.push_back(detection);
  pending.first_arrival = arrival;
  const auto position = std::upper_bound(
    pending_.begin(), pending_.end(), stamp,
    [](double value, const PendingFrame & frame) {return value < frame.frame.stamp;});
  pending_.insert(position, std::move(pending));
  return true;
}

std::vector<Frame> FrameAssembler::pop(double now)
{
  // Every frame but the newest has seen a detection of a newer frame. The
  // newest one waits for frame_timeout.
  size_t complete = pending_.empty() ? 0 : pending_.size() - 1;
  if (!pending_.empty() && now - pending_.back().first_arrival >= params_.frame_timeout) {
    complete = pending_.size();
  }

  std::vector<Frame> frames;
  frames.reserve(complete);
  for (size_t i = 0; i < complete; ++i) {
    frames.push_back(std::move(pending_[i].frame));
  }
  pending_.erase(
    pending_.begin(), std::next(pending_.begin(), static_cast<std::ptrdiff_t>(complete)));
  if (!frames.empty()) {
    last_stamp_ = frames.back().stamp;
  }
  return frames;
}

void FrameAssembler::clear()
{
  pending_.clear();
  last_stamp_.reset();
  dropped_late_ = 0;
}

}  // namespace estimation
}  // namespace interceptor
