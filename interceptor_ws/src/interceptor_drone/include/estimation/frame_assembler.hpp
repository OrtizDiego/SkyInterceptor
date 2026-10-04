#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include "estimation/track_manager.hpp"

namespace interceptor
{
namespace estimation
{

struct FrameAssemblerParams
{
  // Detections whose stamps are this close belong to the same frame [s]. Well
  // below the sensor period (33 ms at 30 Hz).
  double stamp_tolerance = 0.005;
  // A frame is closed this long after its first detection arrived, unless a
  // detection of a newer frame closes it earlier [s, arrival clock]
  double frame_timeout = 0.01;
};

// All detections taken at one time, the input of TrackManager::processFrame
struct Frame
{
  double stamp = 0.0;
  std::vector<Measurement> detections;
};

// Groups single-detection messages into frames by their sensor stamp.
//
// The tracker must see every detection of a sensor frame in one
// processFrame() call: a detection delivered on its own is a frame in which
// every other track missed, and a second call at the same stamp makes every
// track miss again. Perception publishes one TargetDetection per object, all
// with the image stamp, so the node collects them here first.
//
// A pending frame is complete (returned by pop()) when
//   - a detection of a newer frame has arrived (sources publish in stamp order), or
//   - frame_timeout has passed since its first detection arrived (the last
//     frame before a gap, or a frame with a single detection).
// Frames come out oldest first. A detection for a frame that was already
// returned, or older than one, is dropped as late.
class FrameAssembler
{
public:
  explicit FrameAssembler(const FrameAssemblerParams & params = FrameAssemblerParams());

  // Adds a detection taken at stamp that arrived at arrival (any monotonic
  // clock, seconds). Returns false if it was dropped as late.
  bool add(double stamp, const Measurement & detection, double arrival);

  // Removes and returns the complete frames at arrival time now, oldest first
  std::vector<Frame> pop(double now);

  size_t pending() const {return pending_.size();}
  // Detections dropped as late since construction or clear()
  size_t droppedLate() const {return dropped_late_;}
  // Stamp of the last frame returned by pop()
  std::optional<double> lastStamp() const {return last_stamp_;}
  const FrameAssemblerParams & params() const {return params_;}

  void clear();

private:
  struct PendingFrame
  {
    Frame frame;
    double first_arrival = 0.0;
  };

  FrameAssemblerParams params_;
  std::vector<PendingFrame> pending_;  // Sorted by stamp, oldest first
  std::optional<double> last_stamp_;
  size_t dropped_late_ = 0;
};

}  // namespace estimation
}  // namespace interceptor
