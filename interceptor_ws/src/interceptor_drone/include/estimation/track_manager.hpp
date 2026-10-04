#pragma once

#include <Eigen/Dense>

#include <deque>
#include <optional>
#include <vector>

#include "estimation/imm_filter.hpp"

namespace interceptor
{
namespace estimation
{

// One 3D position detection in the world frame
struct Measurement
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity();
  int class_id = 0;  // TargetClass value
};

struct TrackManagerParams
{
  // Model sets, chosen per class: aerial for UAV, ground for everything else
  ImmParams ground_imm = defaultGroundImmParams();  // CV + CA
  ImmParams aerial_imm = defaultAerialImmParams();  // CV + CT

  double gate_probability = 0.99;      // chi-square gate on the 3D innovation
  // An unassociated detection inside this (wider) gate of a same-class track
  // that got no detection in the frame does not start a new track
  double spawn_gate_probability = 0.9999;
  int confirm_m = 3;                   // Confirm after M hits ...
  int confirm_n = 5;                   // ... in the last N frames
  // A confirmed track is deleted once it has missed this many consecutive
  // frames AND has coasted for longer than coast_timeout, so an occlusion
  // shorter than coast_timeout never deletes it, even while other targets keep
  // producing frames.
  int max_missed_frames = 10;
  // m^2, delete when trace(P_pos) exceeds this. With the default noise a
  // coasting track reaches it after ~4 s, well past coast_timeout.
  double max_position_variance = 50.0;
  double coast_timeout = 2.0;           // s without an update, then is_valid = false
  // The heading follows the velocity only while the horizontal speed is above
  // min_heading_speed AND above heading_speed_sigmas standard deviations of its
  // own estimate; otherwise it holds. Without the second test the velocity
  // noise of a standing target would make the held heading jump around.
  double min_heading_speed = 0.5;       // m/s
  double heading_speed_sigmas = 3.0;
};

// Snapshot of a track at a given time
struct TrackEstimate
{
  int id = -1;
  int class_id = 0;
  bool confirmed = false;  // passed M-of-N
  bool is_valid = false;   // updated within the last coast_timeout

  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
  Eigen::Matrix3d position_cov = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d velocity_cov = Eigen::Matrix3d::Zero();
  double heading = 0.0;    // rad, atan2(vy, vx), held while slower than min_heading_speed
  double turn_rate = 0.0;  // rad/s (CT models only)
  Eigen::VectorXd model_probabilities;

  double quality = 0.0;         // 0..1: hit ratio in the M-of-N window, fading while coasting
  int hits = 0;                 // Updates since birth
  int missed_frames = 0;        // Consecutive frames without an update
  double age = 0.0;             // s since birth
  double time_since_update = 0.0;  // s
};

// Chi-square quantile with 3 degrees of freedom, e.g. 11.345 for p = 0.99
double chiSquareQuantile3Dof(double probability);

// Minimum-cost assignment of every row to a distinct column (Hungarian
// algorithm). cost must have rows <= cols and finite entries; returns the
// column of every row.
std::vector<int> solveAssignment(const Eigen::MatrixXd & cost);

// True if the horizontal velocity is fast and certain enough to define a heading
bool headingObservable(
  const Eigen::Vector3d & velocity, const Eigen::Matrix3d & velocity_cov,
  const TrackManagerParams & params);

class Track
{
public:
  Track(int id, const Measurement & detection, double stamp, const ImmParams & imm, int window);

  int id() const {return id_;}
  int classId() const {return class_id_;}
  bool confirmed() const {return confirmed_;}
  const ImmFilter & filter() const {return filter_;}

  // IMM cycle step 1: predict to the frame time
  void predict(double stamp);
  // IMM cycle step 2: either an associated detection or a miss
  void update(const Measurement & detection);
  void miss();

  int hitsInWindow() const;
  int missesInWindow() const;
  int missedFrames() const {return missed_frames_;}
  int hits() const {return hits_;}
  double lastUpdateStamp() const {return last_update_stamp_;}

  void confirm() {confirmed_ = true;}
  // Re-evaluates the held heading from the filtered velocity
  void refreshHeading(const TrackManagerParams & params);

  TrackEstimate estimate(double stamp, const TrackManagerParams & params) const;

private:
  void record(bool hit);

  int id_;
  int class_id_;
  ImmFilter filter_;
  double birth_stamp_;
  double filter_stamp_;
  double last_update_stamp_;
  std::deque<bool> window_;  // Hit/miss of the last N frames
  size_t window_size_;
  int hits_ = 1;
  int missed_frames_ = 0;
  bool confirmed_ = false;
  double heading_ = 0.0;
};

// Multi-target tracker: one IMM filter per track, chi-square gating, global
// nearest-neighbour association (confirmed tracks first), M-of-N confirmation
// and coasting.
//
// Feed it one call per sensor frame (all detections taken at the same time);
// a frame with no detections still counts as a miss for every track. Times are
// in seconds on any monotonic clock.
class TrackManager
{
public:
  explicit TrackManager(const TrackManagerParams & params = TrackManagerParams());

  // Predicts all tracks to the frame time, associates the detections, updates,
  // spawns tentative tracks from unassociated detections and deletes dead
  // tracks. Detections only associate with tracks of the same class.
  // If assignments is given it receives the track id of every detection (-1 if
  // it was dropped as a likely duplicate of a track, see spawn_gate_probability).
  // Frames older than the last one are dropped (returns false).
  bool processFrame(
    double stamp, const std::vector<Measurement> & detections,
    std::vector<int> * assignments = nullptr);

  // Deletes tracks whose position covariance, extrapolated to stamp, has grown
  // beyond max_position_variance. Call it while no frames arrive.
  void prune(double stamp);

  // All tracks (tentative and confirmed) extrapolated to stamp
  std::vector<TrackEstimate> tracks(double stamp) const;
  std::optional<TrackEstimate> track(int id, double stamp) const;

  size_t size() const {return tracks_.size();}
  double gateThreshold() const {return gate_threshold_;}
  const TrackManagerParams & params() const {return params_;}
  void clear();

private:
  const ImmParams & immFor(int class_id) const;

  TrackManagerParams params_;
  double gate_threshold_;
  double spawn_gate_threshold_;
  std::vector<Track> tracks_;
  int next_id_ = 1;
  std::optional<double> last_frame_stamp_;
};

}  // namespace estimation
}  // namespace interceptor
