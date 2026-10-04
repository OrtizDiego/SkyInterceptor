#include "estimation/track_manager.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "common/mission.hpp"

namespace interceptor
{
namespace estimation
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

// Assignment costs: outside the gate, and a track left without a detection
constexpr double kForbidden = 1e9;
constexpr double kMissCost = 1e6;

// CDF of the chi-square distribution with 3 degrees of freedom
double chiSquareCdf3Dof(double x)
{
  if (x <= 0.0) {
    return 0.0;
  }
  return std::erf(std::sqrt(0.5 * x)) - std::sqrt(2.0 * x / kPi) * std::exp(-0.5 * x);
}

}  // namespace

double chiSquareQuantile3Dof(double probability)
{
  if (!(probability > 0.0 && probability < 1.0)) {
    throw std::invalid_argument("chi-square quantile needs a probability in (0, 1)");
  }
  // The CDF is monotonic: bisect. P(chi2_3 > 100) ~ 1e-20, far beyond any gate.
  double lo = 0.0;
  double hi = 100.0;
  for (int i = 0; i < 100; ++i) {
    const double mid = 0.5 * (lo + hi);
    if (chiSquareCdf3Dof(mid) < probability) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  return 0.5 * (lo + hi);
}

std::vector<int> solveAssignment(const Eigen::MatrixXd & cost)
{
  // Hungarian algorithm with potentials (Kuhn-Munkres, O(n^2 m)), 1-based
  const int n = static_cast<int>(cost.rows());
  const int m = static_cast<int>(cost.cols());
  if (n > m) {
    throw std::invalid_argument("solveAssignment needs rows <= cols");
  }
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<double> u(static_cast<size_t>(n) + 1, 0.0);
  std::vector<double> v(static_cast<size_t>(m) + 1, 0.0);
  std::vector<int> p(static_cast<size_t>(m) + 1, 0);    // row matched to column j
  std::vector<int> way(static_cast<size_t>(m) + 1, 0);
  const auto at = [](auto & vec, int i) -> auto & {return vec[static_cast<size_t>(i)];};

  for (int i = 1; i <= n; ++i) {
    at(p, 0) = i;
    int j0 = 0;
    std::vector<double> minv(static_cast<size_t>(m) + 1, inf);
    std::vector<char> used(static_cast<size_t>(m) + 1, 0);
    do {
      at(used, j0) = 1;
      const int i0 = at(p, j0);
      double delta = inf;
      int j1 = 0;
      for (int j = 1; j <= m; ++j) {
        if (at(used, j)) {
          continue;
        }
        const double cur = cost(i0 - 1, j - 1) - at(u, i0) - at(v, j);
        if (cur < at(minv, j)) {
          at(minv, j) = cur;
          at(way, j) = j0;
        }
        if (at(minv, j) < delta) {
          delta = at(minv, j);
          j1 = j;
        }
      }
      for (int j = 0; j <= m; ++j) {
        if (at(used, j)) {
          at(u, at(p, j)) += delta;
          at(v, j) -= delta;
        } else {
          at(minv, j) -= delta;
        }
      }
      j0 = j1;
    } while (at(p, j0) != 0);
    do {
      const int j1 = at(way, j0);
      at(p, j0) = at(p, j1);
      j0 = j1;
    } while (j0 != 0);
  }

  std::vector<int> col_of_row(static_cast<size_t>(n), -1);
  for (int j = 1; j <= m; ++j) {
    if (at(p, j) != 0) {
      at(col_of_row, at(p, j) - 1) = j - 1;
    }
  }
  return col_of_row;
}

bool headingObservable(
  const Eigen::Vector3d & velocity, const Eigen::Matrix3d & velocity_cov,
  const TrackManagerParams & params)
{
  const double speed = velocity.head<2>().norm();
  if (speed <= params.min_heading_speed) {
    return false;
  }
  // Variance of the speed: velocity covariance projected on the velocity direction
  const Eigen::Vector2d dir = velocity.head<2>() / speed;
  const double speed_var = dir.dot(velocity_cov.topLeftCorner<2, 2>() * dir);
  return speed > params.heading_speed_sigmas * std::sqrt(std::max(0.0, speed_var));
}

// --- Track ----------------------------------------------------------------------

Track::Track(
  int id, const Measurement & detection, double stamp, const ImmParams & imm, int window)
: id_(id),
  class_id_(detection.class_id),
  filter_(imm),
  birth_stamp_(stamp),
  filter_stamp_(stamp),
  last_update_stamp_(stamp),
  window_size_(static_cast<size_t>(window))
{
  filter_.initialize(detection.position, detection.covariance);
  window_.push_back(true);
}

void Track::predict(double stamp)
{
  filter_.predict(std::max(0.0, stamp - filter_stamp_));
  filter_stamp_ = std::max(filter_stamp_, stamp);
}

void Track::update(const Measurement & detection)
{
  if (!filter_.update(detection.position, detection.covariance)) {
    miss();
    return;
  }
  record(true);
  ++hits_;
  missed_frames_ = 0;
  last_update_stamp_ = filter_stamp_;
}

void Track::miss()
{
  record(false);
  ++missed_frames_;
}

void Track::record(bool hit)
{
  window_.push_back(hit);
  while (window_.size() > window_size_) {
    window_.pop_front();
  }
}

int Track::hitsInWindow() const
{
  return static_cast<int>(std::count(window_.begin(), window_.end(), true));
}

int Track::missesInWindow() const
{
  return static_cast<int>(window_.size()) - hitsInWindow();
}

void Track::refreshHeading(const TrackManagerParams & params)
{
  const ImmEstimate & e = filter_.estimate();
  const Eigen::Vector3d v = e.velocity();
  if (headingObservable(v, e.velocityCovariance(), params)) {
    heading_ = std::atan2(v.y(), v.x());
  }
}

TrackEstimate Track::estimate(double stamp, const TrackManagerParams & params) const
{
  const ImmEstimate e = filter_.extrapolate(stamp - filter_stamp_);

  TrackEstimate out;
  out.id = id_;
  out.class_id = class_id_;
  out.confirmed = confirmed_;
  out.position = e.position();
  out.velocity = e.velocity();
  out.acceleration = e.acceleration;
  out.position_cov = e.positionCovariance();
  out.velocity_cov = e.velocityCovariance();
  out.turn_rate = e.turnRate();
  out.model_probabilities = e.model_probabilities;
  out.heading = headingObservable(out.velocity, out.velocity_cov, params) ?
    std::atan2(out.velocity.y(), out.velocity.x()) : heading_;

  out.hits = hits_;
  out.missed_frames = missed_frames_;
  out.age = std::max(0.0, stamp - birth_stamp_);
  out.time_since_update = std::max(0.0, stamp - last_update_stamp_);
  out.is_valid = out.time_since_update <= params.coast_timeout;

  const double hit_ratio =
    static_cast<double>(hitsInWindow()) / static_cast<double>(window_size_);
  const double freshness = params.coast_timeout > 0.0 ?
    std::clamp(1.0 - out.time_since_update / params.coast_timeout, 0.0, 1.0) :
    (out.time_since_update > 0.0 ? 0.0 : 1.0);
  out.quality = hit_ratio * freshness;
  return out;
}

// --- TrackManager ----------------------------------------------------------------

TrackManager::TrackManager(const TrackManagerParams & params)
: params_(params),
  gate_threshold_(chiSquareQuantile3Dof(params.gate_probability)),
  spawn_gate_threshold_(chiSquareQuantile3Dof(params.spawn_gate_probability))
{
  if (params_.spawn_gate_probability < params_.gate_probability) {
    throw std::invalid_argument("spawn_gate_probability must be >= gate_probability");
  }
  validate(params_.ground_imm);
  validate(params_.aerial_imm);
  if (params_.confirm_n < 1 || params_.confirm_m < 1 || params_.confirm_m > params_.confirm_n) {
    throw std::invalid_argument("track confirmation needs 1 <= confirm_m <= confirm_n");
  }
  if (params_.max_missed_frames < 1) {
    throw std::invalid_argument("max_missed_frames must be at least 1");
  }
  if (params_.max_position_variance <= 0.0 || params_.coast_timeout < 0.0 ||
    params_.min_heading_speed < 0.0 || params_.heading_speed_sigmas < 0.0)
  {
    throw std::invalid_argument(
            "max_position_variance must be positive, coast_timeout, min_heading_speed and "
            "heading_speed_sigmas non-negative");
  }
}

const ImmParams & TrackManager::immFor(int class_id) const
{
  return class_id == static_cast<int>(TargetClass::UAV) ? params_.aerial_imm : params_.ground_imm;
}

bool TrackManager::processFrame(
  double stamp, const std::vector<Measurement> & detections, std::vector<int> * assignments)
{
  if (last_frame_stamp_ && stamp < *last_frame_stamp_) {
    return false;
  }
  last_frame_stamp_ = stamp;

  for (auto & track : tracks_) {
    track.predict(stamp);
  }

  // Gate every same-class (track, detection) pair; the cost of a pair is the
  // normalised distance d^2 + log det S, which penalises uncertain tracks.
  const auto n_tracks = static_cast<Eigen::Index>(tracks_.size());
  const auto n_detections = static_cast<Eigen::Index>(detections.size());
  Eigen::MatrixXd cost = Eigen::MatrixXd::Constant(n_tracks, n_detections, kForbidden);
  Eigen::MatrixXd mahalanobis2 = Eigen::MatrixXd::Constant(
    n_tracks, n_detections, std::numeric_limits<double>::infinity());
  for (Eigen::Index t = 0; t < n_tracks; ++t) {
    const Track & track = tracks_[static_cast<size_t>(t)];
    for (Eigen::Index d = 0; d < n_detections; ++d) {
      const Measurement & detection = detections[static_cast<size_t>(d)];
      if (detection.class_id != track.classId()) {
        continue;
      }
      double normalized = 0.0;
      if (track.filter().gatingDistance(
          detection.position, detection.covariance, mahalanobis2(t, d), normalized) &&
        mahalanobis2(t, d) <= gate_threshold_)
      {
        cost(t, d) = normalized;
      }
    }
  }

  // Global nearest neighbour, as a cascade: confirmed tracks first, tentative
  // ones get the detections that are left. The gate rejects 1 - gate_probability
  // of true detections, and the tentative twin such a miss spawns would
  // otherwise steal detections from the confirmed track; starved, it dies
  // within confirm_n frames.
  std::vector<int> detection_of_track(tracks_.size(), -1);
  std::vector<int> track_of_detection(detections.size(), -1);
  for (const bool confirmed : {true, false}) {
    std::vector<size_t> rows;
    for (size_t t = 0; t < tracks_.size(); ++t) {
      if (tracks_[t].confirmed() == confirmed) {
        rows.push_back(t);
      }
    }
    std::vector<size_t> cols;
    for (size_t d = 0; d < detections.size(); ++d) {
      if (track_of_detection[d] < 0) {
        cols.push_back(d);
      }
    }
    if (rows.empty() || cols.empty()) {
      continue;
    }
    // One extra "miss" column per track, so every track can stay unassigned.
    // The miss cost exceeds any gated pair: the assignment maximises the number
    // of associations first, then minimises the total cost.
    const auto n_rows = static_cast<Eigen::Index>(rows.size());
    const auto n_cols = static_cast<Eigen::Index>(cols.size());
    Eigen::MatrixXd sub = Eigen::MatrixXd::Constant(n_rows, n_cols + n_rows, kForbidden);
    for (Eigen::Index r = 0; r < n_rows; ++r) {
      for (Eigen::Index c = 0; c < n_cols; ++c) {
        sub(r, c) = cost(
          static_cast<Eigen::Index>(rows[static_cast<size_t>(r)]),
          static_cast<Eigen::Index>(cols[static_cast<size_t>(c)]));
      }
      sub(r, n_cols + r) = kMissCost;
    }
    const std::vector<int> col_of_row = solveAssignment(sub);
    for (Eigen::Index r = 0; r < n_rows; ++r) {
      const int c = col_of_row[static_cast<size_t>(r)];
      if (c < 0 || c >= n_cols || sub(r, c) >= kMissCost) {
        continue;  // miss
      }
      const size_t t = rows[static_cast<size_t>(r)];
      const size_t d = cols[static_cast<size_t>(c)];
      detection_of_track[t] = static_cast<int>(d);
      track_of_detection[d] = static_cast<int>(t);
    }
  }

  if (assignments) {
    assignments->assign(detections.size(), -1);
  }
  for (size_t t = 0; t < tracks_.size(); ++t) {
    const int d = detection_of_track[t];
    if (d >= 0) {
      tracks_[t].update(detections[static_cast<size_t>(d)]);
      if (assignments) {
        (*assignments)[static_cast<size_t>(d)] = tracks_[t].id();
      }
    } else {
      tracks_[t].miss();
    }
  }

  // Unassociated detections start tentative tracks, unless they are within the
  // wider spawn gate of a same-class track that got no detection this frame:
  // then it is most likely that track's own detection in the tail of the gate,
  // and a new track would only be a duplicate of it.
  for (size_t d = 0; d < detections.size(); ++d) {
    if (track_of_detection[d] >= 0) {
      continue;
    }
    bool duplicate = false;
    for (Eigen::Index t = 0; t < n_tracks && !duplicate; ++t) {
      duplicate = detection_of_track[static_cast<size_t>(t)] < 0 &&
        mahalanobis2(t, static_cast<Eigen::Index>(d)) <= spawn_gate_threshold_;
    }
    if (duplicate) {
      continue;
    }
    tracks_.emplace_back(
      next_id_++, detections[d], stamp, immFor(detections[d].class_id), params_.confirm_n);
    if (assignments) {
      (*assignments)[d] = tracks_.back().id();
    }
  }

  // M-of-N confirmation; tentative tracks die once M hits are out of reach,
  // confirmed ones after max_missed_frames misses and coast_timeout
  const int max_tentative_misses = params_.confirm_n - params_.confirm_m;
  tracks_.erase(
    std::remove_if(
      tracks_.begin(), tracks_.end(),
      [&](Track & track) {
        if (!track.confirmed()) {
          if (track.hitsInWindow() >= params_.confirm_m) {
            track.confirm();
          } else if (track.missesInWindow() > max_tentative_misses) {
            return true;
          }
        } else {
          const bool lost = track.missedFrames() >= params_.max_missed_frames &&
          stamp - track.lastUpdateStamp() > params_.coast_timeout;
          if (lost) {
            return true;
          }
        }
        track.refreshHeading(params_);
        return false;
      }),
    tracks_.end());

  prune(stamp);
  return true;
}

void TrackManager::prune(double stamp)
{
  tracks_.erase(
    std::remove_if(
      tracks_.begin(), tracks_.end(),
      [&](const Track & track) {
        return track.estimate(stamp, params_).position_cov.trace() >
        params_.max_position_variance;
      }),
    tracks_.end());
}

std::vector<TrackEstimate> TrackManager::tracks(double stamp) const
{
  std::vector<TrackEstimate> out;
  out.reserve(tracks_.size());
  for (const auto & track : tracks_) {
    out.push_back(track.estimate(stamp, params_));
  }
  return out;
}

std::optional<TrackEstimate> TrackManager::track(int id, double stamp) const
{
  for (const auto & track : tracks_) {
    if (track.id() == id) {
      return track.estimate(stamp, params_);
    }
  }
  return std::nullopt;
}

void TrackManager::clear()
{
  tracks_.clear();
  last_frame_stamp_.reset();
}

}  // namespace estimation
}  // namespace interceptor
