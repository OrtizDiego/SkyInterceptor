#include <gtest/gtest.h>
#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <set>
#include <stdexcept>
#include <vector>

#include "common/mission.hpp"
#include "estimation/track_manager.hpp"
#include "synthetic_trajectory.hpp"

namespace interceptor
{
namespace estimation
{
namespace
{

constexpr double kRate = 30.0;  // Hz, ground-truth detection rate
constexpr double kSigma = 0.3;  // m, measurement noise (plan §5 Phase 1 exit criterion)
constexpr int kPerson = static_cast<int>(TargetClass::PERSON);
constexpr int kCar = static_cast<int>(TargetClass::CAR);
constexpr int kUav = static_cast<int>(TargetClass::UAV);

Measurement detection(const Eigen::Vector3d & p, int class_id = kPerson)
{
  Measurement m;
  m.position = p;
  m.covariance = Eigen::Matrix3d::Identity() * kSigma * kSigma;
  m.class_id = class_id;
  return m;
}

std::vector<TrackEstimate> confirmedTracks(const TrackManager & tracker, double t)
{
  std::vector<TrackEstimate> out;
  for (const auto & track : tracker.tracks(t)) {
    if (track.confirmed) {
      out.push_back(track);
    }
  }
  return out;
}

struct SingleTargetRun
{
  double rmse = 0.0;
  std::set<int> confirmed_ids;  // every id a confirmed track had during the run
};

// Feeds one noisy target to the tracker, one frame per truth sample. Frames for
// which drop(t) is true are not delivered at all (sensor dropout). The RMSE is
// that of the confirmed track over the frames after warmup seconds.
//
// A detection in the 1 % tail of the 99 % gate starts a tentative twin; that is
// expected and harmless as long as the twin never gets confirmed, which is
// what confirmed_ids checks.
SingleTargetRun runSingleTarget(
  TrackManager & tracker, const std::vector<testing::TruthSample> & truth, int class_id,
  unsigned seed, const std::function<bool(double)> & drop = nullptr, double warmup = 1.0)
{
  testing::MeasurementNoise noise(kSigma, seed);
  SingleTargetRun run;
  double sum2 = 0.0;
  int n = 0;
  for (const auto & sample : truth) {
    if (drop && drop(sample.t)) {
      continue;
    }
    EXPECT_TRUE(
      tracker.processFrame(sample.t, {detection(noise.sample(sample.position), class_id)}));
    const auto confirmed = confirmedTracks(tracker, sample.t);
    for (const auto & track : confirmed) {
      run.confirmed_ids.insert(track.id);
    }
    if (sample.t >= warmup) {
      EXPECT_EQ(confirmed.size(), 1u) << "t = " << sample.t;
      if (!confirmed.empty()) {
        sum2 += (confirmed[0].position - sample.position).squaredNorm();
        ++n;
      }
    }
  }
  run.rmse = std::sqrt(sum2 / std::max(n, 1));
  return run;
}

// --- Gate --------------------------------------------------------------------------

TEST(TrackManager, ChiSquareGateQuantiles)
{
  EXPECT_NEAR(chiSquareQuantile3Dof(0.99), 11.3449, 1e-3);
  EXPECT_NEAR(chiSquareQuantile3Dof(0.95), 7.8147, 1e-3);
  EXPECT_NEAR(chiSquareQuantile3Dof(0.5), 2.3660, 1e-3);
  EXPECT_THROW(chiSquareQuantile3Dof(1.0), std::invalid_argument);
  EXPECT_NEAR(TrackManager().gateThreshold(), 11.3449, 1e-3);
}

TEST(TrackManager, RejectsInvalidParameters)
{
  TrackManagerParams p;
  p.confirm_m = 6;
  EXPECT_THROW(TrackManager{p}, std::invalid_argument);
  p = TrackManagerParams();
  p.gate_probability = 0.0;
  EXPECT_THROW(TrackManager{p}, std::invalid_argument);
  p = TrackManagerParams();
  p.max_position_variance = 0.0;
  EXPECT_THROW(TrackManager{p}, std::invalid_argument);
}

// --- Track management ----------------------------------------------------------------

TEST(TrackManager, ConfirmsAfterThreeOfFive)
{
  TrackManager tracker;
  const Eigen::Vector3d p(10.0, 5.0, 0.9);
  tracker.processFrame(0.0, {detection(p)});
  ASSERT_EQ(tracker.size(), 1u);
  EXPECT_FALSE(tracker.tracks(0.0)[0].confirmed);
  tracker.processFrame(1.0 / kRate, {});  // miss
  tracker.processFrame(2.0 / kRate, {detection(p)});
  EXPECT_FALSE(tracker.tracks(2.0 / kRate)[0].confirmed);
  tracker.processFrame(3.0 / kRate, {detection(p)});
  ASSERT_EQ(tracker.size(), 1u);
  const TrackEstimate track = tracker.tracks(3.0 / kRate)[0];
  EXPECT_TRUE(track.confirmed);
  EXPECT_TRUE(track.is_valid);
  EXPECT_EQ(track.hits, 3);
  EXPECT_EQ(track.class_id, kPerson);
}

TEST(TrackManager, DeletesTentativeTrackOnceThreeOfFiveIsOutOfReach)
{
  TrackManager tracker;
  tracker.processFrame(0.0, {detection({0.0, 0.0, 0.9})});
  tracker.processFrame(1.0 / kRate, {});
  tracker.processFrame(2.0 / kRate, {});
  EXPECT_EQ(tracker.size(), 1u);  // 1 hit + 2 frames left could still make 3 of 5
  tracker.processFrame(3.0 / kRate, {});
  EXPECT_EQ(tracker.size(), 0u);
}

TEST(TrackManager, DropsFramesOlderThanTheLastOne)
{
  TrackManager tracker;
  EXPECT_TRUE(tracker.processFrame(1.0, {detection({0.0, 0.0, 0.9})}));
  EXPECT_FALSE(tracker.processFrame(0.9, {detection({0.0, 0.0, 0.9})}));
  EXPECT_TRUE(tracker.processFrame(1.0, {}));
  EXPECT_EQ(tracker.size(), 1u);
}

TEST(TrackManager, AssociatesOnlyWithinTheSameClass)
{
  TrackManagerParams params;
  // Three-model aerial set, to tell from the track which set a class gets
  params.aerial_imm = makeImmParams(
    {"cv", "ca", "ct"}, {0.9, 0.05, 0.05, 0.05, 0.9, 0.05, 0.05, 0.05, 0.9});
  TrackManager tracker(params);
  const Eigen::Vector3d p(0.0, 0.0, 12.0);
  std::vector<int> ids;
  tracker.processFrame(0.0, {detection(p, kPerson)}, &ids);
  const int person_id = ids.at(0);
  // A drone right where the person track is never joins it, and a person track
  // can never turn into a uav track
  tracker.processFrame(1.0 / kRate, {detection(p, kUav)}, &ids);
  ASSERT_EQ(tracker.size(), 2u);
  EXPECT_NE(ids.at(0), person_id);
  const auto person = tracker.track(person_id, 1.0 / kRate);
  const auto drone = tracker.track(ids.at(0), 1.0 / kRate);
  ASSERT_TRUE(person && drone);
  EXPECT_EQ(person->class_id, kPerson);
  EXPECT_EQ(drone->class_id, kUav);
  EXPECT_EQ(person->model_probabilities.size(), 2);  // ground set: CV + CA
  EXPECT_EQ(drone->model_probabilities.size(), 3);   // aerial set
}

TEST(TrackManager, GatesOutFarDetections)
{
  TrackManager tracker;
  for (int k = 0; k < 10; ++k) {
    tracker.processFrame(k / kRate, {detection({0.0, 0.0, 0.9})});
  }
  std::vector<int> ids;
  tracker.processFrame(10.0 / kRate, {detection({8.0, 0.0, 0.9})}, &ids);
  EXPECT_EQ(tracker.size(), 2u);
  EXPECT_EQ(tracker.tracks(10.0 / kRate)[0].missed_frames, 1);
}

// --- Synthetic trajectories (plan §5, Phase 1 exit) ----------------------------------

TEST(TrackManager, ConstantVelocityWalker)
{
  TrackManager tracker;
  const auto truth = testing::generateTrajectory(
    {-10.0, 3.0, 0.9}, {1.4, 0.2, 0.0}, {{20.0}}, kRate);
  const SingleTargetRun run = runSingleTarget(tracker, truth, kPerson, 11);
  EXPECT_LT(run.rmse, 0.3);
  EXPECT_EQ(run.confirmed_ids.size(), 1u);
}

TEST(TrackManager, ConstantAccelerationCar)
{
  TrackManager tracker;
  const auto truth = testing::generateTrajectory(
    {0.0, 0.0, 0.8}, {1.0, 1.0, 0.0}, {{10.0, 2.0}}, kRate);
  const SingleTargetRun run = runSingleTarget(tracker, truth, kCar, 12);
  EXPECT_LT(run.rmse, 0.3);
  EXPECT_EQ(run.confirmed_ids.size(), 1u);
}

TEST(TrackManager, JoggerWithTurns)
{
  // 4 m/s jogger: two 90 degree turns of 2 s each (3.1 m/s^2 lateral)
  TrackManager tracker;
  const double w = M_PI / 4.0;
  const auto truth = testing::generateTrajectory(
    {0.0, 0.0, 0.9}, {4.0, 0.0, 0.0},
    {{3.0}, {2.0, 0.0, w}, {3.0}, {2.0, 0.0, -w}, {3.0}}, kRate);
  const SingleTargetRun run = runSingleTarget(tracker, truth, kPerson, 13);
  EXPECT_LT(run.rmse, 0.3);
  EXPECT_EQ(run.confirmed_ids.size(), 1u);
}

TEST(TrackManager, CoordinatedTurnDrone)
{
  // 10 m/s drone, climbing, with a 0.4 rad/s turn (4 m/s^2 lateral)
  TrackManager tracker;
  const auto truth = testing::generateTrajectory(
    {0.0, 0.0, 20.0}, {10.0, 0.0, 0.5}, {{3.0}, {8.0, 0.0, 0.4}, {3.0}}, kRate);
  const SingleTargetRun run = runSingleTarget(tracker, truth, kUav, 14);
  EXPECT_LT(run.rmse, 0.3);
  EXPECT_EQ(run.confirmed_ids.size(), 1u);
}

// --- Occlusion and coasting -----------------------------------------------------------

TEST(TrackManager, SurvivesTwoSecondDropout)
{
  TrackManager tracker;
  const auto truth = testing::generateTrajectory(
    {0.0, 0.0, 0.9}, {1.4, 0.5, 0.0}, {{12.0}}, kRate);
  const auto dropout = [](double t) {return t > 5.0 && t < 7.0;};

  // Up to the dropout
  std::vector<testing::TruthSample> before;
  std::vector<testing::TruthSample> after;
  for (const auto & s : truth) {
    if (s.t <= 5.0) {
      before.push_back(s);
    } else if (s.t >= 7.0) {
      after.push_back(s);
    }
  }
  const SingleTargetRun first = runSingleTarget(tracker, before, kPerson, 21, dropout);
  ASSERT_EQ(first.confirmed_ids.size(), 1u);
  const int id = *first.confirmed_ids.begin();

  // Coasting: no frames at all for 2 s, the track keeps predicting and stays valid
  for (const double t : {5.5, 6.0, 6.95}) {
    tracker.prune(t);
    const auto track = tracker.track(id, t);
    ASSERT_TRUE(track.has_value()) << "lost at t = " << t;
    EXPECT_TRUE(track->is_valid);
    EXPECT_LT(track->quality, 1.0);
  }
  const auto coasted = tracker.track(id, 7.0);
  ASSERT_TRUE(coasted.has_value());
  EXPECT_LT((coasted->position - after.front().position).norm(), 1.0);

  // Reacquired by the same track
  const SingleTargetRun second = runSingleTarget(tracker, after, kPerson, 22, nullptr, 0.0);
  EXPECT_EQ(second.confirmed_ids, std::set<int>{id});
}

TEST(TrackManager, KeepsOccludedTrackWhileOtherTargetsAreVisible)
{
  // Two walkers 10 m apart; A is occluded for 2 s while B keeps producing frames,
  // so A misses far more than max_missed_frames but must survive coast_timeout
  TrackManager tracker;
  const auto a = testing::generateTrajectory({0.0, 0.0, 0.9}, {1.4, 0.0, 0.0}, {{12.0}}, kRate);
  const auto b = testing::generateTrajectory({0.0, 10.0, 0.9}, {1.4, 0.0, 0.0}, {{12.0}}, kRate);
  testing::MeasurementNoise noise(kSigma, 31);
  std::set<int> ids_a;
  for (size_t k = 0; k < a.size(); ++k) {
    std::vector<Measurement> frame{detection(noise.sample(b[k].position))};
    const bool occluded = a[k].t > 5.0 && a[k].t < 7.0;
    if (!occluded) {
      frame.push_back(detection(noise.sample(a[k].position)));
    }
    tracker.processFrame(a[k].t, frame);
    if (a[k].t < 1.0) {
      continue;
    }
    // The confirmed track nearest to A is always the same one
    const auto confirmed = confirmedTracks(tracker, a[k].t);
    ASSERT_EQ(confirmed.size(), 2u) << "t = " << a[k].t;
    const auto & nearest =
      (confirmed[0].position - a[k].position).norm() <
      (confirmed[1].position - a[k].position).norm() ? confirmed[0] : confirmed[1];
    ids_a.insert(nearest.id);
    EXPECT_TRUE(nearest.is_valid) << "t = " << a[k].t;
    if (occluded) {
      // Coasting: the prediction drifts, but its covariance must keep up
      const Eigen::Vector3d e = nearest.position - a[k].position;
      EXPECT_LT(e.dot(nearest.position_cov.ldlt().solve(e)), chiSquareQuantile3Dof(0.999))
        << "t = " << a[k].t;
    } else {
      EXPECT_LT((nearest.position - a[k].position).norm(), 1.0) << "t = " << a[k].t;
    }
  }
  EXPECT_EQ(ids_a.size(), 1u);
}

TEST(TrackManager, InvalidatesAndDeletesLostTrack)
{
  TrackManager tracker;
  const auto truth = testing::generateTrajectory({0.0, 0.0, 0.9}, {1.4, 0.0, 0.0}, {{3.0}}, kRate);
  const SingleTargetRun run = runSingleTarget(tracker, truth, kPerson, 41);
  ASSERT_EQ(run.confirmed_ids.size(), 1u);
  const int id = *run.confirmed_ids.begin();
  const double t_lost = truth.back().t;

  const auto coasting = tracker.track(id, t_lost + 2.5);
  ASSERT_TRUE(coasting.has_value());
  EXPECT_FALSE(coasting->is_valid);
  EXPECT_DOUBLE_EQ(coasting->quality, 0.0);

  // The covariance grows while nobody sees the target, and the track goes
  double deleted_after = -1.0;
  for (double dt = 0.0; dt < 30.0; dt += 0.1) {
    tracker.prune(t_lost + dt);
    if (tracker.size() == 0) {
      deleted_after = dt;
      break;
    }
  }
  EXPECT_GT(deleted_after, 2.0);
  EXPECT_LT(deleted_after, 10.0);
}

TEST(TrackManager, DeletesCoastedTrackAfterMaxMissedFrames)
{
  // A different target keeps producing frames: the lost one goes right after
  // coast_timeout instead of waiting for the covariance limit
  TrackManager tracker;
  for (int k = 0; k < 30; ++k) {
    tracker.processFrame(
      k / kRate, {detection({0.0, 0.0, 0.9}), detection({20.0, 0.0, 0.9})});
  }
  ASSERT_EQ(tracker.size(), 2u);
  int k = 30;
  for (; k < 200 && tracker.size() == 2; ++k) {
    tracker.processFrame(k / kRate, {detection({20.0, 0.0, 0.9})});
  }
  const double lost_for = (k - 1) / kRate - 29 / kRate;
  EXPECT_EQ(tracker.size(), 1u);
  EXPECT_GT(lost_for, 2.0);
  EXPECT_LT(lost_for, 2.0 + 2.0 / kRate);
}

// --- Heading ----------------------------------------------------------------------------

TEST(TrackManager, HeadingFromVelocityHeldWhenStopped)
{
  TrackManager tracker;
  // Walks north, stops, stands still
  const auto truth = testing::generateTrajectory(
    {0.0, 0.0, 0.9}, {0.0, 1.5, 0.0}, {{5.0}, {1.0, -1.5}, {5.0}}, kRate);
  testing::MeasurementNoise noise(kSigma, 51);
  int id = -1;
  for (const auto & s : truth) {
    std::vector<int> ids;
    tracker.processFrame(s.t, {detection(noise.sample(s.position))}, &ids);
    id = ids.at(0);
    if (std::abs(s.t - 4.5) < 1e-6) {
      EXPECT_NEAR(tracker.track(id, s.t)->heading, M_PI / 2.0, 0.15);
    }
  }
  const auto track = tracker.track(id, truth.back().t);
  ASSERT_TRUE(track.has_value());
  EXPECT_LT(track->velocity.head<2>().norm(), 0.5);
  EXPECT_NEAR(track->heading, M_PI / 2.0, 0.3);
}

// --- Multiple targets ---------------------------------------------------------------

int nearestConfirmedId(const TrackManager & tracker, double t, const Eigen::Vector3d & p)
{
  int id = -1;
  double best = std::numeric_limits<double>::infinity();
  for (const auto & track : confirmedTracks(tracker, t)) {
    const double d = (track.position - p).norm();
    if (d < best) {
      best = d;
      id = track.id;
    }
  }
  return id;
}

// Two same-class targets crossing at right angles: A passes the crossing point
// at t = 7 s, B offset seconds later. Identities are taken from the confirmed
// tracks at t = 3 s and must be unchanged at t = 15 s, with exactly two
// confirmed tracks throughout.
void runCrossing(int class_id, double speed, double height, double offset, unsigned seed)
{
  TrackManager tracker;
  const auto a = testing::generateTrajectory(
    {-7.0 * speed, 0.0, height}, {speed, 0.0, 0.0}, {{15.0}}, kRate);
  const auto b = testing::generateTrajectory(
    {0.0, -(7.0 + offset) * speed, height}, {0.0, speed, 0.0}, {{15.0}}, kRate);
  testing::MeasurementNoise noise(kSigma, seed);
  int id_a = -1;
  int id_b = -1;
  for (size_t k = 0; k < a.size(); ++k) {
    tracker.processFrame(
      a[k].t, {detection(noise.sample(a[k].position), class_id),
        detection(noise.sample(b[k].position), class_id)});
    if (a[k].t > 1.0) {
      ASSERT_EQ(confirmedTracks(tracker, a[k].t).size(), 2u)
        << "seed " << seed << ", t = " << a[k].t;
    }
    if (k == 90) {
      id_a = nearestConfirmedId(tracker, a[k].t, a[k].position);
      id_b = nearestConfirmedId(tracker, a[k].t, b[k].position);
    }
  }
  const double t_end = a.back().t;
  EXPECT_EQ(nearestConfirmedId(tracker, t_end, a.back().position), id_a) << "seed " << seed;
  EXPECT_EQ(nearestConfirmedId(tracker, t_end, b.back().position), id_b) << "seed " << seed;
  const auto track_a = tracker.track(id_a, t_end);
  ASSERT_TRUE(track_a.has_value()) << "seed " << seed;
  EXPECT_LT((track_a->position - a.back().position).norm(), 1.0) << "seed " << seed;
}

TEST(TrackManager, CrossingWalkersKeepTheirIds)
{
  // Two people cannot be in the same place at once: 1 m closest approach.
  // (Through the very same point at the very same time, position-only
  // detections with 0.3 m noise are ambiguous for ~0.8 s and ids swap in ~6 %
  // of runs.)
  for (unsigned seed = 60; seed < 70; ++seed) {
    runCrossing(kPerson, 1.4, 0.9, 1.0, seed);
  }
}

TEST(TrackManager, CrossingDronesKeepTheirIds)
{
  // 10 m/s drones through the same point at the same time
  for (unsigned seed = 70; seed < 80; ++seed) {
    runCrossing(kUav, 10.0, 20.0, 0.0, seed);
  }
}

// --- Association internals ------------------------------------------------------------

TEST(TrackManager, AssignmentIsGloballyOptimal)
{
  // Greedy would take the cheapest pair (0, 0) and force row 1 onto column 1
  Eigen::MatrixXd cost(2, 3);
  cost << 1.0, 2.0, 50.0,
    1.5, 100.0, 50.0;
  EXPECT_EQ(solveAssignment(cost), (std::vector<int>{1, 0}));
  // More columns than rows; every row gets a distinct column
  Eigen::MatrixXd square = Eigen::MatrixXd::Ones(3, 3) - Eigen::MatrixXd::Identity(3, 3);
  EXPECT_EQ(solveAssignment(square), (std::vector<int>{0, 1, 2}));
  EXPECT_THROW(solveAssignment(Eigen::MatrixXd::Zero(3, 2)), std::invalid_argument);
}

TEST(TrackManager, DetectionInTheGateTailDoesNotSpawnADuplicate)
{
  TrackManager tracker;
  for (int k = 0; k < 30; ++k) {
    tracker.processFrame(k / kRate, {detection({0.0, 0.0, 0.9})});
  }
  const auto track = tracker.tracks(29 / kRate).at(0);
  // Just outside the association gate, inside the spawn gate
  const double sigma = std::sqrt(track.position_cov(0, 0) + kSigma * kSigma);
  const double offset = sigma * std::sqrt(0.5 * (tracker.gateThreshold() + 21.1));
  std::vector<int> ids;
  tracker.processFrame(30 / kRate, {detection({offset, 0.0, 0.9})}, &ids);
  EXPECT_EQ(ids.at(0), -1);
  EXPECT_EQ(tracker.size(), 1u);
  EXPECT_EQ(tracker.tracks(30 / kRate).at(0).missed_frames, 1);
}

}  // namespace
}  // namespace estimation
}  // namespace interceptor
