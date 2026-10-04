#include <gtest/gtest.h>
#include <Eigen/Dense>

#include <stdexcept>
#include <vector>

#include "common/mission.hpp"
#include "estimation/frame_assembler.hpp"
#include "estimation/track_manager.hpp"
#include "synthetic_trajectory.hpp"

namespace interceptor
{
namespace estimation
{
namespace
{

constexpr int kPerson = static_cast<int>(TargetClass::PERSON);
constexpr int kUav = static_cast<int>(TargetClass::UAV);

Measurement detection(double x, int class_id = kPerson)
{
  Measurement m;
  m.position = Eigen::Vector3d(x, 0.0, 0.0);
  m.covariance = Eigen::Matrix3d::Identity() * 0.09;
  m.class_id = class_id;
  return m;
}

TEST(FrameAssemblerTest, GroupsDetectionsWithTheSameStamp)
{
  FrameAssembler assembler;
  EXPECT_TRUE(assembler.add(1.0, detection(1.0), 1.020));
  EXPECT_TRUE(assembler.add(1.0, detection(2.0), 1.021));
  EXPECT_TRUE(assembler.add(1.0, detection(3.0, kUav), 1.022));
  EXPECT_EQ(assembler.pending(), 1u);

  const auto frames = assembler.pop(1.040);
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_DOUBLE_EQ(frames[0].stamp, 1.0);
  ASSERT_EQ(frames[0].detections.size(), 3u);
  EXPECT_DOUBLE_EQ(frames[0].detections[0].position.x(), 1.0);
  EXPECT_EQ(frames[0].detections[2].class_id, kUav);
  EXPECT_EQ(assembler.pending(), 0u);
  EXPECT_EQ(assembler.lastStamp(), 1.0);
}

TEST(FrameAssemblerTest, WaitsForTheTimeoutBeforeClosingTheNewestFrame)
{
  FrameAssemblerParams params;
  params.frame_timeout = 0.01;
  FrameAssembler assembler(params);
  assembler.add(1.0, detection(1.0), 1.020);
  EXPECT_TRUE(assembler.pop(1.020).empty());
  EXPECT_TRUE(assembler.pop(1.029).empty());
  // More detections of the same frame can still join
  assembler.add(1.0, detection(2.0), 1.029);
  const auto frames = assembler.pop(1.030);
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].detections.size(), 2u);
}

TEST(FrameAssemblerTest, NewerDetectionClosesThePendingFrame)
{
  FrameAssembler assembler;
  assembler.add(1.0, detection(1.0), 1.020);
  assembler.add(1.0, detection(2.0), 1.020);
  assembler.add(1.033, detection(1.1), 1.053);  // before the first frame timed out

  auto frames = assembler.pop(1.053);
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_DOUBLE_EQ(frames[0].stamp, 1.0);
  EXPECT_EQ(frames[0].detections.size(), 2u);
  EXPECT_EQ(assembler.pending(), 1u);

  frames = assembler.pop(1.070);
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_DOUBLE_EQ(frames[0].stamp, 1.033);
}

TEST(FrameAssemblerTest, StampTolerance)
{
  FrameAssemblerParams params;
  params.stamp_tolerance = 0.005;
  FrameAssembler assembler(params);
  assembler.add(1.000, detection(1.0), 2.0);
  assembler.add(1.004, detection(2.0), 2.0);  // same frame
  assembler.add(1.010, detection(3.0), 2.0);  // next frame
  EXPECT_EQ(assembler.pending(), 2u);

  const auto frames = assembler.pop(3.0);
  ASSERT_EQ(frames.size(), 2u);
  EXPECT_DOUBLE_EQ(frames[0].stamp, 1.000);  // stamp of the first detection
  EXPECT_EQ(frames[0].detections.size(), 2u);
  EXPECT_DOUBLE_EQ(frames[1].stamp, 1.010);
  EXPECT_EQ(frames[1].detections.size(), 1u);
}

TEST(FrameAssemblerTest, DropsLateDetections)
{
  FrameAssembler assembler;
  assembler.add(1.0, detection(1.0), 1.02);
  ASSERT_EQ(assembler.pop(1.1).size(), 1u);

  // The frame at 1.0 was handed to the tracker: a second frame at the same
  // time would count as a miss for every track it saw
  EXPECT_FALSE(assembler.add(1.0, detection(2.0), 1.11));
  EXPECT_FALSE(assembler.add(1.003, detection(2.0), 1.11));  // within the tolerance
  EXPECT_FALSE(assembler.add(0.9, detection(2.0), 1.11));
  EXPECT_EQ(assembler.droppedLate(), 3u);
  EXPECT_EQ(assembler.pending(), 0u);
  EXPECT_TRUE(assembler.pop(2.0).empty());

  EXPECT_TRUE(assembler.add(1.033, detection(2.0), 1.12));
  EXPECT_EQ(assembler.pop(2.0).size(), 1u);
}

TEST(FrameAssemblerTest, OutOfOrderArrivalComesOutInStampOrder)
{
  FrameAssembler assembler;
  assembler.add(1.033, detection(2.0), 1.050);
  assembler.add(1.000, detection(1.0), 1.051);  // older frame, delivered late
  assembler.add(1.033, detection(3.0), 1.052);

  const auto frames = assembler.pop(1.2);
  ASSERT_EQ(frames.size(), 2u);
  EXPECT_DOUBLE_EQ(frames[0].stamp, 1.000);
  EXPECT_DOUBLE_EQ(frames[1].stamp, 1.033);
  EXPECT_EQ(frames[1].detections.size(), 2u);
}

TEST(FrameAssemblerTest, ClearForgetsEverything)
{
  FrameAssembler assembler;
  assembler.add(5.0, detection(1.0), 5.0);
  ASSERT_EQ(assembler.pop(6.0).size(), 1u);
  assembler.add(4.0, detection(1.0), 6.0);
  assembler.add(7.0, detection(1.0), 6.0);
  assembler.clear();
  EXPECT_EQ(assembler.pending(), 0u);
  EXPECT_EQ(assembler.droppedLate(), 0u);
  EXPECT_FALSE(assembler.lastStamp().has_value());
  // After a simulation reset, early stamps are accepted again
  EXPECT_TRUE(assembler.add(0.1, detection(1.0), 0.1));
}

TEST(FrameAssemblerTest, RejectsNegativeParameters)
{
  FrameAssemblerParams params;
  params.stamp_tolerance = -0.001;
  EXPECT_THROW(FrameAssembler{params}, std::invalid_argument);
  params = FrameAssemblerParams();
  params.frame_timeout = -1.0;
  EXPECT_THROW(FrameAssembler{params}, std::invalid_argument);
}

// Two walkers at 30 Hz, each detection delivered as its own message as the
// perception pipeline does. Grouped into frames, every detection of a sensor
// frame updates its track in one processFrame() call and no track ever misses.
// Fed one message at a time, every track misses in every frame the other
// target was detected in.
TEST(FrameAssemblerTest, TracksSeeEveryDetectionOfAFrameAtOnce)
{
  const auto a = testing::generateTrajectory({0.0, 0.0, 0.9}, {1.4, 0.0, 0.0}, {{4.0}}, 30.0);
  const auto b = testing::generateTrajectory({0.0, 10.0, 0.9}, {0.0, -1.4, 0.0}, {{4.0}}, 30.0);
  ASSERT_EQ(a.size(), b.size());
  // Less noise than the 0.3 m the detections claim, so no detection falls in
  // the 1 % tail outside the gate and every frame must be a hit
  testing::MeasurementNoise noise(0.1, 7);

  FrameAssembler assembler;
  TrackManager grouped;
  TrackManager ungrouped;
  const double latency = 0.02;  // perception delay; both messages arrive within 1 ms
  for (size_t k = 0; k < a.size(); ++k) {
    const double t = a[k].t;
    Measurement da = detection(0.0);
    Measurement db = detection(0.0);
    da.position = noise.sample(a[k].position);
    db.position = noise.sample(b[k].position);

    ASSERT_TRUE(assembler.add(t, da, t + latency));
    for (const auto & frame : assembler.pop(t + latency)) {
      grouped.processFrame(frame.stamp, frame.detections);
    }
    ASSERT_TRUE(assembler.add(t, db, t + latency + 0.001));
    for (const auto & frame : assembler.pop(t + latency + 0.001)) {
      grouped.processFrame(frame.stamp, frame.detections);
    }

    ungrouped.processFrame(t, {da});
    ungrouped.processFrame(t, {db});
  }
  for (const auto & frame : assembler.pop(10.0)) {
    grouped.processFrame(frame.stamp, frame.detections);
  }

  const double t_end = a.back().t;
  const auto tracks = grouped.tracks(t_end);
  ASSERT_EQ(tracks.size(), 2u);
  for (const auto & track : tracks) {
    EXPECT_TRUE(track.confirmed);
    EXPECT_EQ(track.missed_frames, 0);
    EXPECT_DOUBLE_EQ(track.quality, 1.0);
    EXPECT_EQ(track.hits, static_cast<int>(a.size()));
  }

  // Without grouping every track loses half of its M-of-N window
  for (const auto & track : ungrouped.tracks(t_end)) {
    EXPECT_LT(track.quality, 0.7) << "track " << track.id;
  }
}

}  // namespace
}  // namespace estimation
}  // namespace interceptor
