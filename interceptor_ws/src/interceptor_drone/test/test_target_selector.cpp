#include <gtest/gtest.h>
#include <Eigen/Dense>

#include <optional>
#include <stdexcept>
#include <vector>

#include "common/mission.hpp"
#include "estimation/target_selector.hpp"
#include "estimation/track_manager.hpp"

namespace interceptor
{
namespace estimation
{
namespace
{

constexpr int kPerson = static_cast<int>(TargetClass::PERSON);
constexpr int kCar = static_cast<int>(TargetClass::CAR);
constexpr int kBicycle = static_cast<int>(TargetClass::BICYCLE);
constexpr int kUav = static_cast<int>(TargetClass::UAV);

const std::vector<int> kFollowClasses = {kPerson, kBicycle, kCar};
const std::vector<int> kInterceptClasses = {kUav};

TrackEstimate track(int id, int class_id, double x, bool confirmed = true, bool valid = true)
{
  TrackEstimate t;
  t.id = id;
  t.class_id = class_id;
  t.position = Eigen::Vector3d(x, 0.0, 0.0);
  t.confirmed = confirmed;
  t.is_valid = valid;
  return t;
}

SelectionRequest request(
  const std::vector<int> & classes, int operator_track_id = -1,
  std::optional<Eigen::Vector3d> reference = Eigen::Vector3d::Zero())
{
  SelectionRequest r;
  r.eligible_classes = classes;
  r.operator_track_id = operator_track_id;
  r.reference = reference;
  return r;
}

TEST(TargetSelectorTest, NothingSelectableWithoutEligibleClasses)
{
  TargetSelector selector;
  const std::vector<TrackEstimate> tracks = {track(1, kPerson, 5.0), track(2, kUav, 8.0)};
  EXPECT_FALSE(selector.select(tracks, request({})).has_value());  // HOLD
  EXPECT_FALSE(selector.select({}, request(kFollowClasses)).has_value());
}

TEST(TargetSelectorTest, PicksTheClosestConfirmedValidEligibleTrack)
{
  TargetSelector selector;
  const std::vector<TrackEstimate> tracks = {
    track(1, kUav, 2.0),                   // not eligible in FOLLOW
    track(2, kPerson, 3.0, false),         // tentative
    track(3, kPerson, 4.0, true, false),   // lost
    track(4, kCar, 20.0),
    track(5, kPerson, 12.0),
  };
  EXPECT_EQ(selector.select(tracks, request(kFollowClasses)), 5);

  TargetSelector intercept;
  EXPECT_EQ(intercept.select(tracks, request(kInterceptClasses)), 1);
}

TEST(TargetSelectorTest, DistanceIsMeasuredFromTheReference)
{
  TargetSelector selector;
  const std::vector<TrackEstimate> tracks = {track(1, kPerson, 0.0), track(2, kPerson, 30.0)};
  EXPECT_EQ(
    selector.select(tracks, request(kFollowClasses, -1, Eigen::Vector3d(28.0, 0.0, 10.0))), 2);
}

TEST(TargetSelectorTest, HysteresisKeepsTheSelectedTrack)
{
  TargetSelectorParams params;
  params.switch_margin = 3.0;
  TargetSelector selector(params);
  std::vector<TrackEstimate> tracks = {track(1, kPerson, 10.0), track(2, kPerson, 11.0)};
  ASSERT_EQ(selector.select(tracks, request(kFollowClasses)), 1);

  // The other walker gets closer, but not by more than the margin
  for (double x2 = 11.0; x2 >= 7.5; x2 -= 0.25) {
    tracks[1].position.x() = x2;
    EXPECT_EQ(selector.select(tracks, request(kFollowClasses)), 1) << "x2 = " << x2;
  }
  // Clearly closer now
  tracks[1].position.x() = 6.9;
  EXPECT_EQ(selector.select(tracks, request(kFollowClasses)), 2);
  // And it doesn't flip back when they are level again
  tracks[1].position.x() = 10.0;
  EXPECT_EQ(selector.select(tracks, request(kFollowClasses)), 2);
}

TEST(TargetSelectorTest, LostTrackStaysSelectedUntilAnotherOneIsInView)
{
  TargetSelector selector;
  std::vector<TrackEstimate> tracks = {track(1, kPerson, 5.0)};
  ASSERT_EQ(selector.select(tracks, request(kFollowClasses)), 1);

  // Coasting past coast_timeout and nothing else in view: keep it, the planner
  // sees is_valid = false
  tracks[0].is_valid = false;
  EXPECT_EQ(selector.select(tracks, request(kFollowClasses)), 1);
  // A tentative track is not a replacement
  tracks.push_back(track(2, kPerson, 30.0, false));
  EXPECT_EQ(selector.select(tracks, request(kFollowClasses)), 1);
  // A confirmed one is, however far
  tracks[1].confirmed = true;
  EXPECT_EQ(selector.select(tracks, request(kFollowClasses)), 2);
}

TEST(TargetSelectorTest, DeletedTrackIsReplaced)
{
  TargetSelector selector;
  ASSERT_EQ(selector.select({track(1, kPerson, 5.0)}, request(kFollowClasses)), 1);
  EXPECT_EQ(selector.select({track(2, kPerson, 40.0)}, request(kFollowClasses)), 2);
  EXPECT_FALSE(selector.select({}, request(kFollowClasses)).has_value());
  EXPECT_FALSE(selector.selected().has_value());
}

TEST(TargetSelectorTest, OperatorChoiceWinsOverDistance)
{
  TargetSelector selector;
  const std::vector<TrackEstimate> tracks = {
    track(1, kPerson, 2.0), track(2, kPerson, 50.0), track(3, kPerson, 3.0, false),
    track(4, kPerson, 60.0, true, false)};
  EXPECT_EQ(selector.select(tracks, request(kFollowClasses, 2)), 2);
  // A lost operator track stays selected
  EXPECT_EQ(selector.select(tracks, request(kFollowClasses, 4)), 4);
  // No fallback to another track if the requested one is unusable
  EXPECT_FALSE(selector.select(tracks, request(kFollowClasses, 3)).has_value());  // tentative
  EXPECT_FALSE(selector.select(tracks, request(kFollowClasses, 9)).has_value());  // unknown
}

TEST(TargetSelectorTest, IneligibleClassIsNeverSelected)
{
  // INTERCEPT must never pick a person, even if the operator asks for it
  TargetSelector selector;
  const std::vector<TrackEstimate> tracks = {track(1, kPerson, 2.0), track(2, kCar, 3.0)};
  EXPECT_FALSE(selector.select(tracks, request(kInterceptClasses)).has_value());
  EXPECT_FALSE(selector.select(tracks, request(kInterceptClasses, 1)).has_value());
  EXPECT_FALSE(selector.select(tracks, request(kInterceptClasses, 2)).has_value());
}

TEST(TargetSelectorTest, ModeChangeDropsAnIneligibleSelection)
{
  TargetSelector selector;
  const std::vector<TrackEstimate> tracks = {track(1, kPerson, 2.0), track(2, kUav, 40.0)};
  ASSERT_EQ(selector.select(tracks, request(kFollowClasses)), 1);
  EXPECT_EQ(selector.select(tracks, request(kInterceptClasses)), 2);
  EXPECT_FALSE(selector.select(tracks, request({})).has_value());
}

TEST(TargetSelectorTest, BackToAutomaticKeepsTheOperatorTrack)
{
  TargetSelector selector;
  const std::vector<TrackEstimate> tracks = {track(1, kPerson, 2.0), track(2, kPerson, 4.0)};
  ASSERT_EQ(selector.select(tracks, request(kFollowClasses, 2)), 2);
  // Track 1 is closer, but by less than the margin
  EXPECT_EQ(selector.select(tracks, request(kFollowClasses)), 2);
}

TEST(TargetSelectorTest, WithoutReferencePrefersTheOldestTrack)
{
  TargetSelector selector;
  std::vector<TrackEstimate> tracks = {track(7, kPerson, 50.0), track(3, kPerson, 90.0)};
  EXPECT_EQ(selector.select(tracks, request(kFollowClasses, -1, std::nullopt)), 3);
  tracks.push_back(track(1, kPerson, 1.0));
  EXPECT_EQ(selector.select(tracks, request(kFollowClasses, -1, std::nullopt)), 3);
}

TEST(TargetSelectorTest, RejectsNegativeMargin)
{
  TargetSelectorParams params;
  params.switch_margin = -1.0;
  EXPECT_THROW(TargetSelector{params}, std::invalid_argument);
}

}  // namespace
}  // namespace estimation
}  // namespace interceptor
