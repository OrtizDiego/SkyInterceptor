#include <gtest/gtest.h>

#include <stdexcept>
#include <string>
#include <vector>

#include "common/mission.hpp"

namespace interceptor
{
namespace
{

TEST(MissionTest, ParsesMissionModes)
{
  EXPECT_EQ(missionModeFromString("hold"), MissionMode::HOLD);
  EXPECT_EQ(missionModeFromString("follow"), MissionMode::FOLLOW);
  EXPECT_EQ(missionModeFromString("INTERCEPT"), MissionMode::INTERCEPT);
  EXPECT_FALSE(missionModeFromString("attack").has_value());
  EXPECT_FALSE(missionModeFromString("").has_value());
}

TEST(MissionTest, MissionModeValuesRoundTrip)
{
  for (int value = 0; value <= 2; ++value) {
    const auto mode = missionModeFromValue(value);
    ASSERT_TRUE(mode.has_value());
    EXPECT_EQ(static_cast<int>(*mode), value);
    EXPECT_EQ(missionModeFromString(toString(*mode)), mode);
  }
  EXPECT_FALSE(missionModeFromValue(3).has_value());
  EXPECT_FALSE(missionModeFromValue(-1).has_value());
}

TEST(MissionTest, TargetClassValuesRoundTrip)
{
  for (int value = 0; value <= 4; ++value) {
    const auto target_class = targetClassFromValue(value);
    ASSERT_TRUE(target_class.has_value());
    EXPECT_EQ(static_cast<int>(*target_class), value);
    EXPECT_EQ(targetClassFromString(toString(*target_class)), target_class);
  }
  EXPECT_FALSE(targetClassFromValue(5).has_value());
  EXPECT_EQ(targetClassFromString("UAV"), TargetClass::UAV);
}

TEST(MissionTest, ParsesClassWhitelist)
{
  const auto classes = parseTargetClasses({"person", "bicycle", "car"});
  EXPECT_EQ(
    classes,
    (std::vector<TargetClass>{TargetClass::PERSON, TargetClass::BICYCLE, TargetClass::CAR}));
  EXPECT_THROW(parseTargetClasses({"person", "dog"}), std::invalid_argument);
}

TEST(MissionTest, OnlyUavIsNotAGroundClass)
{
  EXPECT_TRUE(isGroundClass(TargetClass::PERSON));
  EXPECT_TRUE(isGroundClass(TargetClass::CAR));
  EXPECT_TRUE(isGroundClass(TargetClass::TRUCK));
  EXPECT_TRUE(isGroundClass(TargetClass::BICYCLE));
  EXPECT_FALSE(isGroundClass(TargetClass::UAV));
}

}  // namespace
}  // namespace interceptor
