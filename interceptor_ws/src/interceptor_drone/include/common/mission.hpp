#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace interceptor
{

// Mission modes. Values match MissionMode.msg / SetMissionMode.srv.
enum class MissionMode : uint8_t
{
  HOLD = 0,
  FOLLOW = 1,
  INTERCEPT = 2,
};

// Target classes. Values match the TargetDetection.msg class constants.
enum class TargetClass : int32_t
{
  PERSON = 0,
  CAR = 1,
  TRUCK = 2,
  BICYCLE = 3,
  UAV = 4,
};

// "hold" | "follow" | "intercept" (case-insensitive); nullopt for anything else
std::optional<MissionMode> missionModeFromString(const std::string & name);
std::optional<MissionMode> missionModeFromValue(int value);
std::string toString(MissionMode mode);

// "person" | "car" | "truck" | "bicycle" | "uav" (case-insensitive)
std::optional<TargetClass> targetClassFromString(const std::string & name);
std::optional<TargetClass> targetClassFromValue(int value);
std::string toString(TargetClass target_class);

// Parses a class whitelist such as the eligible_classes parameter. Throws
// std::invalid_argument naming the first unknown class.
std::vector<TargetClass> parseTargetClasses(const std::vector<std::string> & names);

// Persons and vehicles: everything the keep-out barrier protects
bool isGroundClass(TargetClass target_class);

}  // namespace interceptor
