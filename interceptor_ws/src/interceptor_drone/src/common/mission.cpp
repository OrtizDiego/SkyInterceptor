#include "common/mission.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <stdexcept>
#include <utility>

namespace interceptor
{
namespace
{

const std::array<std::pair<MissionMode, const char *>, 3> MISSION_MODE_NAMES = {{
  {MissionMode::HOLD, "hold"},
  {MissionMode::FOLLOW, "follow"},
  {MissionMode::INTERCEPT, "intercept"},
}};

const std::array<std::pair<TargetClass, const char *>, 5> TARGET_CLASS_NAMES = {{
  {TargetClass::PERSON, "person"},
  {TargetClass::CAR, "car"},
  {TargetClass::TRUCK, "truck"},
  {TargetClass::BICYCLE, "bicycle"},
  {TargetClass::UAV, "uav"},
}};

std::string toLower(std::string text)
{
  std::transform(
    text.begin(), text.end(), text.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return text;
}

template<typename Enum, std::size_t N>
std::optional<Enum> fromName(
  const std::array<std::pair<Enum, const char *>, N> & table, const std::string & name)
{
  const std::string lower = toLower(name);
  for (const auto & entry : table) {
    if (lower == entry.second) {
      return entry.first;
    }
  }
  return std::nullopt;
}

template<typename Enum, std::size_t N>
std::optional<Enum> fromValue(const std::array<std::pair<Enum, const char *>, N> & table, int value)
{
  for (const auto & entry : table) {
    if (static_cast<int>(entry.first) == value) {
      return entry.first;
    }
  }
  return std::nullopt;
}

template<typename Enum, std::size_t N>
std::string toName(const std::array<std::pair<Enum, const char *>, N> & table, Enum value)
{
  for (const auto & entry : table) {
    if (entry.first == value) {
      return entry.second;
    }
  }
  return "unknown";
}

}  // namespace

std::optional<MissionMode> missionModeFromString(const std::string & name)
{
  return fromName(MISSION_MODE_NAMES, name);
}

std::optional<MissionMode> missionModeFromValue(int value)
{
  return fromValue(MISSION_MODE_NAMES, value);
}

std::string toString(MissionMode mode)
{
  return toName(MISSION_MODE_NAMES, mode);
}

std::optional<TargetClass> targetClassFromString(const std::string & name)
{
  return fromName(TARGET_CLASS_NAMES, name);
}

std::optional<TargetClass> targetClassFromValue(int value)
{
  return fromValue(TARGET_CLASS_NAMES, value);
}

std::string toString(TargetClass target_class)
{
  return toName(TARGET_CLASS_NAMES, target_class);
}

std::vector<TargetClass> parseTargetClasses(const std::vector<std::string> & names)
{
  std::vector<TargetClass> classes;
  classes.reserve(names.size());
  for (const auto & name : names) {
    const auto target_class = targetClassFromString(name);
    if (!target_class) {
      throw std::invalid_argument("unknown target class '" + name + "'");
    }
    classes.push_back(*target_class);
  }
  return classes;
}

bool isGroundClass(TargetClass target_class)
{
  return target_class != TargetClass::UAV;
}

}  // namespace interceptor
