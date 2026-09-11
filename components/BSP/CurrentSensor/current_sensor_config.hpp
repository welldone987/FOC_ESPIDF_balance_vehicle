#pragma once

#include <cstdint>

namespace vehicle {
namespace current_sensor {
namespace config {

// kShuntOhm和kAmplifierGain把INA240输出电压换算为相电流，单位A。
inline constexpr float kShuntOhm = 0.01f;
inline constexpr float kAmplifierGain = 50.0f;
inline constexpr unsigned kOffsetSamples = 1000U;
inline constexpr int kAdcMinMv = 150;
inline constexpr int kAdcMaxMv = 2450;
inline constexpr int kOffsetMinMv = 1300;
inline constexpr int kOffsetMaxMv = 1900;
inline constexpr int kOffsetNoiseMv = 100;
// kPhaseTripA限制实测相电流及重建C相电流，单位A。
inline constexpr float kPhaseTripA = 1.3f;
inline constexpr std::int64_t kReadMaxDurationUs = 2000;
// kPolarity把INA240输出方向转换为桥臂流向电机为正的相电流坐标。
inline constexpr float kPolarity = 1.0f;
inline constexpr std::uint32_t kAdcDefaultVrefMv = 1100U;

static_assert(kReadMaxDurationUs > 0);
static_assert(kShuntOhm > 0.0f && kAmplifierGain > 0.0f);

} // namespace config
} // namespace current_sensor
} // namespace vehicle
