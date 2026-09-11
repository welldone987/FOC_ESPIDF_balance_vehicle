#pragma once

#include <cstdint>

namespace vehicle {
namespace power {
namespace config {

inline constexpr float kStartupUndervoltageThresholdV = 9.0f;
// kBatteryVoltageScale恢复7.5k/1k分压前的母线电压。
inline constexpr float kBatteryVoltageScale = 8.5f;
inline constexpr std::uint32_t kAdcDefaultVrefMv = 1100U;

} // namespace config
} // namespace power
} // namespace vehicle
