#pragma once

#include <cstdint>

namespace vehicle {
namespace power {

inline constexpr float StartupUndervoltageThreshold_V = 9.0f;
// BatteryVoltageScale恢复7.5k/1k分压前的母线电压。
inline constexpr float BatteryVoltageScale = 8.5f;
inline constexpr std::uint32_t AdcDefaultVref_mV = 1100U;

} // namespace power
} // namespace vehicle
