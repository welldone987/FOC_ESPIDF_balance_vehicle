#pragma once

#include <cstdint>

namespace vehicle {
namespace power {

// StartupUndervoltageThreshold_V是启动检查要求的最低母线电压，单位V。
inline constexpr float StartupUndervoltageThreshold_V = 9.0f;
// BatteryVoltageScale恢复7.5k/1k分压前的母线电压。
inline constexpr float BatteryVoltageScale = 8.5f;
// AdcDefaultVref_mV是线性校准使用的缺省参考电压，单位mV。
inline constexpr std::uint32_t AdcDefaultVref_mV = 1100U;

} // namespace power
} // namespace vehicle
