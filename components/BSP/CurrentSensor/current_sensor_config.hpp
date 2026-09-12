#pragma once

#include <cstdint>

namespace vehicle {
namespace current_sensor {

// Shunt_Ohm和AmplifierGain把INA240输出电压换算为相电流，单位A。
inline constexpr float Shunt_Ohm = 0.01f;
inline constexpr float AmplifierGain = 50.0f;
inline constexpr unsigned OffsetSamples = 1000U;
inline constexpr int AdcMin_mV = 150;
inline constexpr int AdcMax_mV = 2450;
inline constexpr int OffsetMin_mV = 1300;
inline constexpr int OffsetMax_mV = 2000;
inline constexpr int OffsetNoise_mV = 100;
// PhaseTrip_A限制实测相电流及重建C相电流，单位A。
inline constexpr float PhaseTrip_A = 1.3f;
inline constexpr std::int64_t ReadMaxDuration_us = 2000;
// Polarity把INA240输出方向转换为桥臂流向电机为正的相电流坐标。
inline constexpr float Polarity = 1.0f;
inline constexpr std::uint32_t AdcDefaultVref_mV = 1100U;

static_assert(ReadMaxDuration_us > 0);
static_assert(Shunt_Ohm > 0.0f && AmplifierGain > 0.0f);

} // namespace current_sensor
} // namespace vehicle
