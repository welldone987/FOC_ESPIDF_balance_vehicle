#pragma once

#include <cstdint>

namespace vehicle {
namespace current_sensor {

// Shunt_Ohm是相线分流电阻阻值，单位Ohm。
inline constexpr float Shunt_Ohm = 0.01f;
// AmplifierGain是INA240A2的电压增益，单位V/V。
inline constexpr float AmplifierGain = 50.0f;
// OffsetSamples是上电零偏校准的采样批次数。
inline constexpr unsigned OffsetSamples = 1000U;
// AdcMin_mV和AdcMax_mV限定有效相电流采样输入范围，单位mV。
inline constexpr int AdcMin_mV = 150;
inline constexpr int AdcMax_mV = 2450;
// OffsetMin_mV和OffsetMax_mV限定可接受的静态零偏均值，单位mV。
inline constexpr int OffsetMin_mV = 1300;
inline constexpr int OffsetMax_mV = 2000;
// OffsetNoise_mV限制零偏校准期间的峰峰噪声，单位mV。
inline constexpr int OffsetNoise_mV = 100;
// PhaseTrip_A限制实测相电流及重建C相电流，单位A。
inline constexpr float PhaseTrip_A = 1.3f;
// ReadMaxDuration_us限制一批四路ADC读取的允许耗时，单位us。
inline constexpr std::int64_t ReadMaxDuration_us = 2000;
// Polarity把INA240输出方向转换为桥臂流向电机为正的相电流坐标。
inline constexpr float Polarity = 1.0f;
// AdcDefaultVref_mV是线性校准使用的缺省参考电压，单位mV。
inline constexpr std::uint32_t AdcDefaultVref_mV = 1100U;

static_assert(ReadMaxDuration_us > 0);
static_assert(Shunt_Ohm > 0.0f && AmplifierGain > 0.0f);

} // namespace current_sensor
} // namespace vehicle
