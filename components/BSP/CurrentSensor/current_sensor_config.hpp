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
// DmaSampleFreq_Hz是ADC连续转换的总频率，取ESP32驱动允许下限，与20kHz PWM载波同频。
inline constexpr std::uint32_t DmaSampleFreq_Hz = 20000U;
// DmaScansPerFrame是每次DMA转换帧包含的四通道扫描数。
inline constexpr unsigned DmaScansPerFrame = 16U;
// DmaFrameBytes是转换帧字节数；一次四通道扫描8字节，帧长度保持4字节对齐。
inline constexpr std::uint32_t DmaFrameBytes = 8U * DmaScansPerFrame;
// DmaStoreBytes是驱动内部池容量，池满时按flush_pool丢弃最旧数据。
inline constexpr std::uint32_t DmaStoreBytes = 1024U;
// DmaReadBufferBytes是单次排空读取的缓冲上限。
inline constexpr unsigned DmaReadBufferBytes = 256U;
// DmaDrainReadLimit限制单次Read()的排空次数，兜住异常情况下的读取耗时。
inline constexpr unsigned DmaDrainReadLimit = 4U;
// DmaScanPeriod_us是一次四通道扫描的标称周期，作为采样时刻的保守上界。
inline constexpr std::int64_t DmaScanPeriod_us = 4 * 1000000LL / DmaSampleFreq_Hz;
// Polarity把INA240输出方向转换为桥臂流向电机为正的相电流坐标。
inline constexpr float Polarity = 1.0f;
// AdcDefaultVref_mV是线性校准使用的缺省参考电压，单位mV。
inline constexpr std::uint32_t AdcDefaultVref_mV = 1100U;

static_assert(ReadMaxDuration_us > 0);
static_assert(DmaSampleFreq_Hz >= 20000U && DmaSampleFreq_Hz <= 2000000U);
static_assert(DmaFrameBytes % 4U == 0U && DmaFrameBytes % DmaScansPerFrame == 0U);
static_assert(DmaReadBufferBytes % 4U == 0U && DmaReadBufferBytes > DmaFrameBytes / DmaScansPerFrame);
static_assert(DmaStoreBytes >= DmaFrameBytes);
static_assert(DmaDrainReadLimit > 0U);
static_assert(Shunt_Ohm > 0.0f && AmplifierGain > 0.0f);

} // namespace current_sensor
} // namespace vehicle
