#pragma once

#include <cstdint>

namespace vehicle {
namespace imu {

// Address是BMI160的7位I2C地址。
inline constexpr std::uint8_t Address = 0x69U;
// I2cFrequency_Hz设置BMI160所在I2C0总线的时钟频率，单位Hz。
inline constexpr std::uint32_t I2cFrequency_Hz = 400000U;
// AccelerationScale把BMI160在±2g量程下的原始计数换算为g。
inline constexpr float AccelerationScale = 16384.0f;
// GyroScale把BMI160在±1000dps量程下的原始计数换算为deg/s。
inline constexpr float GyroScale = 32.8f;
// FocTimeout_ms限制陀螺仪FOC偏置校准的等待时间，单位ms。
inline constexpr std::uint32_t FocTimeout_ms = 500U;
// ComplementaryTimeConstant_s是互补滤波器陀螺仪权重的衰减时间常数，单位s。
inline constexpr float ComplementaryTimeConstant_s = 0.098f;

static_assert(ComplementaryTimeConstant_s > 0.0f);

} // namespace imu
} // namespace vehicle
