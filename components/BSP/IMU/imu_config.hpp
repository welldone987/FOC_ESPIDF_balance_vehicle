#pragma once

#include <cstdint>

namespace vehicle {
namespace imu {

inline constexpr std::uint8_t Address = 0x69U;
inline constexpr std::uint32_t I2cFrequency_Hz = 400000U;
inline constexpr float AccelerationScale = 16384.0f;
inline constexpr float GyroScale = 32.8f;
inline constexpr std::uint32_t FocTimeout_ms = 500U;
inline constexpr float ComplementaryTimeConstant_s = 0.098f;

static_assert(ComplementaryTimeConstant_s > 0.0f);

} // namespace imu
} // namespace vehicle
