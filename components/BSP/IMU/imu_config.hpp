#pragma once

#include <cstdint>

namespace vehicle {
namespace imu {
namespace config {

inline constexpr std::uint8_t kAddress = 0x69U;
inline constexpr std::uint32_t kI2cFrequencyHz = 400000U;
inline constexpr float kAccelerationScale = 16384.0f;
inline constexpr float kGyroScale = 32.8f;
inline constexpr std::uint32_t kFocTimeoutMs = 500U;
inline constexpr float kComplementaryTimeConstantS = 0.098f;

static_assert(kComplementaryTimeConstantS > 0.0f);

} // namespace config
} // namespace imu
} // namespace vehicle
