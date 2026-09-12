#pragma once

#include <cstdint>

namespace vehicle {
namespace encoder {

// I2cFrequency_Hz设置AS5600所在I2C总线的时钟频率。
inline constexpr std::uint32_t I2cFrequency_Hz = 400000U;
inline constexpr std::uint8_t As5600Address = 0x36U;
inline constexpr std::uint8_t As5600RawAngleRegister = 0x0cU;
inline constexpr std::int64_t ReadMaxDuration_us = 2000;
inline constexpr std::int64_t OutputMaxAge_us = 4000;
inline constexpr float WheelVelocityFilter_s = 0.01f;

static_assert(ReadMaxDuration_us > 0 && OutputMaxAge_us >= ReadMaxDuration_us);
static_assert(WheelVelocityFilter_s >= 0.0f);

} // namespace encoder
} // namespace vehicle
