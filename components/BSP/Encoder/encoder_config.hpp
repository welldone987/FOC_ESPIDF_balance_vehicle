#pragma once

#include <cstdint>

namespace vehicle {
namespace encoder {
namespace config {

// kI2cFrequencyHz设置AS5600所在I2C总线的时钟频率。
inline constexpr std::uint32_t kI2cFrequencyHz = 400000U;
inline constexpr std::uint8_t kAs5600Address = 0x36U;
inline constexpr std::uint8_t kAs5600RawAngleRegister = 0x0cU;
inline constexpr std::int64_t kReadMaxDurationUs = 2000;
inline constexpr std::int64_t kOutputMaxAgeUs = 4000;
inline constexpr float kWheelVelocityFilterS = 0.01f;

static_assert(kReadMaxDurationUs > 0 && kOutputMaxAgeUs >= kReadMaxDurationUs);
static_assert(kWheelVelocityFilterS >= 0.0f);

} // namespace config
} // namespace encoder
} // namespace vehicle
