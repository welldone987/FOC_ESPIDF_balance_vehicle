#pragma once

#include <cstdint>

namespace vehicle {
namespace encoder {

// I2cFrequency_Hz设置AS5600所在I2C总线的时钟频率，单位Hz。
inline constexpr std::uint32_t I2cFrequency_Hz = 400000U;
// As5600Address是AS5600的7位I2C地址。
inline constexpr std::uint8_t As5600Address = 0x36U;
// As5600RawAngleRegister指向AS5600的12位原始角度寄存器。
inline constexpr std::uint8_t As5600RawAngleRegister = 0x0cU;
// ReadMaxDuration_us限制单次Refresh()读取AS5600的允许耗时，单位us。
inline constexpr std::int64_t ReadMaxDuration_us = 3000;
// OutputMaxAge_us限制编码器采样时刻到PWM提交的最大年龄，单位us。
inline constexpr std::int64_t OutputMaxAge_us = 4000;
// WheelVelocityFilter_s是轮速一阶低通滤波时间常数，单位s。
inline constexpr float WheelVelocityFilter_s = 0.01f;

static_assert(ReadMaxDuration_us > 0 && OutputMaxAge_us >= ReadMaxDuration_us);
static_assert(WheelVelocityFilter_s >= 0.0f);

} // namespace encoder
} // namespace vehicle
