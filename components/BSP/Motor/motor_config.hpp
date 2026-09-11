#pragma once

#include <cstdint>
#include "current_sensor_config.hpp"

namespace vehicle {
namespace motor {
namespace config {

inline constexpr std::uint32_t kControlRateHz = 1000U;
inline constexpr std::uint64_t kControlPeriodUs = 1000000ULL / kControlRateHz;
inline constexpr float kMaximumControlGapS = 0.010f;
inline constexpr int kPolePairs = 7;
inline constexpr float kPwmBusReferenceV = 12.0f;
inline constexpr float kSensorAlignmentVoltageV = 2.0f;
// kMotor0ForwardSign和kMotor1ForwardSign把车辆前进坐标映射到左右电机坐标。
inline constexpr float kMotor0ForwardSign = -1.0f;
inline constexpr float kMotor1ForwardSign = -1.0f;
// kCurrentHardwareVerified控制是否允许驱动初始化和有运动的电机对齐。
inline constexpr bool kCurrentHardwareVerified = true;
inline constexpr float kCurrentLimitA = 1.0f;
inline constexpr std::int64_t kCurrentOutputMaxAgeUs = 2000;
// kCurrentKpVPerA和kCurrentKiVPerAS属于BSP/Motor的Iq电流PI。
inline constexpr float kCurrentKpVPerA = 5.0f;
inline constexpr float kCurrentKiVPerAS = 200.0f;
// kCurrentFilterS是电流PI输入端Iq一阶低通滤波时间常数，单位s。
inline constexpr float kCurrentFilterS = 0.0005f;
inline constexpr float kUqLimitV = 3.0f;
inline constexpr float kSvpwmLinearMargin = 0.9f;

static_assert(kCurrentLimitA > 0.0f);
static_assert(::vehicle::current_sensor::config::kPhaseTripA > kCurrentLimitA);
static_assert(kCurrentOutputMaxAgeUs >= ::vehicle::current_sensor::config::kReadMaxDurationUs);
static_assert(kUqLimitV > 0.0f && kPwmBusReferenceV > 0.0f);
static_assert(kSvpwmLinearMargin > 0.0f && kSvpwmLinearMargin <= 1.0f);
static_assert(kCurrentFilterS >= 0.0f);
static_assert(kMotor0ForwardSign == 1.0f || kMotor0ForwardSign == -1.0f);
static_assert(kMotor1ForwardSign == 1.0f || kMotor1ForwardSign == -1.0f);

} // namespace config
} // namespace motor
} // namespace vehicle
