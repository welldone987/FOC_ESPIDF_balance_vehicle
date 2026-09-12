#pragma once

#include <cstdint>
#include "current_sensor_config.hpp"

namespace vehicle {
namespace motor {

inline constexpr std::uint32_t ControlRate_Hz = 500U;
inline constexpr std::uint64_t ControlPeriod_us = 1000000ULL / ControlRate_Hz;
inline constexpr float MaximumControlGap_s = 0.010f;
inline constexpr int PolePairs = 7;
inline constexpr float PwmBusReference_V = 12.0f;
inline constexpr float SensorAlignmentVoltage_V = 2.0f;
// ForwardSign_M0和ForwardSign_M1把车辆前进坐标映射到左右电机坐标。
inline constexpr float ForwardSign_M0 = -1.0f;
inline constexpr float ForwardSign_M1 = -1.0f;
// CurrentHardwareVerified控制是否允许驱动初始化和有运动的电机对齐。
inline constexpr bool CurrentHardwareVerified = true;
inline constexpr float CurrentLimit_A = 1.0f;
inline constexpr std::int64_t CurrentOutputMaxAge_us = 2000;
// IqKp_V_per_A和IqKi_V_per_A_s属于BSP/Motor的Iq电流PI。
inline constexpr float IqKp_V_per_A = 5.0f;
inline constexpr float IqKi_V_per_A_s = 200.0f;
// CurrentFilter_s是电流PI输入端Iq一阶低通滤波时间常数，单位s。
inline constexpr float CurrentFilter_s = 0.0005f;
// CurrentOutputFilter_s是电流PI输出端Uq一阶低通滤波时间常数，单位s；调用处已注释，当前不生效。
// Lesson10源码写0.05s但注释为5ms；这里采用注释意图，避免给500Hz电流环引入50ms延迟。
inline constexpr float CurrentOutputFilter_s = 0.005f;
// 旧版6V的80%；电流目标仍独立限制为每轮±1A。
inline constexpr float UqLimit_V = 4.8f;
inline constexpr float SvpwmLinearMargin = 0.9f;

static_assert(CurrentLimit_A > 0.0f);
static_assert(::vehicle::current_sensor::PhaseTrip_A > CurrentLimit_A);
static_assert(CurrentOutputMaxAge_us >= ::vehicle::current_sensor::ReadMaxDuration_us);
static_assert(UqLimit_V > 0.0f && PwmBusReference_V > 0.0f);
static_assert(SvpwmLinearMargin > 0.0f && SvpwmLinearMargin <= 1.0f);
static_assert(CurrentFilter_s >= 0.0f);
static_assert(CurrentOutputFilter_s > 0.0f);
static_assert(ForwardSign_M0 == 1.0f || ForwardSign_M0 == -1.0f);
static_assert(ForwardSign_M1 == 1.0f || ForwardSign_M1 == -1.0f);

} // namespace motor
} // namespace vehicle
