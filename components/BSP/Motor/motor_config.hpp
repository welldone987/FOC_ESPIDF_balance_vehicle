#pragma once

#include <cstdint>
#include "current_sensor_config.hpp"

namespace vehicle {
namespace motor {

// ControlRate_Hz是Iq电流环的目标频率，单位Hz。
inline constexpr std::uint32_t ControlRate_Hz = 500U;
// ControlPeriod_us是首轮和缺省使用的标称电流周期，单位us。
inline constexpr std::uint64_t ControlPeriod_us = 1000000ULL / ControlRate_Hz;
// MaximumControlGap_s限制相邻控制周期的最大间隔，单位s。
inline constexpr float MaximumControlGap_s = 0.010f;
// PolePairs是每台BLDC电机的极对数。
inline constexpr int PolePairs = 7;
// PwmBusReference_V是SVPWM和驱动限幅使用的调制参考电压，单位V。
inline constexpr float PwmBusReference_V = 12.0f;
// SensorAlignmentVoltage_V是启动对齐施加的相电压，单位V。
inline constexpr float SensorAlignmentVoltage_V = 2.0f;
// ForwardSign_M0和ForwardSign_M1把车辆前进坐标映射到左右电机坐标。
inline constexpr float ForwardSign_M0 = -1.0f;
inline constexpr float ForwardSign_M1 = -1.0f;
// CurrentHardwareVerified控制是否允许驱动初始化和有运动的电机对齐。
inline constexpr bool CurrentHardwareVerified = true;
// CurrentLimit_A限制每轮最终Iq目标，单位A。
inline constexpr float CurrentLimit_A = 1.0f;
// CurrentOutputMaxAge_us限制电流采样时刻到PWM提交的最大年龄，单位us。
inline constexpr std::int64_t CurrentOutputMaxAge_us = 2000;
// IqKp_V_per_A和IqKi_V_per_A_s属于BSP/Motor的Iq电流PI。
inline constexpr float IqKp_V_per_A = 5.0f;
inline constexpr float IqKi_V_per_A_s = 200.0f;
// CurrentFilter_s是电流PI输入端Iq一阶低通滤波时间常数，单位s。
inline constexpr float CurrentFilter_s = 0.0005f;
// 旧版6V的80%。
// 电流目标仍独立限制为每轮±1A。
inline constexpr float UqLimit_V = 4.8f;
// SvpwmLinearMargin限制SVPWM线性调制区占母线参考的比例。
inline constexpr float SvpwmLinearMargin = 0.9f;

static_assert(CurrentLimit_A > 0.0f);
static_assert(::vehicle::current_sensor::PhaseTrip_A > CurrentLimit_A);
static_assert(CurrentOutputMaxAge_us >= ::vehicle::current_sensor::ReadMaxDuration_us);
static_assert(UqLimit_V > 0.0f && PwmBusReference_V > 0.0f);
static_assert(SvpwmLinearMargin > 0.0f && SvpwmLinearMargin <= 1.0f);
static_assert(CurrentFilter_s >= 0.0f);
static_assert(ForwardSign_M0 == 1.0f || ForwardSign_M0 == -1.0f);
static_assert(ForwardSign_M1 == 1.0f || ForwardSign_M1 == -1.0f);

} // namespace motor
} // namespace vehicle
