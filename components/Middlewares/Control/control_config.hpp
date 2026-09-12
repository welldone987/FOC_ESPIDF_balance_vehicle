#pragma once

#include <cstdint>

namespace vehicle {
namespace control {

inline constexpr unsigned AttitudePeriod_us = 5000U;
inline constexpr float OuterPeriod_s = 0.010f;
inline constexpr float MaximumControlGap_s = 0.010f;
inline constexpr float RadToDeg = 57.295779513f;
inline constexpr float DegToRad = 1.0f / RadToDeg;
inline constexpr float FallAngle_rad = 50.0f * DegToRad;
inline constexpr float Gravity_m_s2 = 9.81f;
inline constexpr float WheelRadius_m = 0.04f;
inline constexpr float WheelTrack_m = 0.18f;
// 命令在控制就绪后接收且年龄小于CommandTimeout_us时有效。
inline constexpr std::int64_t CommandTimeout_us = 300000;
// 旧版满杆25rad/s的80%；斜坡到满杆1s，避免旧限速形成长期瓶颈。
inline constexpr float DriveSpeedLimit_rad_s = 20.0f;
inline constexpr float WheelAcceleration_rad_s2 = 20.0f;
// 旧版转向为电压，无法按比例换算偏航；此处为放开响应的待实测目标。
inline constexpr float YawRateLimit_rad_s = 2.0f;
inline constexpr float YawAcceleration_rad_s2 = 4.0f;
inline constexpr float PitchOffset_rad = 1.8f * DegToRad;
inline constexpr float PitchLimit_rad = 4.8f * DegToRad;
// AttitudeKp_A_per_rad和AttitudeKd_A_per_rad_s属于Control姿态PD，输出平衡电流请求。
inline constexpr float AttitudeKp_A_per_rad = 0.056f * RadToDeg;
inline constexpr float AttitudeKd_A_per_rad_s = 0.01f * RadToDeg;
// SpeedKp_rad_per_rad_s和SpeedKi_rad_per_rad属于Control速度PI，输出目标俯仰角。
inline constexpr float SpeedKp_rad_per_rad_s = 1.0f * DegToRad;
// SpeedKi_rad_per_rad为0时保持速度积分项关闭。
inline constexpr float SpeedKi_rad_per_rad = 0.0f;
inline constexpr float YawKp_A_per_rad_s = 0.2f;
inline constexpr float YawKi_A_per_rad = 0.0f;
inline constexpr float AccelerationFeedforward = 0.0f;
inline constexpr float YawAccelerationFeedforward = 0.0f;
inline constexpr float YawRateFeedforward = 0.0f;

static_assert(WheelRadius_m > 0.0f && WheelTrack_m > 0.0f);
static_assert(AttitudePeriod_us > 0U);

} // namespace control
} // namespace vehicle
