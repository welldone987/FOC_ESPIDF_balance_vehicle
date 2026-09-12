#pragma once

namespace vehicle {
namespace control {
namespace config {

inline constexpr unsigned kAttitudePeriodUs = 5000U;
inline constexpr float kOuterPeriodS = 0.010f;
inline constexpr float kMaximumControlGapS = 0.010f;
inline constexpr float kRadToDeg = 57.295779513f;
inline constexpr float kDegToRad = 1.0f / kRadToDeg;
inline constexpr float kFallAngleRad = 50.0f * kDegToRad;
inline constexpr float kGravityMps2 = 9.81f;
inline constexpr float kWheelRadiusM = 0.04f;
inline constexpr float kWheelTrackM = 0.18f;
// 旧版满杆25rad/s的80%；斜坡到满杆1s，避免旧限速形成长期瓶颈。
inline constexpr float kDriveSpeedLimitRadS = 20.0f;
inline constexpr float kWheelAccelerationRadS2 = 20.0f;
// 旧版转向为电压，无法按比例换算偏航；此处为放开响应的待实测目标。
inline constexpr float kYawRateLimitRadS = 2.0f;
inline constexpr float kYawAccelerationRadS2 = 4.0f;
inline constexpr float kPitchOffsetRad = 1.8f * kDegToRad;
inline constexpr float kPitchLimitRad = 4.8f * kDegToRad;
// kAttitudeKpAPerRad和kAttitudeKdAPerRadS属于Control姿态PD，输出平衡电流请求。
inline constexpr float kAttitudeKpAPerRad = 0.056f * kRadToDeg;
inline constexpr float kAttitudeKdAPerRadS = 0.01f * kRadToDeg;
// kSpeedKpRadPerRadS和kSpeedKiRadPerRad属于Control速度PI，输出目标俯仰角。
inline constexpr float kSpeedKpRadPerRadS = 1.0f * kDegToRad;
// kSpeedKiRadPerRad为0时保持速度积分项关闭。
inline constexpr float kSpeedKiRadPerRad = 0.0f;
inline constexpr float kYawKpAPerRadS = 0.2f;
inline constexpr float kYawKiAPerRad = 0.0f;
inline constexpr float kAccelerationFeedforward = 0.0f;
inline constexpr float kYawAccelerationFeedforward = 0.0f;
inline constexpr float kYawRateFeedforward = 0.0f;

static_assert(kWheelRadiusM > 0.0f && kWheelTrackM > 0.0f);
static_assert(kAttitudePeriodUs > 0U);

} // namespace config
} // namespace control
} // namespace vehicle
