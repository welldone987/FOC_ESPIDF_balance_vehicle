#include "balance_controller.hpp"
#include <algorithm>
#include <cmath>
#include "control_config.hpp"
#include "motor_config.hpp"

namespace vehicle {
namespace control {
namespace {
float limited(float value, float limit) { return std::clamp(value, -limit, limit); }
float advance(float &reference, float target, float rate, float dt)
{
    const float delta = limited(target - reference, rate * dt);
    reference += delta;
    return delta / dt;
}
// 输出饱和时仅允许使积分退出饱和的方向；下游无余量时冻结积分。
float pi(float error, float kp, float ki, float feedforward, float limit,
         float dt, bool downstream_saturated, float &integral)
{
    const float candidate = limited(integral + ki * error * dt, limit);
    const float requested = kp * error + candidate + feedforward;
    if (!downstream_saturated &&
        (std::abs(requested) <= limit || requested * error <= 0.0f)) {
        integral = candidate;
    }
    return limited(kp * error + integral + feedforward, limit);
}
} // namespace
void initialize(ControllerState &state) { state = {}; }
void observeCurrentSaturation(ControllerState &state, bool saturated)
{
    state.balance_saturated |= saturated;
    state.turn_saturated |= saturated;
}
ControlOutput update(ControllerState &s, const ControlInput &in, float dt)
{
    if (!(dt > 0.0f && dt <= config::kMaximumControlGapS) ||
        !std::isfinite(in.left_velocity_rad_s) || !std::isfinite(in.right_velocity_rad_s) ||
        !std::isfinite(in.pitch_rad) || !std::isfinite(in.pitch_rate_rad_s) ||
        !std::isfinite(in.throttle_velocity_rad_s) || !std::isfinite(in.yaw_rate_rad_s)) {
        initialize(s);
        return {};
    }
    // 本地平衡使能独立于遥控；未ARM时闭合零速/零偏航环。
    if (!in.balancing) { initialize(s); return {0, 0, 0, 0, 0, true}; }
    s.outer_elapsed_s += dt;
    if (s.outer_elapsed_s >= config::kOuterPeriodS) {
        const float outer_dt = s.outer_elapsed_s;
        s.outer_elapsed_s = 0.0f;
        const float acceleration = advance(s.speed_reference_rad_s,
            in.driving ? limited(in.throttle_velocity_rad_s, config::kDriveSpeedLimitRadS) : 0.0f,
            config::kWheelAccelerationRadS2, outer_dt);
        const float yaw_acceleration = advance(s.yaw_reference_rad_s,
            in.driving ? limited(in.yaw_rate_rad_s, config::kYawRateLimitRadS) : 0.0f,
            config::kYawAccelerationRadS2, outer_dt);
        const float speed = (in.left_velocity_rad_s + in.right_velocity_rad_s) * 0.5f;
        const float pitch_ff = config::kAccelerationFeedforward *
            acceleration * config::kWheelRadiusM / config::kGravityMps2;
        s.target_pitch_rad = pi(s.speed_reference_rad_s - speed,
            config::kSpeedKpRadPerRadS, config::kSpeedKiRadPerRad, pitch_ff, config::kPitchLimitRad,
            outer_dt, s.balance_saturated, s.speed_integral_rad);
        const float yaw = config::kWheelRadiusM / config::kWheelTrackM *
            (in.right_velocity_rad_s - in.left_velocity_rad_s);
        const float yaw_ff = config::kYawAccelerationFeedforward * yaw_acceleration +
            config::kYawRateFeedforward * s.yaw_reference_rad_s;
        s.turn_request_a = pi(s.yaw_reference_rad_s - yaw,
            config::kYawKpAPerRadS, config::kYawKiAPerRad, yaw_ff, motor::config::kCurrentLimitA,
            outer_dt, s.turn_saturated, s.yaw_integral_a);
        s.balance_saturated = false;
        s.turn_saturated = false;
    }
    // 正公共电流使轮子前进、车体后倾；使用实测角速度而非目标角微分。
    const float request = config::kAttitudeKpAPerRad *
        (in.pitch_rad - config::kPitchOffsetRad - s.target_pitch_rad) +
        config::kAttitudeKdAPerRadS * in.pitch_rate_rad_s;
    if (!std::isfinite(request) || !std::isfinite(s.target_pitch_rad) || !std::isfinite(s.turn_request_a)) {
        initialize(s);
        return {};
    }
    const float balance = limited(request, motor::config::kCurrentLimitA);
    const float turn = limited(s.turn_request_a, motor::config::kCurrentLimitA - std::abs(balance));
    // 汇总整个外环窗口的饱和，不能仅保留最后一次结果。
    s.balance_saturated |= std::abs(request) > motor::config::kCurrentLimitA;
    s.turn_saturated |= std::abs(s.turn_request_a) > motor::config::kCurrentLimitA - std::abs(balance);
    return {s.target_pitch_rad + config::kPitchOffsetRad, balance, turn,
            balance - turn, balance + turn,
            std::isfinite(balance) && std::isfinite(turn)};
}
} // namespace control
} // namespace vehicle
