#include "balance_controller.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include "control_config.hpp"

namespace vehicle {
namespace control {
namespace {
float LimitSymmetric(float value, float limit) { return std::clamp(value, -limit, limit); }
float AdvanceReference(float &reference, float target, float rate, float dt_s)
{
    const float delta = LimitSymmetric(target - reference, rate * dt_s);
    reference += delta;
    return delta / dt_s;
}
// 输出饱和时仅允许使积分退出饱和的方向；下游无余量时冻结积分。
float PiControl(float error, float kp, float ki, float feedforward, float limit,
         float dt_s, bool downstream_saturated, float &integral)
{
    const float candidate = LimitSymmetric(integral + ki * error * dt_s, limit);
    const float requested = kp * error + candidate + feedforward;
    if (!downstream_saturated &&
        (std::abs(requested) <= limit || requested * error <= 0.0f)) {
        integral = candidate;
    }
    return LimitSymmetric(kp * error + integral + feedforward, limit);
}
} // namespace
void Initialize(ControllerState &state) { state = {}; }
void ObserveCurrentSaturation(ControllerState &state, bool saturated)
{
    state.balance_saturated |= saturated;
    state.turn_saturated |= saturated;
}
ControlOutput Update(ControllerState &state, const ControlInput &input, float dt_s)
{
    if (!(dt_s > 0.0f && dt_s <= MaximumControlGap_s) ||
        !std::isfinite(input.velocity_M0_rad_s) || !std::isfinite(input.velocity_M1_rad_s) ||
        !std::isfinite(input.pitch_rad) || !std::isfinite(input.pitch_rate_rad_s) ||
        !std::isfinite(input.throttle_velocity_rad_s) || !std::isfinite(input.yaw_rate_rad_s)) {
        Initialize(state);
        return {};
    }
    // 本地平衡使能独立于遥控；未ARM时闭合零速/零偏航环。
    if (!input.balancing) { Initialize(state); return {0, 0, 0, 0, 0, true}; }
    state.outer_elapsed_s += dt_s;
    if (state.outer_elapsed_s >= OuterPeriod_s) {
        const float outer_dt_s = state.outer_elapsed_s;
        state.outer_elapsed_s = 0.0f;
        const float acceleration = AdvanceReference(state.speed_reference_rad_s,
            input.driving ? LimitSymmetric(input.throttle_velocity_rad_s, DriveSpeedLimit_rad_s) : 0.0f,
            WheelAcceleration_rad_s2, outer_dt_s);
        const float yaw_acceleration = AdvanceReference(state.yaw_reference_rad_s,
            input.driving ? LimitSymmetric(input.yaw_rate_rad_s, YawRateLimit_rad_s) : 0.0f,
            YawAcceleration_rad_s2, outer_dt_s);
        const float speed = (input.velocity_M0_rad_s + input.velocity_M1_rad_s) * 0.5f;
        const float pitch_ff = AccelerationFeedforward *
            acceleration * WheelRadius_m / Gravity_m_s2;
        state.target_pitch_rad = PiControl(state.speed_reference_rad_s - speed,
            SpeedKp_rad_per_rad_s, SpeedKi_rad_per_rad, pitch_ff, PitchLimit_rad,
            outer_dt_s, state.balance_saturated, state.speed_integral_rad);
        const float yaw = WheelRadius_m / WheelTrack_m *
            (input.velocity_M1_rad_s - input.velocity_M0_rad_s);
        const float yaw_ff = YawAccelerationFeedforward * yaw_acceleration +
            YawRateFeedforward * state.yaw_reference_rad_s;
        state.turn_request_A = PiControl(state.yaw_reference_rad_s - yaw,
            YawKp_A_per_rad_s, YawKi_A_per_rad, yaw_ff, std::numeric_limits<float>::max(),
            outer_dt_s, state.turn_saturated, state.yaw_integral_A);
        state.balance_saturated = false;
        state.turn_saturated = false;
    }
    // 正公共电流使轮子前进、车体后倾；使用实测角速度而非目标角微分。
    const float request = AttitudeKp_A_per_rad *
        (input.pitch_rad - PitchOffset_rad - state.target_pitch_rad) +
        AttitudeKd_A_per_rad_s * input.pitch_rate_rad_s;
    if (!std::isfinite(request) || !std::isfinite(state.target_pitch_rad) || !std::isfinite(state.turn_request_A)) {
        Initialize(state);
        return {};
    }
    // 仅混合物理电流请求；每轮±1A统一由BSP电流环入口限制并反馈饱和。
    const float balance = request;
    const float turn = state.turn_request_A;
    return {state.target_pitch_rad + PitchOffset_rad, balance, turn,
            balance - turn, balance + turn,
            std::isfinite(balance - turn) && std::isfinite(balance + turn)};
}
} // namespace control
} // namespace vehicle
