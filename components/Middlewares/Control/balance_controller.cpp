#include "balance_controller.hpp"
#include <algorithm>
#include <cmath>
#include "vehicle_config.hpp"

namespace vehicle::control {
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
ControlOutput update(ControllerState &s, const ControlInput &in, float dt)
{
    if (!(dt > 0.0f && dt <= config::kMaximumControlGapS)) {
        initialize(s);
        return {};
    }
    // 停止/失联清除驾驶轨迹与积分；保留姿态平衡，急停由任务锁存停机。
    if (!in.driving) { initialize(s); }
    s.outer_elapsed_s += dt;
    if (in.driving && s.outer_elapsed_s >= config::kOuterPeriodS) {
        const float outer_dt = s.outer_elapsed_s;
        s.outer_elapsed_s = 0.0f;
        const float acceleration = advance(s.speed_reference,
            limited(in.throttle_velocity_rad_s, config::kDriveSpeedLimitRadS),
            config::kWheelAccelerationRadS2, outer_dt);
        const float yaw_acceleration = advance(s.yaw_reference,
            limited(in.yaw_rate_rad_s, config::kYawRateLimitRadS),
            config::kYawAccelerationRadS2, outer_dt);
        const float speed = (in.left_velocity_rad_s + in.right_velocity_rad_s) * 0.5f;
        const float pitch_ff = config::kAccelerationFeedforward *
            acceleration * config::kWheelRadiusM / config::kGravityMps2 * config::kRadToDeg;
        s.target_pitch_deg = pi(s.speed_reference - speed,
            config::kSpeedKp, config::kSpeedKi, pitch_ff, config::kPitchLimitDeg,
            outer_dt, s.balance_saturated, s.speed_integral);
        const float yaw = config::kWheelRadiusM / config::kWheelTrackM *
            (in.right_velocity_rad_s - in.left_velocity_rad_s);
        const float yaw_ff = config::kYawAccelerationFeedforward * yaw_acceleration +
            config::kYawRateFeedforward * s.yaw_reference;
        s.turn_request_a = pi(s.yaw_reference - yaw,
            config::kYawKp, config::kYawKi, yaw_ff, config::kCurrentLimitA,
            outer_dt, s.turn_saturated, s.yaw_integral);
        s.balance_saturated = false;
        s.turn_saturated = false;
    }
    // 正公共电流使轮子前进、车体后倾；使用实测角速度而非目标角微分。
    const float request = config::kAttitudeKp *
        (in.pitch_deg - config::kPitchOffsetDeg - s.target_pitch_deg) +
        config::kAttitudeKd * in.pitch_rate_deg_s;
    const float balance = limited(request, config::kCurrentLimitA);
    const float turn = limited(s.turn_request_a, config::kCurrentLimitA - std::abs(balance));
    // 汇总整个外环窗口的饱和，不能仅保留最后一次结果。
    s.balance_saturated |= std::abs(request) > config::kCurrentLimitA;
    s.turn_saturated |= std::abs(s.turn_request_a) > config::kCurrentLimitA - std::abs(balance);
    return {s.target_pitch_deg + config::kPitchOffsetDeg, balance, turn,
            config::kMotor0Direction * (balance - turn),
            config::kMotor1Direction * (balance + turn)};
}
} // namespace vehicle::control
