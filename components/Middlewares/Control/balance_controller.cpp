#include "balance_controller.hpp"

#include <algorithm>

#include "esp_timer.h"
#include "vehicle_config.hpp"

namespace vehicle {
namespace control {

BalanceController::Pid::Pid(float p, float i, float d, float ramp, float limit)
    : p_(p),
      i_(i),
      d_(d),
      ramp_(ramp),
      limit_(limit),
      error_prev_(0.0f),
      output_prev_(0.0f),
      integral_prev_(0.0f),
      timestamp_prev_us_(esp_timer_get_time())
{
}

float BalanceController::Pid::update(float error)
{
    const std::int64_t now_us = esp_timer_get_time();
    float dt = static_cast<float>(now_us - timestamp_prev_us_) * 1.0e-6f;
    if (dt <= 0.0f || dt > 0.5f) {
        dt = 1.0e-3f;
    }
    timestamp_prev_us_ = now_us;

    const float proportional = p_ * error;
    float integral = integral_prev_ + i_ * dt * 0.5f * (error + error_prev_);
    integral = std::clamp(integral, -limit_, limit_);
    const float derivative = d_ * (error - error_prev_) / dt;

    float output = proportional + integral + derivative;
    output = std::clamp(output, -limit_, limit_);

    if (ramp_ > 0.0f) {
        const float output_rate = (output - output_prev_) / dt;
        if (output_rate > ramp_) {
            output = output_prev_ + ramp_ * dt;
        } else if (output_rate < -ramp_) {
            output = output_prev_ - ramp_ * dt;
        }
    }

    integral_prev_ = integral;
    output_prev_ = output;
    error_prev_ = error;
    return output;
}

BalanceController::LowPass::LowPass(float time_constant_s)
    : time_constant_s_(time_constant_s),
      previous_output_(0.0f),
      timestamp_prev_us_(esp_timer_get_time())
{
}

float BalanceController::LowPass::update(float value)
{
    const std::int64_t now_us = esp_timer_get_time();
    float dt = static_cast<float>(now_us - timestamp_prev_us_) * 1.0e-6f;

    if (dt < 0.0f) {
        dt = 1.0e-3f;
    } else if (dt > 0.3f) {
        previous_output_ = value;
        timestamp_prev_us_ = now_us;
        return value;
    }
    timestamp_prev_us_ = now_us;

    const float alpha = time_constant_s_ / (time_constant_s_ + dt);
    const float output = alpha * previous_output_ + (1.0f - alpha) * value;
    previous_output_ = output;
    return output;
}

BalanceController::BalanceController()
    : balance_pid_(
          config::kBalancePidP,
          config::kBalancePidI,
          config::kBalancePidD,
          config::kBalancePidRamp,
          config::kBalancePidLimitV),
      speed_pid_(
          config::kSpeedPidP,
          config::kSpeedPidI,
          config::kSpeedPidD,
          config::kSpeedPidRamp,
          config::kSpeedPidLimitDeg),
      pitch_command_filter_(config::kPitchCommandFilterTfS),
      throttle_filter_(config::kThrottleFilterTfS),
      steering_filter_(config::kSteeringFilterTfS)
{
}

ControlOutput BalanceController::update(const ControlInput &input)
{
    const float average_velocity_rad_s =
        (config::kMotor0Direction * input.left_velocity_rad_s +
         config::kMotor1Direction * input.right_velocity_rad_s) /
        2.0f;

    const float filtered_throttle_rad_s =
        throttle_filter_.update(input.throttle_velocity_rad_s);
    const float target_pitch_deg = pitch_command_filter_.update(
        speed_pid_.update(average_velocity_rad_s - filtered_throttle_rad_s));

    const float pitch_error_deg = config::kPitchOffsetDeg - input.pitch_deg;
    const float balance_voltage_v =
        balance_pid_.update(pitch_error_deg + target_pitch_deg);
    const float steering_voltage_v =
        steering_filter_.update(input.steering_voltage_v);

    return ControlOutput{
        target_pitch_deg,
        balance_voltage_v,
        steering_voltage_v,
        config::kMotor0Direction * (balance_voltage_v + steering_voltage_v),
        config::kMotor1Direction * (balance_voltage_v - steering_voltage_v),
    };
}

} // namespace control
} // namespace vehicle
