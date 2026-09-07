#include "balance_controller.hpp"

#include <algorithm>

#include "esp_timer.h"
#include "vehicle_config.hpp"

namespace vehicle {
namespace control {
namespace {

/*
 * 控制计算在单次调用中完成速度外环、姿态内环、命令滤波和左右电压混合。
 * PidState与LowPassState保存跨控制周期的历史误差、输出和时间戳。
 */

// updatePid()按实测调用间隔更新PID积分、微分、输出限幅和输出斜率限幅。
float updatePid(PidState &state, float error)
{
    const std::int64_t now_us = esp_timer_get_time();
    // dt由相邻调用时间戳得到，单位s；异常间隔使用1ms保护积分和微分计算。
    float dt = static_cast<float>(now_us - state.previous_time_us) * 1.0e-6f;
    if (dt <= 0.0f || dt > 0.5f) {
        dt = 1.0e-3f;
    }
    state.previous_time_us = now_us;

    // 梯形积分先限幅，再与比例和微分项合成为控制输出。
    float integral = state.previous_integral +
                     state.i * dt * 0.5f * (error + state.previous_error);
    integral = std::clamp(integral, -state.limit, state.limit);
    float output = state.p * error + integral +
                   state.d * (error - state.previous_error) / dt;
    output = std::clamp(output, -state.limit, state.limit);

    // ramp限制输出相对上一周期的变化率，单位为输出量/s。
    if (state.ramp > 0.0f) {
        const float output_rate = (output - state.previous_output) / dt;
        if (output_rate > state.ramp) {
            output = state.previous_output + state.ramp * dt;
        } else if (output_rate < -state.ramp) {
            output = state.previous_output - state.ramp * dt;
        }
    }

    state.previous_integral = integral;
    state.previous_output = output;
    state.previous_error = error;
    return output;
}

// updateLowPass()按实测调用间隔更新一阶低通输出。
float updateLowPass(LowPassState &state, float value)
{
    const std::int64_t now_us = esp_timer_get_time();
    float dt = static_cast<float>(now_us - state.previous_time_us) * 1.0e-6f;
    if (dt < 0.0f) {
        dt = 1.0e-3f;
    } else if (dt > 0.3f) {
        state.previous_output = value;
        state.previous_time_us = now_us;
        return value;
    }
    state.previous_time_us = now_us;

    // alpha由时间常数和采样间隔决定，时间常数与dt的单位均为s。
    const float alpha = state.time_constant_s / (state.time_constant_s + dt);
    state.previous_output = alpha * state.previous_output + (1.0f - alpha) * value;
    return state.previous_output;
}

} // namespace

void initialize(ControllerState &state)
{
    // 所有历史状态从同一时刻开始，避免首次调用使用未定义的旧时间间隔。
    const std::int64_t now_us = esp_timer_get_time();
    state = ControllerState{
        {config::kBalancePidP, config::kBalancePidI, config::kBalancePidD,
         config::kBalancePidRamp, config::kBalancePidLimitV, 0.0f, 0.0f, 0.0f, now_us},
        {config::kSpeedPidP, config::kSpeedPidI, config::kSpeedPidD,
         config::kSpeedPidRamp, config::kSpeedPidLimitDeg, 0.0f, 0.0f, 0.0f, now_us},
        {config::kPitchCommandFilterTfS, 0.0f, now_us},
        {config::kThrottleFilterTfS, 0.0f, now_us},
        {config::kSteeringFilterTfS, 0.0f, now_us},
    };
}

ControlOutput update(ControllerState &state, const ControlInput &input)
{
    // 方向因子把左右轮速统一到车辆前进方向，再计算平均轮速。
    const float average_velocity_rad_s =
        (config::kMotor0Direction * input.left_velocity_rad_s +
         config::kMotor1Direction * input.right_velocity_rad_s) /
        2.0f;
    // filtered_throttle保存低通后的目标轮速，单位rad/s。
    const float filtered_throttle =
        updateLowPass(state.throttle_filter, input.throttle_velocity_rad_s);
    // 速度PID把平均轮速误差转换为目标俯仰角，再经过目标角低通，单位deg。
    const float target_pitch = updateLowPass(
        state.pitch_filter, updatePid(state.speed_pid, average_velocity_rad_s - filtered_throttle));
    // 姿态误差包含校准偏置和速度环目标，balance_voltage单位为V。
    const float balance_voltage = updatePid(
        state.balance_pid, config::kPitchOffsetDeg - input.pitch_deg + target_pitch);
    // steering_voltage保存低通后的转向差分电压，单位V。
    const float steering_voltage =
        updateLowPass(state.steering_filter, input.steering_voltage_v);

    // 左右目标分别叠加或相减转向量，再应用各自电机方向因子。
    return ControlOutput{
        target_pitch,
        balance_voltage,
        steering_voltage,
        config::kMotor0Direction * (balance_voltage + steering_voltage),
        config::kMotor1Direction * (balance_voltage - steering_voltage),
    };
}

} // namespace control
} // namespace vehicle
