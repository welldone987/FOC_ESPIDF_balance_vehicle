#pragma once
#include <cstdint>
#include "error_info.hpp"
#include "motor_timing.hpp"

namespace vehicle {
namespace motor {

/*
 * 电机服务组合编码器角度、电流采样、Iq PI和SVPWM输出。
 * readWheelState()建立本周期角度现场，runCurrentControl()随后完成唯一正常PWM写入。
 */
struct WheelState {
    float left_velocity_rad_s;
    float right_velocity_rad_s;
    bool valid;
};
// CurrentCommand保存车辆前进坐标下的左右目标电流，单位A。
struct CurrentCommand { float left_target_a; float right_target_a; };
struct MotorSample {
    float iq_reference_a;
    float iq_measured_a;
    float uq_applied_v;
    float phase_a_a;
    float phase_b_a;
    float phase_c_a;
    bool voltage_saturated;
    bool reference_limited;
};
struct CurrentFeedback {
    MotorSample left;
    MotorSample right;
    float dt_s;
    std::int64_t sample_age_us;
    bool valid;
};
// 硬件确认门通过后才进行有运动的对齐；完成后公共使能关闭。
using BootReporter = void (*)(std::uint16_t step, const char *state);
esp_err_t initialize(ErrorInfo *error=nullptr, BootReporter report=nullptr);
esp_err_t readWheelState(WheelState *out, ErrorInfo *error=nullptr);
// 必须在本周期readWheelState之后调用，使用本周期外环目标；唯一正常PWM写入点。
esp_err_t runCurrentControl(const CurrentCommand &command, CurrentFeedback *out, ErrorInfo *error=nullptr,
    CurrentTiming *timing=nullptr);
// 普通停机可恢复；故障停机锁存直到复位。两者均关闭公共使能并清零PI。
esp_err_t inhibitOutputs(ErrorInfo *error=nullptr);
esp_err_t pauseOutputs(ErrorInfo *error=nullptr);
esp_err_t disableOutputs(ErrorInfo *error=nullptr);
} // namespace motor
} // namespace vehicle
