#pragma once
#include <cstdint>
#include "error_info.hpp"
#include "motor_timing.hpp"

namespace vehicle {
namespace motor {

/*
 * 电机服务组合编码器角度、电流采样、Iq PI和SVPWM输出。
 * ReadWheelState()建立本周期角度现场，RunCurrentControl()随后完成唯一正常PWM写入。
 */
struct WheelState {
    float velocity_M0_rad_s;
    float velocity_M1_rad_s;
    bool valid;
};
// CurrentCommand保存车辆前进坐标下的左右目标电流，单位A。
struct CurrentCommand { float target_M0_A; float target_M1_A; };
struct MotorSample {
    float iq_reference_A;
    float iq_measured_A;
    float uq_applied_V;
    float phase_a_A;
    float phase_b_A;
    float phase_c_A;
    bool voltage_saturated;
    bool reference_limited;
};
struct CurrentFeedback {
    MotorSample sample_M0;
    MotorSample sample_M1;
    float dt_s;
    std::int64_t sample_age_us;
    bool valid;
};
// 硬件确认门通过后才进行有运动的对齐；完成后公共使能关闭。
using BootReporter = void (*)(std::uint16_t step, const char *state);
esp_err_t Initialize(ErrorInfo *error=nullptr, BootReporter report=nullptr);
esp_err_t ReadWheelState(WheelState *out, ErrorInfo *error=nullptr);
// 必须在本周期ReadWheelState之后调用，使用本周期外环目标；唯一正常PWM写入点。
esp_err_t RunCurrentControl(const CurrentCommand &command, CurrentFeedback *out, ErrorInfo *error=nullptr,
    CurrentTiming *timing=nullptr);
// 普通停机可恢复；故障停机锁存直到复位。两者均关闭公共使能并清零PI。
esp_err_t InhibitOutputs(ErrorInfo *error=nullptr);
esp_err_t PauseOutputs(ErrorInfo *error=nullptr);
esp_err_t DisableOutputs(ErrorInfo *error=nullptr);
} // namespace motor
} // namespace vehicle
