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
// WheelState保存本周期两轮角速度和采样有效性。
struct WheelState {
    // velocity_M0_rad_s和velocity_M1_rad_s是滤波后的轮角速度，单位rad/s。
    float velocity_M0_rad_s;
    float velocity_M1_rad_s;
    // valid标记本周期轮速是否可用于控制。
    bool valid;
};
// CurrentCommand保存车辆前进坐标下的左右目标电流，单位A。
struct CurrentCommand { float target_M0_A; float target_M1_A; };
// MotorSample保存一台电机的Iq控制结果和相电流。
struct MotorSample {
    // iq_reference_A是限幅后的Iq目标，iq_measured_A是滤波后的Iq反馈，单位A。
    float iq_reference_A;
    float iq_measured_A;
    // uq_applied_V是实际施加的q轴电压，单位V。
    float uq_applied_V;
    // phase_a_A、phase_b_A和phase_c_A是三相采样电流，单位A。
    float phase_a_A;
    float phase_b_A;
    float phase_c_A;
    // voltage_saturated标记PI输出达到电压上限。
    bool voltage_saturated;
    // reference_limited标记目标被CurrentLimit_A裁剪。
    bool reference_limited;
};
// CurrentFeedback把两台电机的采样结果和本周期时间信息交给控制任务。
struct CurrentFeedback {
    // sample_M0和sample_M1保存两轮结果。
    MotorSample sample_M0;
    MotorSample sample_M1;
    // dt_s是本次与上次电流采样的实际间隔，单位s。
    float dt_s;
    // sample_age_us是采样开始到结果提交的年龄，单位us。
    std::int64_t sample_age_us;
    // valid标记本周期反馈是否有效。
    bool valid;
};
// 硬件确认门通过后才进行有运动的对齐。
// 对齐完成后公共使能关闭。
using BootReporter = void (*)(std::uint16_t step, const char *state);
esp_err_t Initialize(ErrorInfo *error=nullptr, BootReporter report=nullptr);
esp_err_t ReadWheelState(WheelState *out, ErrorInfo *error=nullptr);
// 必须在本周期ReadWheelState之后调用，使用本周期外环目标。
// RunCurrentControl()是唯一正常PWM写入点。
esp_err_t RunCurrentControl(const CurrentCommand &command, CurrentFeedback *out, ErrorInfo *error=nullptr,
    CurrentTiming *timing=nullptr);
// 普通停机可恢复，故障停机锁存直到复位。
// 两者均关闭公共使能并清零PI。
esp_err_t InhibitOutputs(ErrorInfo *error=nullptr);
esp_err_t PauseOutputs(ErrorInfo *error=nullptr);
esp_err_t DisableOutputs(ErrorInfo *error=nullptr);
} // namespace motor
} // namespace vehicle
