#include "motor_foc_service.hpp"
#include <algorithm>
#include <cmath>
#include "board_pins.hpp"
#include "checked_encoder.hpp"
#include "current_sense.hpp"
#include "current_control.hpp"
#include "encoder_config.hpp"
#include "motor_config.hpp"
#include "svpwm.hpp"
#include "esp_simplefoc.h"
#include "esp_timer.h"
#include "esp_log.h"

namespace vehicle {
namespace motor {
namespace {
/*
 * 电机服务独占两台BLDCDriver3PWM、两只AS5600编码器和公共使能GPIO12。
 * AlignMotor()在启动阶段复用SimpleFOC的initFOC()确定电角度零点。
 * UpdateWheel()逐周期更新机械角、轮速和电角度。
 * CalculateCurrent()完成Iq投影、PI和SVPWM占空比计算。
 */
// encoder_M0和encoder_M1分别读取两轮AS5600机械角。
encoder::CheckedEncoder encoder_M0{I2C_NUM_0, pins::i2c_scl_M0, pins::i2c_sda_M0};
encoder::CheckedEncoder encoder_M1{I2C_NUM_1, pins::i2c_scl_M1, pins::i2c_sda_M1};
// driver_M0和driver_M1输出两轮三相PWM。
// 不传enable引脚：GPIO12为双轮公共电源，不能由任一驱动独立切换。
BLDCDriver3PWM driver_M0{pins::motor_pwm_a_M0, pins::motor_pwm_b_M0, pins::motor_pwm_c_M0};
BLDCDriver3PWM driver_M1{pins::motor_pwm_a_M1, pins::motor_pwm_b_M1, pins::motor_pwm_c_M1};
// alignment_motor_M0和alignment_motor_M1仅用于启动对齐。
// 运行路径不调用move/loopFOC及库电流PI。
BLDCMotor alignment_motor_M0{PolePairs};
BLDCMotor alignment_motor_M1{PolePairs};
// MotorState保存一台电机的对齐结果、滤波状态和PI状态。
struct MotorState {
    // encoder_direction是SimpleFOC对齐得到的编码器方向（+1或-1）。
    float encoder_direction;
    // zero_electrical_angle_rad是对齐得到的零电角度，单位rad。
    float zero_electrical_angle_rad;
    // previous_angle_rad是上一周期的机械角，单位rad。
    float previous_angle_rad;
    // electrical_angle_rad是本周期电角度，单位rad。
    float electrical_angle_rad;
    // velocity_rad_s是滤波后的轮角速度，单位rad/s。
    float velocity_rad_s;
    // iq_filtered_A是低通滤波后的Iq反馈，单位A。
    float iq_filtered_A;
    // uq_filtered_V是输出端Uq滤波值，单位V。
    // 滤波调用当前被注释。
    float uq_filtered_V;
    // current_filter_ready标记iq_filtered_A是否已用首帧初始化。
    bool current_filter_ready;
    // pi保存Iq电流PI的跨周期状态。
    CurrentPiState pi;
};
// state_M0和state_M1分别保存两轮的跨周期状态。
MotorState state_M0{};
MotorState state_M1{};
// initialized为true后ReadWheelState()和RunCurrentControl()才允许运行。
bool initialized = false;
// stopped在故障停机后锁存，禁止再次初始化。
bool stopped = false;
// enable_ready标记公共使能GPIO已配置为输出。
bool enable_ready = false;
// outputs_enabled标记公共使能当前是否为高。
bool outputs_enabled = false;
// driver_ready_M0和driver_ready_M1标记对应驱动是否初始化成功。
bool driver_ready_M0 = false;
bool driver_ready_M1 = false;
// wheel_sample_ready标记本周期ReadWheelState()已建立角度现场。
bool wheel_sample_ready = false;
// encoder_started_us保存本周期编码器读取的开始时刻，单位us。
std::int64_t encoder_started_us = 0;
// previous_encoder_us保存上一周期编码器读取时刻。
// previous_encoder_us为0表示首轮。
std::int64_t previous_encoder_us = 0;
// previous_current_us保存上一周期电流采样开始时刻。
// previous_current_us为0表示首轮。
std::int64_t previous_current_us = 0;
// TwoPi用于单圈差分的角度回绕，单位rad。
constexpr float TwoPi = 6.283185307179586f;

bool AlignMotor(BLDCMotor &motor, encoder::CheckedEncoder &sensor, BLDCDriver3PWM &driver, MotorState &state)
{
    motor.linkSensor(&sensor);
    motor.linkDriver(&driver);
    motor.controller = MotionControlType::torque;
    motor.torque_controller = TorqueControlType::voltage;
    motor.voltage_limit = UqLimit_V;
    motor.voltage_sensor_align = SensorAlignmentVoltage_V;
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    if (!motor.init()) { return false; }
    // 对齐会转动车轮，沿用库的有界扫描。
    // 此阶段没有运行软件电流保护。
    const bool success = motor.initFOC() && sensor.healthy() &&
        std::isfinite(motor.zero_electric_angle) &&
        (motor.sensor_direction == Direction::CW || motor.sensor_direction == Direction::CCW);
    motor.disable();
    if (!success) { return false; }
    state.encoder_direction = static_cast<float>(motor.sensor_direction);
    state.zero_electrical_angle_rad = motor.zero_electric_angle;
    sensor.set_cached(true);
    return true;
}

float UpdateWheel(encoder::CheckedEncoder &sensor, MotorState &state, float forward_sign, float dt_s)
{
    sensor.update();
    const float angle_rad = sensor.getMechanicalAngle();
    // 单圈差分避免累计转数损失精度。
    // 每周期都更新，静止时原始速度确实为0。
    const float delta_rad = std::remainder(angle_rad - state.previous_angle_rad, TwoPi);
    state.previous_angle_rad = angle_rad;
    const float velocity_rad_s = state.encoder_direction * forward_sign * delta_rad / dt_s;
    const float alpha = dt_s / (encoder::WheelVelocityFilter_s + dt_s);
    state.velocity_rad_s += alpha * (velocity_rad_s - state.velocity_rad_s);
    state.electrical_angle_rad = std::remainder(state.encoder_direction * PolePairs *
        angle_rad - state.zero_electrical_angle_rad, TwoPi);
    return state.velocity_rad_s;
}

MotorSample CalculateCurrent(MotorState &state, const current_sensor::PhaseCurrents &phase,
    float requested_A, float forward_sign, float dt_s, PhaseDuty &duty)
{
    const float reference_A = std::clamp(requested_A, -CurrentLimit_A, CurrentLimit_A);
    const float iq_A = ProjectQCurrent(phase.a, phase.b, state.electrical_angle_rad);
    if (!state.current_filter_ready) { state.iq_filtered_A = iq_A; state.current_filter_ready = true; }
    const float alpha = dt_s / (CurrentFilter_s + dt_s);
    state.iq_filtered_A += alpha * (iq_A - state.iq_filtered_A);
    const float voltage_limit_V = std::min(UqLimit_V,
        SvpwmLinearMargin * PwmBusReference_V / 1.7320508075688772f);
    // pi保存本周期的Iq PI输出。
    const auto pi = UpdateCurrentPi(state.pi, forward_sign * reference_A - state.iq_filtered_A,
        dt_s, voltage_limit_V);
    // 暂停PI后Uq低通，直接输出限幅后的PI电压，遥测与实际输出保持一致。
    // const float output_alpha = dt_s / (CurrentOutputFilter_s + dt_s);
    // state.uq_filtered_V += output_alpha * (pi.applied_V - state.uq_filtered_V);
    state.uq_filtered_V = pi.applied_V;
    if (pi.valid) { duty = CalculateSvpwmDuty(state.uq_filtered_V, state.electrical_angle_rad, PwmBusReference_V); }
    // 遥测Iq与Uq转换为车辆前进坐标。
    // 相电流保持桥臂到电机坐标。
    return {reference_A, forward_sign * state.iq_filtered_A, forward_sign * state.uq_filtered_V,
        phase.a, phase.b, phase.c, pi.saturated, requested_A != reference_A};
}
void WritePwm(BLDCDriver3PWM &driver, const PhaseDuty &duty)
{
    // setPwm接收V，不是占空比。
    // 三相驱动限幅保持母线参考，Uq已在PI处限制。
    driver.setPwm(duty.a * PwmBusReference_V, duty.b * PwmBusReference_V,
        duty.c * PwmBusReference_V);
}
} // namespace


esp_err_t Initialize(ErrorInfo *error, BootReporter report)
{
    if (stopped) { return VEHICLE_ERROR(error, ESP_ERR_INVALID_STATE, motor_state, application, 0); }
    if (initialized) { return ESP_OK; }
    esp_err_t rc = PauseOutputs(error);
    if (rc != ESP_OK) { return rc; }
    if (!CurrentHardwareVerified) { return VEHICLE_ERROR(error, ESP_ERR_INVALID_STATE, motor_gate, application, 0); }
    driver_M0.voltage_power_supply = driver_M1.voltage_power_supply = PwmBusReference_V;
    driver_M0.voltage_limit = driver_M1.voltage_limit = PwmBusReference_V;
    if (report) { report(static_cast<std::uint16_t>(ErrorPoint::driver_M0),"BEGIN"); }
    driver_ready_M0 = driver_M0.init(0) != 0;
    if (!driver_ready_M0) { return VEHICLE_ERROR(error, ESP_FAIL, driver_M0, simplefoc, 0); }
    driver_M0.disable();
    if (report) { report(static_cast<std::uint16_t>(ErrorPoint::driver_M0),"OK"); }

    if (report) { report(static_cast<std::uint16_t>(ErrorPoint::driver_M1),"BEGIN"); }
    driver_ready_M1 = driver_M1.init(1) != 0;
    if (!driver_ready_M1) { return VEHICLE_ERROR(error, ESP_FAIL, driver_M1, simplefoc, 0); }
    driver_M1.disable();
    if (report) { report(static_cast<std::uint16_t>(ErrorPoint::driver_M1),"OK"); }

    if (report) { report(static_cast<std::uint16_t>(ErrorPoint::current_unit),"BEGIN"); }
    rc = current_sensor::Initialize(error);
    if (rc != ESP_OK) { return rc; }
    if (report) { report(static_cast<std::uint16_t>(ErrorPoint::current_unit),"OK"); }
    if (report) { report(static_cast<std::uint16_t>(ErrorPoint::encoder_init_M0),"BEGIN"); }
    rc=encoder_M0.Initialize(error,ErrorPoint::encoder_init_M0);
    if (rc != ESP_OK) { return rc; }
    if (report) { report(static_cast<std::uint16_t>(ErrorPoint::encoder_init_M0),"OK"); }
    if (report) { report(static_cast<std::uint16_t>(ErrorPoint::encoder_init_M1),"BEGIN"); }
    rc=encoder_M1.Initialize(error,ErrorPoint::encoder_init_M1);
    if (rc != ESP_OK) { return rc; }

    if (report) { report(static_cast<std::uint16_t>(ErrorPoint::encoder_init_M1),"OK"); }
    rc = gpio_set_level(pins::motor_enable, 1);
    if (rc != ESP_OK) { return VEHICLE_ERROR(error, rc, enable_gpio, esp, rc); }
    if (report) { report(static_cast<std::uint16_t>(ErrorPoint::alignment_M1),"BEGIN"); }
    if (!AlignMotor(alignment_motor_M1, encoder_M1, driver_M1, state_M1)) {
        if (!encoder_M1.healthy()) { return VEHICLE_ERROR(error, encoder_M1.raw_error(), encoder_read_M1, esp, encoder_M1.raw_error()); }
        return VEHICLE_ERROR(error, ESP_FAIL, alignment_M1, simplefoc, 0);
    }
    if (report) { report(static_cast<std::uint16_t>(ErrorPoint::alignment_M1),"OK"); }
    if (report) { report(static_cast<std::uint16_t>(ErrorPoint::alignment_M0),"BEGIN"); }
    if (!AlignMotor(alignment_motor_M0, encoder_M0, driver_M0, state_M0)) {
        if (!encoder_M0.healthy()) { return VEHICLE_ERROR(error, encoder_M0.raw_error(), encoder_read_M0, esp, encoder_M0.raw_error()); }
        return VEHICLE_ERROR(error, ESP_FAIL, alignment_M0, simplefoc, 0);
    }

    if (report) { report(static_cast<std::uint16_t>(ErrorPoint::alignment_M0),"OK"); }
    rc = PauseOutputs(error);
    if (rc != ESP_OK) { return rc; }
    if (!encoder_M0.Refresh()) { return VEHICLE_ERROR(error, encoder_M0.raw_error(), encoder_read_M0, esp, encoder_M0.raw_error()); }
    if (!encoder_M1.Refresh()) { return VEHICLE_ERROR(error, encoder_M1.raw_error(), encoder_read_M1, esp, encoder_M1.raw_error()); }
    state_M0.previous_angle_rad = encoder_M0.angle_rad();
    state_M1.previous_angle_rad = encoder_M1.angle_rad();
    previous_encoder_us = 0;
    initialized = true;
    return ESP_OK;
}

esp_err_t ReadWheelState(WheelState *out, ErrorInfo *error)
{
    wheel_sample_ready = false;
    if (!out || !initialized || stopped) { return VEHICLE_ERROR(error, ESP_ERR_INVALID_STATE, motor_state, application, 0); }
    *out = {};
    const auto started_us = esp_timer_get_time();
    const bool first_sample = previous_encoder_us == 0;
    // 首轮使用标称周期，其后使用上一轮读取时刻到本轮的实际间隔。
    const float dt_s = first_sample ? ControlPeriod_us * 1.0e-6f :
        (started_us - previous_encoder_us) * 1.0e-6f;
    if (!(dt_s > 0.0f && dt_s <= MaximumControlGap_s)) {
        return VEHICLE_ERROR(error, ESP_ERR_INVALID_STATE, wheel_dt, application, 0, dt_s, MaximumControlGap_s, -1, 3);
    }
    if (!encoder_M0.Refresh()) { return VEHICLE_ERROR(error, encoder_M0.raw_error(), encoder_read_M0, esp, encoder_M0.raw_error()); }
    if (!encoder_M1.Refresh()) { return VEHICLE_ERROR(error, encoder_M1.raw_error(), encoder_read_M1, esp, encoder_M1.raw_error()); }
    // 两台电机的轮速在状态副本上计算，成功后一起提交。
    auto next_state_M0 = state_M0;
    auto next_state_M1 = state_M1;
    if (first_sample) {
        next_state_M0.previous_angle_rad = encoder_M0.angle_rad();
        next_state_M1.previous_angle_rad = encoder_M1.angle_rad();
    }
    const float velocity_M0_rad_s = UpdateWheel(encoder_M0, next_state_M0, ForwardSign_M0, dt_s);
    const float velocity_M1_rad_s = UpdateWheel(encoder_M1, next_state_M1, ForwardSign_M1, dt_s);
    if (!std::isfinite(velocity_M0_rad_s) || !std::isfinite(velocity_M1_rad_s)) {
        return VEHICLE_ERROR(error, ESP_ERR_INVALID_RESPONSE, wheel_invalid, application, 0);
    }
    if (esp_timer_get_time() - started_us > encoder::ReadMaxDuration_us) {
        return VEHICLE_ERROR(error, ESP_ERR_TIMEOUT, wheel_age, application, 0, esp_timer_get_time()-started_us, encoder::ReadMaxDuration_us, -1, 3, 1);
    }
    state_M0=next_state_M0; state_M1=next_state_M1;
    encoder_started_us=started_us; previous_encoder_us=started_us;
    wheel_sample_ready = true;
    *out = {velocity_M0_rad_s, velocity_M1_rad_s, true};
    return ESP_OK;
}

esp_err_t RunCurrentControl(const CurrentCommand &command, CurrentFeedback *out, ErrorInfo *error, CurrentTiming *timing)
{
    // trace指向调用者提供的计时结构。
    // 调用者省略时使用局部变量。
    CurrentTiming local{};
    auto &trace = timing ? *timing : local;
    trace = {};
    if (!out || !initialized || stopped || !wheel_sample_ready) { return VEHICLE_ERROR(error, ESP_ERR_INVALID_STATE, motor_state, application, 0); }
    *out = {};
    wheel_sample_ready = false;
    if (!std::isfinite(command.target_M0_A) || !std::isfinite(command.target_M1_A)) {
        return VEHICLE_ERROR(error, ESP_ERR_INVALID_ARG, command_invalid, application, 0);
    }
    // sample保存本周期四路ADC采样的相电流和采样时刻。
    current_sensor::Sample sample{};
    trace.stage=CurrentStage::adc;
    const auto adc_started_us=esp_timer_get_time();
    const auto rc = current_sensor::Read(&sample, error);
    const auto math_started_us=esp_timer_get_time();
    trace.adc_us=math_started_us-adc_started_us;
    if (rc != ESP_OK) { return rc; }
    trace.stage=CurrentStage::math;
    // 首轮使用标称周期，其后使用电流批次开始时间之差。
    const float dt_s = previous_current_us == 0 ? ControlPeriod_us * 1.0e-6f :
        (sample.started_us - previous_current_us) * 1.0e-6f;
    if (!(dt_s > 0.0f && dt_s <= MaximumControlGap_s)) {
        return VEHICLE_ERROR(error, ESP_ERR_INVALID_STATE, current_dt, application, 0, dt_s, MaximumControlGap_s, -1, 3);
    }
    // 两台电机的电流计算在状态副本上进行，成功后一起提交。
    auto next_state_M0=state_M0; auto next_state_M1=state_M1;
    // duty_M0和duty_M1保存本周期三相占空比。
    PhaseDuty duty_M0{}, duty_M1{};
    const auto sample_M0 = CalculateCurrent(next_state_M0, sample.phases_a[0], command.target_M0_A,
        ForwardSign_M0, dt_s, duty_M0);
    const auto sample_M1 = CalculateCurrent(next_state_M1, sample.phases_a[1], command.target_M1_A,
        ForwardSign_M1, dt_s, duty_M1);
    if (!duty_M0.valid) { return VEHICLE_ERROR(error, ESP_ERR_INVALID_RESPONSE, pi_svpwm_M0, application, 0); }
    if (!duty_M1.valid) { return VEHICLE_ERROR(error, ESP_ERR_INVALID_RESPONSE, pi_svpwm_M1, application, 0); }
    auto checked_us=esp_timer_get_time();
    trace.math_us=checked_us-math_started_us;
    trace.stage=CurrentStage::before_pwm;
    trace.encoder_age_us=checked_us-encoder_started_us;
    trace.current_age_us=checked_us-sample.started_us;
    if (trace.encoder_age_us > encoder::OutputMaxAge_us) {
        return VEHICLE_ERROR(error, ESP_ERR_TIMEOUT, output_age, application, 0,
            trace.encoder_age_us, encoder::OutputMaxAge_us, -1, 3, 1);
    }
    if (trace.current_age_us > CurrentOutputMaxAge_us) {
        return VEHICLE_ERROR(error, ESP_ERR_TIMEOUT, current_output_age, application, 0,
            trace.current_age_us, CurrentOutputMaxAge_us, -1, 3, 1);
    }
    trace.stage=CurrentStage::pwm;
    const auto pwm_started_us=esp_timer_get_time();
    WritePwm(driver_M0, duty_M0); WritePwm(driver_M1, duty_M1);
    checked_us=esp_timer_get_time();
    trace.pwm_us=checked_us-pwm_started_us;
    trace.stage=CurrentStage::after_pwm;
    trace.encoder_age_us=checked_us-encoder_started_us;
    trace.current_age_us=checked_us-sample.started_us;
    if (trace.encoder_age_us > encoder::OutputMaxAge_us) {
        return VEHICLE_ERROR(error, ESP_ERR_TIMEOUT, output_age, application, 0,
            trace.encoder_age_us, encoder::OutputMaxAge_us, -1, 3, 1);
    }
    if (trace.current_age_us > CurrentOutputMaxAge_us) {
        return VEHICLE_ERROR(error, ESP_ERR_TIMEOUT, current_output_age, application, 0,
            trace.current_age_us, CurrentOutputMaxAge_us, -1, 3, 1);
    }
    trace.stage=CurrentStage::enable;
    if (!outputs_enabled) {
        const auto enabled = gpio_set_level(pins::motor_enable, 1);
        if (enabled != ESP_OK) { return VEHICLE_ERROR(error, enabled, enable_gpio, esp, enabled); }
        outputs_enabled=true;
    }
    state_M0=next_state_M0; state_M1=next_state_M1; previous_current_us=sample.started_us;
    *out = {sample_M0, sample_M1, dt_s, esp_timer_get_time()-sample.started_us, true};
    trace.stage=CurrentStage::complete;
    return ESP_OK;
}
esp_err_t InhibitOutputs(ErrorInfo *error)
{
    const auto level = gpio_set_level(pins::motor_enable, 0);
    esp_err_t direction=ESP_OK;
    if (!enable_ready) {
        direction=gpio_set_direction(pins::motor_enable, GPIO_MODE_OUTPUT);
        enable_ready=direction == ESP_OK;
    }
    if (driver_ready_M0) { driver_M0.disable(); }
    if (driver_ready_M1) { driver_M1.disable(); }
    outputs_enabled=false;
    if (level != ESP_OK) { return VEHICLE_ERROR(error, level, disable_gpio, esp, level); }
    if (direction != ESP_OK) { return VEHICLE_ERROR(error, direction, disable_gpio, esp, direction); }
    return ESP_OK;
}
esp_err_t PauseOutputs(ErrorInfo *error)
{
    const auto rc = InhibitOutputs(error);
    state_M0.pi = state_M1.pi = {};
    state_M0.iq_filtered_A = state_M1.iq_filtered_A = 0.0f;
    state_M0.uq_filtered_V = state_M1.uq_filtered_V = 0.0f;
    state_M0.current_filter_ready = state_M1.current_filter_ready = false;
    previous_current_us=0; wheel_sample_ready=false;
    return rc;
}
esp_err_t DisableOutputs(ErrorInfo *error)
{
    stopped=true;
    return PauseOutputs(error);
}
} // namespace motor
} // namespace vehicle
