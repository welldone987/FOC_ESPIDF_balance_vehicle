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
encoder::CheckedEncoder left_sensor{I2C_NUM_0, board::pins::kI2c0Scl, board::pins::kI2c0Sda};
encoder::CheckedEncoder right_sensor{I2C_NUM_1, board::pins::kI2c1Scl, board::pins::kI2c1Sda};
// 不传enable引脚：GPIO12为双轮公共电源，不能由任一驱动独立切换。
BLDCDriver3PWM left_driver{board::pins::kMotor0PwmA, board::pins::kMotor0PwmB, board::pins::kMotor0PwmC};
BLDCDriver3PWM right_driver{board::pins::kMotor1PwmA, board::pins::kMotor1PwmB, board::pins::kMotor1PwmC};
// 库电机对象仅用于启动对齐；运行路径不调用move/loopFOC及库电流PI。
BLDCMotor left_alignment{config::kPolePairs};
BLDCMotor right_alignment{config::kPolePairs};
struct MotorState {
    float encoder_direction;
    float zero_electrical_angle_rad;
    float previous_angle_rad;
    float electrical_angle_rad;
    float velocity_rad_s;
    float iq_filtered_a;
    float uq_filtered_v;
    bool current_filter_ready;
    CurrentPiState pi;
};
MotorState left_state{};
MotorState right_state{};
bool initialized = false;
bool stopped = false;
bool enable_ready = false;
bool outputs_enabled = false;
bool left_driver_ready = false;
bool right_driver_ready = false;
bool wheel_sample_ready = false;
std::int64_t encoder_started_us = 0;
std::int64_t previous_encoder_us = 0;
std::int64_t previous_current_us = 0;
constexpr float two_pi = 6.283185307179586f;

bool align(BLDCMotor &motor, encoder::CheckedEncoder &sensor, BLDCDriver3PWM &driver, MotorState &state)
{
    motor.linkSensor(&sensor);
    motor.linkDriver(&driver);
    motor.controller = MotionControlType::torque;
    motor.torque_controller = TorqueControlType::voltage;
    motor.voltage_limit = config::kUqLimitV;
    motor.voltage_sensor_align = config::kSensorAlignmentVoltageV;
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    if (!motor.init()) { return false; }
    // 对齐可能转动，沿用库的有界扫描；此阶段没有运行软件电流保护。
    const bool success = motor.initFOC() && sensor.healthy() &&
        std::isfinite(motor.zero_electric_angle) &&
        (motor.sensor_direction == Direction::CW || motor.sensor_direction == Direction::CCW);
    motor.disable();
    if (!success) { return false; }
    state.encoder_direction = static_cast<float>(motor.sensor_direction);
    state.zero_electrical_angle_rad = motor.zero_electric_angle;
    sensor.setCached(true);
    return true;
}

float updateWheel(encoder::CheckedEncoder &sensor, MotorState &state, float forward_sign, float dt_s)
{
    sensor.update();
    const float angle_rad = sensor.getMechanicalAngle();
    // 单圈差分避免累计转数损失精度；每周期都更新，静止时原始速度确实为0。
    const float delta_rad = std::remainder(angle_rad - state.previous_angle_rad, two_pi);
    state.previous_angle_rad = angle_rad;
    const float velocity_rad_s = state.encoder_direction * forward_sign * delta_rad / dt_s;
    const float alpha = dt_s / (encoder::config::kWheelVelocityFilterS + dt_s);
    state.velocity_rad_s += alpha * (velocity_rad_s - state.velocity_rad_s);
    state.electrical_angle_rad = std::remainder(state.encoder_direction * config::kPolePairs *
        angle_rad - state.zero_electrical_angle_rad, two_pi);
    return state.velocity_rad_s;
}

MotorSample calculateCurrent(MotorState &state, const current_sensor::PhaseCurrents &phase,
    float requested_a, float forward_sign, float dt_s, PhaseDuty &duty)
{
    const float reference_a = std::clamp(requested_a, -config::kCurrentLimitA, config::kCurrentLimitA);
    const float iq_a = projectQCurrent(phase.a, phase.b, state.electrical_angle_rad);
    if (!state.current_filter_ready) { state.iq_filtered_a = iq_a; state.current_filter_ready = true; }
    const float alpha = dt_s / (config::kCurrentFilterS + dt_s);
    state.iq_filtered_a += alpha * (iq_a - state.iq_filtered_a);
    const float voltage_limit_v = std::min(config::kUqLimitV,
        config::kSvpwmLinearMargin * config::kPwmBusReferenceV / 1.7320508075688772f);
    const auto pi = updateCurrentPi(state.pi, forward_sign * reference_a - state.iq_filtered_a,
        dt_s, voltage_limit_v);
    // 暂停PI后Uq低通，直接输出限幅后的PI电压，遥测与实际输出保持一致。
    // const float output_alpha = dt_s / (config::kCurrentOutputFilterS + dt_s);
    // state.uq_filtered_v += output_alpha * (pi.applied_v - state.uq_filtered_v);
    state.uq_filtered_v = pi.applied_v;
    if (pi.valid) { duty = calculateSvpwmDuty(state.uq_filtered_v, state.electrical_angle_rad, config::kPwmBusReferenceV); }
    // 遥测Iq与Uq转换为车辆前进坐标；相电流仍为桥臂到电机坐标。
    return {reference_a, forward_sign * state.iq_filtered_a, forward_sign * state.uq_filtered_v,
        phase.a, phase.b, phase.c, pi.saturated, requested_a != reference_a};
}
void writePwm(BLDCDriver3PWM &driver, const PhaseDuty &duty)
{
    // setPwm接收V，不是占空比；三相驱动限幅保持母线参考，Uq已在PI处限制。
    driver.setPwm(duty.a * config::kPwmBusReferenceV, duty.b * config::kPwmBusReferenceV,
        duty.c * config::kPwmBusReferenceV);
}
} // namespace


esp_err_t initialize(ErrorInfo *error, BootReporter report)
{
    if (stopped) { return VEHICLE_ERROR(error, ESP_ERR_INVALID_STATE, motor_state, application, 0); }
    if (initialized) { return ESP_OK; }
    esp_err_t rc = pauseOutputs(error);
    if (rc != ESP_OK) { return rc; }
    if (!config::kCurrentHardwareVerified) { return VEHICLE_ERROR(error, ESP_ERR_INVALID_STATE, motor_gate, application, 0); }
    left_driver.voltage_power_supply = right_driver.voltage_power_supply = config::kPwmBusReferenceV;
    left_driver.voltage_limit = right_driver.voltage_limit = config::kPwmBusReferenceV;
    if (report) { report(static_cast<std::uint16_t>(ErrorPoint::left_driver),"BEGIN"); }
    left_driver_ready = left_driver.init(0) != 0;
    if (!left_driver_ready) { return VEHICLE_ERROR(error, ESP_FAIL, left_driver, simplefoc, 0); }
    left_driver.disable();
    if (report) { report(static_cast<std::uint16_t>(ErrorPoint::left_driver),"OK"); }

    if (report) { report(static_cast<std::uint16_t>(ErrorPoint::right_driver),"BEGIN"); }
    right_driver_ready = right_driver.init(1) != 0;
    if (!right_driver_ready) { return VEHICLE_ERROR(error, ESP_FAIL, right_driver, simplefoc, 0); }
    right_driver.disable();
    if (report) { report(static_cast<std::uint16_t>(ErrorPoint::right_driver),"OK"); }

    if (report) { report(static_cast<std::uint16_t>(ErrorPoint::current_unit),"BEGIN"); }
    rc = current_sensor::initialize(error);
    if (rc != ESP_OK) { return rc; }
    if (report) { report(static_cast<std::uint16_t>(ErrorPoint::current_unit),"OK"); }
    if (report) { report(static_cast<std::uint16_t>(ErrorPoint::left_encoder_init),"BEGIN"); }
    rc=left_sensor.initialize(error,ErrorPoint::left_encoder_init);
    if (rc != ESP_OK) { return rc; }
    if (report) { report(static_cast<std::uint16_t>(ErrorPoint::left_encoder_init),"OK"); }
    if (report) { report(static_cast<std::uint16_t>(ErrorPoint::right_encoder_init),"BEGIN"); }
    rc=right_sensor.initialize(error,ErrorPoint::right_encoder_init);
    if (rc != ESP_OK) { return rc; }

    if (report) { report(static_cast<std::uint16_t>(ErrorPoint::right_encoder_init),"OK"); }
    rc = gpio_set_level(board::pins::kMotorCommonEnable, 1);
    if (rc != ESP_OK) { return VEHICLE_ERROR(error, rc, enable_gpio, esp, rc); }
    if (report) { report(static_cast<std::uint16_t>(ErrorPoint::right_alignment),"BEGIN"); }
    if (!align(right_alignment, right_sensor, right_driver, right_state)) {
        if (!right_sensor.healthy()) { return VEHICLE_ERROR(error, right_sensor.rawError(), right_encoder_read, esp, right_sensor.rawError()); }
        return VEHICLE_ERROR(error, ESP_FAIL, right_alignment, simplefoc, 0);
    }
    if (report) { report(static_cast<std::uint16_t>(ErrorPoint::right_alignment),"OK"); }
    if (report) { report(static_cast<std::uint16_t>(ErrorPoint::left_alignment),"BEGIN"); }
    if (!align(left_alignment, left_sensor, left_driver, left_state)) {
        if (!left_sensor.healthy()) { return VEHICLE_ERROR(error, left_sensor.rawError(), left_encoder_read, esp, left_sensor.rawError()); }
        return VEHICLE_ERROR(error, ESP_FAIL, left_alignment, simplefoc, 0);
    }

    if (report) { report(static_cast<std::uint16_t>(ErrorPoint::left_alignment),"OK"); }
    rc = pauseOutputs(error);
    if (rc != ESP_OK) { return rc; }
    if (!left_sensor.refresh()) { return VEHICLE_ERROR(error, left_sensor.rawError(), left_encoder_read, esp, left_sensor.rawError()); }
    if (!right_sensor.refresh()) { return VEHICLE_ERROR(error, right_sensor.rawError(), right_encoder_read, esp, right_sensor.rawError()); }
    left_state.previous_angle_rad = left_sensor.angleRad();
    right_state.previous_angle_rad = right_sensor.angleRad();
    previous_encoder_us = 0;
    initialized = true;
    return ESP_OK;
}

esp_err_t readWheelState(WheelState *out, ErrorInfo *error)
{
    wheel_sample_ready = false;
    if (!out || !initialized || stopped) { return VEHICLE_ERROR(error, ESP_ERR_INVALID_STATE, motor_state, application, 0); }
    *out = {};
    const auto started_us = esp_timer_get_time();
    const bool first_sample = previous_encoder_us == 0;
    const float dt_s = first_sample ? config::kControlPeriodUs * 1.0e-6f :
        (started_us - previous_encoder_us) * 1.0e-6f;
    if (!(dt_s > 0.0f && dt_s <= config::kMaximumControlGapS)) {
        return VEHICLE_ERROR(error, ESP_ERR_INVALID_STATE, wheel_dt, application, 0, dt_s, config::kMaximumControlGapS, -1, 3);
    }
    if (!left_sensor.refresh()) { return VEHICLE_ERROR(error, left_sensor.rawError(), left_encoder_read, esp, left_sensor.rawError()); }
    if (!right_sensor.refresh()) { return VEHICLE_ERROR(error, right_sensor.rawError(), right_encoder_read, esp, right_sensor.rawError()); }
    auto next_left = left_state;
    auto next_right = right_state;
    if (first_sample) {
        next_left.previous_angle_rad = left_sensor.angleRad();
        next_right.previous_angle_rad = right_sensor.angleRad();
    }
    const float left_rad_s = updateWheel(left_sensor, next_left, config::kMotor0ForwardSign, dt_s);
    const float right_rad_s = updateWheel(right_sensor, next_right, config::kMotor1ForwardSign, dt_s);
    if (!std::isfinite(left_rad_s) || !std::isfinite(right_rad_s)) {
        return VEHICLE_ERROR(error, ESP_ERR_INVALID_RESPONSE, wheel_invalid, application, 0);
    }
    if (esp_timer_get_time() - started_us > encoder::config::kReadMaxDurationUs) {
        return VEHICLE_ERROR(error, ESP_ERR_TIMEOUT, wheel_age, application, 0, esp_timer_get_time()-started_us, encoder::config::kReadMaxDurationUs, -1, 3, 1);
    }
    left_state=next_left; right_state=next_right;
    encoder_started_us=started_us; previous_encoder_us=started_us;
    wheel_sample_ready = true;
    *out = {left_rad_s, right_rad_s, true};
    return ESP_OK;
}

esp_err_t runCurrentControl(const CurrentCommand &command, CurrentFeedback *out, ErrorInfo *error, CurrentTiming *timing)
{
    CurrentTiming local{};
    auto &trace = timing ? *timing : local;
    trace = {};
    if (!out || !initialized || stopped || !wheel_sample_ready) { return VEHICLE_ERROR(error, ESP_ERR_INVALID_STATE, motor_state, application, 0); }
    *out = {};
    wheel_sample_ready = false;
    if (!std::isfinite(command.left_target_a) || !std::isfinite(command.right_target_a)) {
        return VEHICLE_ERROR(error, ESP_ERR_INVALID_ARG, command_invalid, application, 0);
    }
    current_sensor::Sample sample{};
    trace.stage=CurrentStage::adc;
    const auto adc_started_us=esp_timer_get_time();
    const auto rc = current_sensor::read(&sample, error);
    const auto math_started_us=esp_timer_get_time();
    trace.adc_us=math_started_us-adc_started_us;
    if (rc != ESP_OK) { return rc; }
    trace.stage=CurrentStage::math;
    const float dt_s = previous_current_us == 0 ? config::kControlPeriodUs * 1.0e-6f :
        (sample.started_us - previous_current_us) * 1.0e-6f;
    if (!(dt_s > 0.0f && dt_s <= config::kMaximumControlGapS)) {
        return VEHICLE_ERROR(error, ESP_ERR_INVALID_STATE, current_dt, application, 0, dt_s, config::kMaximumControlGapS, -1, 3);
    }
    auto next_left=left_state; auto next_right=right_state;
    PhaseDuty left_duty{}, right_duty{};
    const auto left = calculateCurrent(next_left, sample.phases_a[0], command.left_target_a,
        config::kMotor0ForwardSign, dt_s, left_duty);
    const auto right = calculateCurrent(next_right, sample.phases_a[1], command.right_target_a,
        config::kMotor1ForwardSign, dt_s, right_duty);
    if (!left_duty.valid) { return VEHICLE_ERROR(error, ESP_ERR_INVALID_RESPONSE, left_pi_svpwm, application, 0); }
    if (!right_duty.valid) { return VEHICLE_ERROR(error, ESP_ERR_INVALID_RESPONSE, right_pi_svpwm, application, 0); }
    auto checked_us=esp_timer_get_time();
    trace.math_us=checked_us-math_started_us;
    trace.stage=CurrentStage::before_pwm;
    trace.encoder_age_us=checked_us-encoder_started_us;
    trace.current_age_us=checked_us-sample.started_us;
    if (trace.encoder_age_us > encoder::config::kOutputMaxAgeUs) {
        return VEHICLE_ERROR(error, ESP_ERR_TIMEOUT, output_age, application, 0,
            trace.encoder_age_us, encoder::config::kOutputMaxAgeUs, -1, 3, 1);
    }
    if (trace.current_age_us > config::kCurrentOutputMaxAgeUs) {
        return VEHICLE_ERROR(error, ESP_ERR_TIMEOUT, current_output_age, application, 0,
            trace.current_age_us, config::kCurrentOutputMaxAgeUs, -1, 3, 1);
    }
    trace.stage=CurrentStage::pwm;
    const auto pwm_started_us=esp_timer_get_time();
    writePwm(left_driver, left_duty); writePwm(right_driver, right_duty);
    checked_us=esp_timer_get_time();
    trace.pwm_us=checked_us-pwm_started_us;
    trace.stage=CurrentStage::after_pwm;
    trace.encoder_age_us=checked_us-encoder_started_us;
    trace.current_age_us=checked_us-sample.started_us;
    if (trace.encoder_age_us > encoder::config::kOutputMaxAgeUs) {
        return VEHICLE_ERROR(error, ESP_ERR_TIMEOUT, output_age, application, 0,
            trace.encoder_age_us, encoder::config::kOutputMaxAgeUs, -1, 3, 1);
    }
    if (trace.current_age_us > config::kCurrentOutputMaxAgeUs) {
        return VEHICLE_ERROR(error, ESP_ERR_TIMEOUT, current_output_age, application, 0,
            trace.current_age_us, config::kCurrentOutputMaxAgeUs, -1, 3, 1);
    }
    trace.stage=CurrentStage::enable;
    if (!outputs_enabled) {
        const auto enabled = gpio_set_level(board::pins::kMotorCommonEnable, 1);
        if (enabled != ESP_OK) { return VEHICLE_ERROR(error, enabled, enable_gpio, esp, enabled); }
        outputs_enabled=true;
    }
    left_state=next_left; right_state=next_right; previous_current_us=sample.started_us;
    *out = {left, right, dt_s, esp_timer_get_time()-sample.started_us, true};
    trace.stage=CurrentStage::complete;
    return ESP_OK;
}
esp_err_t inhibitOutputs(ErrorInfo *error)
{
    const auto level = gpio_set_level(board::pins::kMotorCommonEnable, 0);
    esp_err_t direction=ESP_OK;
    if (!enable_ready) {
        direction=gpio_set_direction(board::pins::kMotorCommonEnable, GPIO_MODE_OUTPUT);
        enable_ready=direction == ESP_OK;
    }
    if (left_driver_ready) { left_driver.disable(); }
    if (right_driver_ready) { right_driver.disable(); }
    outputs_enabled=false;
    if (level != ESP_OK) { return VEHICLE_ERROR(error, level, disable_gpio, esp, level); }
    if (direction != ESP_OK) { return VEHICLE_ERROR(error, direction, disable_gpio, esp, direction); }
    return ESP_OK;
}
esp_err_t pauseOutputs(ErrorInfo *error)
{
    const auto rc = inhibitOutputs(error);
    left_state.pi = right_state.pi = {};
    left_state.iq_filtered_a = right_state.iq_filtered_a = 0.0f;
    left_state.uq_filtered_v = right_state.uq_filtered_v = 0.0f;
    left_state.current_filter_ready = right_state.current_filter_ready = false;
    previous_current_us=0; wheel_sample_ready=false;
    return rc;
}
esp_err_t disableOutputs(ErrorInfo *error)
{
    stopped=true;
    return pauseOutputs(error);
}
} // namespace motor
} // namespace vehicle
