#include "motor_foc_service.hpp"
#include <algorithm>
#include <cmath>
#include "board_pins.hpp"
#include "vehicle_config.hpp"
#include "current_sense.hpp"
#include "current_control.hpp"
#include "svpwm.hpp"
#include "esp_simplefoc.h"
#include "esp_timer.h"

namespace vehicle::motor {
namespace {
// 库对齐时检查每次I2C读数；运行时缓存供Sensor::update消费，不重复访问总线。
class CheckedEncoder final : public AS5600 {
public:
    using AS5600::AS5600;
    bool cached = false;
    bool healthy = true;
    float angle_rad = 0.0f;
    bool refresh()
    {
        const float value_rad = AS5600::getSensorAngle();
        healthy = healthy && std::isfinite(value_rad) && value_rad >= 0.0f && value_rad < 6.283186f;
        if (healthy) { angle_rad = value_rad; }
        return healthy;
    }
    float getSensorAngle() override
    {
        if (!cached) { refresh(); }
        return healthy ? angle_rad : -1.0f;
    }
};
CheckedEncoder left_sensor{I2C_NUM_0, board::pins::kI2c0Scl, board::pins::kI2c0Sda};
CheckedEncoder right_sensor{I2C_NUM_1, board::pins::kI2c1Scl, board::pins::kI2c1Sda};
// 不传enable引脚：GPIO12为双轮公共电源，不能由任一驱动独立切换。
BLDCDriver3PWM left_driver{board::pins::kMotor0PwmA, board::pins::kMotor0PwmB, board::pins::kMotor0PwmC};
BLDCDriver3PWM right_driver{board::pins::kMotor1PwmA, board::pins::kMotor1PwmB, board::pins::kMotor1PwmC};
// 库电机对象仅用于启动对齐；运行路径不调用move/loopFOC及库电流PI。
BLDCMotor left_alignment{config::kMotorPolePairs};
BLDCMotor right_alignment{config::kMotorPolePairs};
struct MotorState {
    float encoder_direction;
    float zero_electrical_angle_rad;
    float previous_angle_rad;
    float electrical_angle_rad;
    float velocity_rad_s;
    float iq_filtered_a;
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

bool align(BLDCMotor &motor, CheckedEncoder &sensor, BLDCDriver3PWM &driver, MotorState &state)
{
    motor.linkSensor(&sensor);
    motor.linkDriver(&driver);
    motor.controller = MotionControlType::torque;
    motor.torque_controller = TorqueControlType::voltage;
    motor.voltage_limit = config::kUqLimitV;
    motor.voltage_sensor_align = config::kMotorSensorAlignmentVoltageV;
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    if (!motor.init()) { return false; }
    // 对齐可能转动，沿用库的有界扫描；此阶段没有运行软件电流保护。
    const bool success = motor.initFOC() && sensor.healthy &&
        std::isfinite(motor.zero_electric_angle) &&
        (motor.sensor_direction == Direction::CW || motor.sensor_direction == Direction::CCW);
    motor.disable();
    if (!success) { return false; }
    state.encoder_direction = static_cast<float>(motor.sensor_direction);
    state.zero_electrical_angle_rad = motor.zero_electric_angle;
    sensor.cached = true;
    return true;
}

float updateWheel(CheckedEncoder &sensor, MotorState &state, float forward_sign, float dt_s)
{
    sensor.update();
    const float angle_rad = sensor.getMechanicalAngle();
    // 单圈差分避免累计转数损失精度；每周期都更新，静止时原始速度确实为0。
    const float delta_rad = std::remainder(angle_rad - state.previous_angle_rad, two_pi);
    state.previous_angle_rad = angle_rad;
    const float velocity_rad_s = state.encoder_direction * forward_sign * delta_rad / dt_s;
    const float alpha = dt_s / (config::kWheelVelocityFilterS + dt_s);
    state.velocity_rad_s += alpha * (velocity_rad_s - state.velocity_rad_s);
    state.electrical_angle_rad = std::remainder(state.encoder_direction * config::kMotorPolePairs *
        angle_rad - state.zero_electrical_angle_rad, two_pi);
    return state.velocity_rad_s;
}

MotorSample calculateCurrent(MotorState &state, const current_sense::PhaseCurrents &phase,
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
    if (pi.valid) { duty = calculateSvpwmDuty(pi.applied_v, state.electrical_angle_rad, config::kPwmBusReferenceV); }
    // 遥测Iq与Uq转换为车辆前进坐标；相电流仍为桥臂到电机坐标。
    return {reference_a, forward_sign * state.iq_filtered_a, forward_sign * pi.applied_v,
        phase.a, phase.b, phase.c, pi.saturated, requested_a != reference_a};
}
void writePwm(BLDCDriver3PWM &driver, const PhaseDuty &duty)
{
    // setPwm接收V，不是占空比；三相驱动限幅保持母线参考，Uq已在PI处限制。
    driver.setPwm(duty.a * config::kPwmBusReferenceV, duty.b * config::kPwmBusReferenceV,
        duty.c * config::kPwmBusReferenceV);
}
} // namespace

esp_err_t initialize()
{
    if (stopped) { return ESP_ERR_INVALID_STATE; }
    if (initialized) { return ESP_OK; }
    // 先拉低输出锁存，再配置GPIO；即使验证门未通过也建立确定的关闭状态。
    esp_err_t result = gpio_set_level(board::pins::kMotorCommonEnable, 0);
    if (result == ESP_OK) { result = gpio_set_direction(board::pins::kMotorCommonEnable, GPIO_MODE_OUTPUT); }
    if (result != ESP_OK) { return result; }
    enable_ready = true;
    if (!config::kCurrentHardwareVerified) { return ESP_ERR_INVALID_STATE; }
    left_driver.voltage_power_supply = right_driver.voltage_power_supply = config::kPwmBusReferenceV;
    left_driver.voltage_limit = right_driver.voltage_limit = config::kPwmBusReferenceV;
    left_driver_ready = left_driver.init(0) != 0;
    if (!left_driver_ready) { disableOutputs(); return ESP_FAIL; }
    left_driver.disable();
    right_driver_ready = right_driver.init(1) != 0;
    if (!right_driver_ready) { disableOutputs(); return ESP_FAIL; }
    right_driver.disable();
    result = current_sense::initialize();
    if (result != ESP_OK) { disableOutputs(); current_sense::release(); return result; }
    left_sensor.init();
    right_sensor.init();
    if (!left_sensor.healthy || !right_sensor.healthy) { disableOutputs(); return ESP_FAIL; }
    if (gpio_set_level(board::pins::kMotorCommonEnable, 1) != ESP_OK ||
        !align(right_alignment, right_sensor, right_driver, right_state) ||
        !align(left_alignment, left_sensor, left_driver, left_state)) {
        disableOutputs(); return ESP_FAIL;
    }
    pauseOutputs();
    // 对齐后重新读取两轮，避免把对齐运动算进第一个速度样本。
    if (!left_sensor.refresh() || !right_sensor.refresh()) { disableOutputs(); return ESP_FAIL; }
    left_state.previous_angle_rad = left_sensor.angle_rad;
    right_state.previous_angle_rad = right_sensor.angle_rad;
    previous_encoder_us = 0;
    initialized = true;
    return ESP_OK;
}

WheelState readWheelState()
{
    wheel_sample_ready = false;
    if (!initialized || stopped) { return {}; }
    encoder_started_us = esp_timer_get_time();
    const bool first_sample = previous_encoder_us == 0;
    const float dt_s = first_sample ? config::kControlPeriodUs * 1.0e-6f :
        (encoder_started_us - previous_encoder_us) * 1.0e-6f;
    previous_encoder_us = encoder_started_us;
    if (!(dt_s > 0.0f && dt_s <= config::kMaximumControlGapS) ||
        !left_sensor.refresh() || !right_sensor.refresh()) { disableOutputs(); return {}; }
    if (first_sample) {
        left_state.previous_angle_rad = left_sensor.angle_rad;
        right_state.previous_angle_rad = right_sensor.angle_rad;
    }
    const float left_rad_s = updateWheel(left_sensor, left_state, config::kMotor0ForwardSign, dt_s);
    const float right_rad_s = updateWheel(right_sensor, right_state, config::kMotor1ForwardSign, dt_s);
    if (!std::isfinite(left_rad_s) || !std::isfinite(right_rad_s) ||
        esp_timer_get_time() - encoder_started_us > config::kCurrentSampleMaxAgeUs) {
        disableOutputs(); return {};
    }
    wheel_sample_ready = true;
    return {left_rad_s, right_rad_s, true};
}

CurrentFeedback runCurrentControl(const CurrentCommand &command)
{
    if (!initialized || stopped || !wheel_sample_ready) { disableOutputs(); return {}; }
    wheel_sample_ready = false;
    if (!std::isfinite(command.left_target_a) || !std::isfinite(command.right_target_a)) {
        disableOutputs(); return {};
    }
    const auto sample = current_sense::read();
    const float dt_s = previous_current_us == 0 ? config::kControlPeriodUs * 1.0e-6f :
        (sample.started_us - previous_current_us) * 1.0e-6f;
    previous_current_us = sample.started_us;
    if (!sample.valid || !(dt_s > 0.0f && dt_s <= config::kMaximumControlGapS)) { disableOutputs(); return {}; }
    PhaseDuty left_duty{}, right_duty{};
    const auto left = calculateCurrent(left_state, sample.phases_a[0], command.left_target_a,
        config::kMotor0ForwardSign, dt_s, left_duty);
    const auto right = calculateCurrent(right_state, sample.phases_a[1], command.right_target_a,
        config::kMotor1ForwardSign, dt_s, right_duty);
    if (!left_duty.valid || !right_duty.valid ||
        esp_timer_get_time() - encoder_started_us > config::kCurrentSampleMaxAgeUs) {
        disableOutputs(); return {};
    }
    writePwm(left_driver, left_duty);
    writePwm(right_driver, right_duty);
    if (esp_timer_get_time() - encoder_started_us > config::kCurrentSampleMaxAgeUs) { disableOutputs(); return {}; }
    if (!outputs_enabled) {
        if (gpio_set_level(board::pins::kMotorCommonEnable, 1) != ESP_OK) { disableOutputs(); return {}; }
        outputs_enabled = true;
    }
    return {left, right, dt_s, esp_timer_get_time() - sample.started_us, true};
}
void pauseOutputs()
{
    // IMU初始化失败时也可能先走停机路径，因此不能依赖电机初始化已完成。
    gpio_set_level(board::pins::kMotorCommonEnable, 0);
    if (!enable_ready) {
        enable_ready = gpio_set_direction(board::pins::kMotorCommonEnable, GPIO_MODE_OUTPUT) == ESP_OK;
    }
    if (left_driver_ready) { left_driver.disable(); }
    if (right_driver_ready) { right_driver.disable(); }
    outputs_enabled = false;
    left_state.pi = right_state.pi = {};
    left_state.iq_filtered_a = right_state.iq_filtered_a = 0.0f;
    left_state.current_filter_ready = right_state.current_filter_ready = false;
    previous_current_us = 0;
    wheel_sample_ready = false;
}
void disableOutputs()
{
    stopped = true;
    pauseOutputs();
}
} // namespace vehicle::motor
