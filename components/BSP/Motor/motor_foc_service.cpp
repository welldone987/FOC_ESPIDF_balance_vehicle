#include "motor_foc_service.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include "board_pins.hpp"
#include "vehicle_config.hpp"
#include "esp_simplefoc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace vehicle::motor {
namespace {
// AS5600原接口用-1表示读失败；运行阶段先读取并验证，再供loopFOC消费缓存。
class CheckedEncoder final : public AS5600 {
public:
    using AS5600::AS5600;
    bool cached = false;
    bool healthy = true;
    float angle = 0.0f;
    bool refresh()
    {
        const float value = AS5600::getSensorAngle();
        healthy = healthy && std::isfinite(value) && value >= 0.0f;
        if (healthy) { angle = value; }
        return healthy;
    }
    float getSensorAngle() override
    {
        if (!cached) { refresh(); }
        return angle;
    }
};
CheckedEncoder left_sensor{I2C_NUM_0, board::pins::kI2c0Scl, board::pins::kI2c0Sda};
CheckedEncoder right_sensor{I2C_NUM_1, board::pins::kI2c1Scl, board::pins::kI2c1Sda};
BLDCMotor left_motor{config::kMotorPolePairs};
BLDCMotor right_motor{config::kMotorPolePairs};
BLDCDriver3PWM left_driver{board::pins::kMotor0PwmA, board::pins::kMotor0PwmB,
    board::pins::kMotor0PwmC, board::pins::kMotor0Enable};
BLDCDriver3PWM right_driver{board::pins::kMotor1PwmA, board::pins::kMotor1PwmB,
    board::pins::kMotor1PwmC, board::pins::kMotor1Enable};
constexpr std::array<gpio_num_t, 4> current_pins{
    board::pins::kMotor0CurrentSenseOut1, board::pins::kMotor0CurrentSenseOut2,
    board::pins::kMotor1CurrentSenseOut1, board::pins::kMotor1CurrentSenseOut2};
adc_oneshot_unit_handle_t adc = nullptr;
adc_cali_handle_t calibration = nullptr;
std::array<adc_channel_t, 4> channels{};
std::array<float, 4> offsets_mv{};
std::array<PhaseCurrent_s, 2> phase_samples{};
std::int64_t sample_started_us = 0;
bool initialized = false;
bool stopped = false;
bool left_driver_ready = false;
bool right_driver_ready = false;

// 适配库所需的唯一继承接口；ADC读取在PI之前完成，库只读取本周期有效快照。
class SampledCurrentSense final : public CurrentSense {
public:
    explicit SampledCurrentSense(unsigned index) : index_(index) {}
    int init() override { initialized = true; return 1; }
    PhaseCurrent_s getPhaseCurrents() override { return phase_samples[index_]; }
    int driverAlign(float, bool) override
    {
        // 不用缓存值伪装自动相序校准；启用门要求已核验OUT1=A、OUT2=B及极性。
        return config::kCurrentHardwareVerified ? 1 : 0;
    }
private:
    unsigned index_;
};
SampledCurrentSense left_current{0};
SampledCurrentSense right_current{1};

esp_err_t readMillivolts(std::array<int, 4> &values)
{
    sample_started_us = esp_timer_get_time();
    for (unsigned i = 0; i < values.size(); ++i) {
        int raw = 0;
        esp_err_t result = adc_oneshot_read(adc, channels[i], &raw);
        if (result != ESP_OK) { return result; }
        result = adc_cali_raw_to_voltage(calibration, raw, &values[i]);
        if (result != ESP_OK) { return result; }
        if (values[i] < config::kCurrentAdcMinMv || values[i] > config::kCurrentAdcMaxMv) {
            return ESP_ERR_INVALID_RESPONSE;
        }
    }
    return esp_timer_get_time() - sample_started_us <= config::kCurrentSampleMaxAgeUs
        ? ESP_OK : ESP_ERR_TIMEOUT;
}

void releaseAdc()
{
    if (calibration) { adc_cali_delete_scheme_line_fitting(calibration); calibration = nullptr; }
    if (adc) { adc_oneshot_del_unit(adc); adc = nullptr; }
}

esp_err_t initializeAdc()
{
    adc_oneshot_unit_init_cfg_t unit{};
    unit.unit_id = ADC_UNIT_1;
    esp_err_t result = adc_oneshot_new_unit(&unit, &adc);
    if (result != ESP_OK) { return result; }
    adc_oneshot_chan_cfg_t channel{};
    channel.atten = ADC_ATTEN_DB_12;
    channel.bitwidth = ADC_BITWIDTH_12;
    for (unsigned i = 0; i < channels.size(); ++i) {
        adc_unit_t actual_unit{};
        result = adc_oneshot_io_to_channel(current_pins[i], &actual_unit, &channels[i]);
        if (result != ESP_OK || actual_unit != ADC_UNIT_1) { return ESP_ERR_INVALID_ARG; }
        result = adc_oneshot_config_channel(adc, channels[i], &channel);
        if (result != ESP_OK) { return result; }
    }
    adc_cali_line_fitting_config_t cal{};
    cal.unit_id = ADC_UNIT_1;
    cal.atten = ADC_ATTEN_DB_12;
    cal.bitwidth = ADC_BITWIDTH_12;
    cal.default_vref = config::kAdcDefaultVrefMv;
    result = adc_cali_create_scheme_line_fitting(&cal, &calibration);
    if (result != ESP_OK) { return result; }
    std::array<int, 4> low{};
    std::array<int, 4> high{};
    low.fill(config::kCurrentAdcMaxMv);
    offsets_mv.fill(0.0f);
    for (unsigned sample = 0; sample < config::kCurrentOffsetSamples; ++sample) {
        std::array<int, 4> mv{};
        result = readMillivolts(mv);
        if (result != ESP_OK) { return result; }
        for (unsigned i = 0; i < mv.size(); ++i) {
            offsets_mv[i] += mv[i];
            low[i] = std::min(low[i], mv[i]);
            high[i] = std::max(high[i], mv[i]);
        }
        vTaskDelay(1); // 仅启动零偏校准使用，驱动器保持关闭。
    }
    for (unsigned i = 0; i < offsets_mv.size(); ++i) {
        offsets_mv[i] /= config::kCurrentOffsetSamples;
        if (offsets_mv[i] < config::kCurrentOffsetMinMv ||
            offsets_mv[i] > config::kCurrentOffsetMaxMv ||
            high[i] - low[i] > config::kCurrentOffsetNoiseMv) {
            return ESP_ERR_INVALID_RESPONSE;
        }
    }
    return ESP_OK;
}

bool sampleCurrents()
{
    std::array<int, 4> mv{};
    if (readMillivolts(mv) != ESP_OK) { return false; }
    std::array<float, 4> amps{};
    for (unsigned i = 0; i < amps.size(); ++i) {
        amps[i] = (mv[i] - offsets_mv[i]) * 0.001f * config::kCurrentPolarity /
            (config::kCurrentShuntOhm * config::kCurrentAmplifierGain);
        if (std::abs(amps[i]) > config::kPhaseTripA) { return false; }
    }
    for (unsigned i = 0; i < phase_samples.size(); ++i) {
        const float a = amps[2 * i];
        const float b = amps[2 * i + 1];
        if (std::abs(a + b) > config::kPhaseTripA) { return false; }
        // c=0是SimpleFOC的双采样标记，库按ic=-ia-ib重构第三相。
        phase_samples[i] = {a, b, 0.0f};
    }
    return true;
}

void configureMotor(BLDCMotor &motor, SampledCurrentSense &sense)
{
    motor.controller = MotionControlType::torque;
    motor.torque_controller = TorqueControlType::foc_current;
    motor.current_limit = config::kCurrentLimitA;
    motor.voltage_limit = config::kCurrentAxisVoltageV;
    motor.voltage_sensor_align = config::kMotorSensorAlignmentVoltageV;
    motor.linkCurrentSense(&sense);
    for (PIDController *pi : {&motor.PID_current_d, &motor.PID_current_q}) {
        pi->P = config::kCurrentKp;
        pi->I = config::kCurrentKi;
        pi->D = 0.0f;
        pi->output_ramp = 0.0f;
        pi->limit = config::kCurrentAxisVoltageV;
    }
    motor.LPF_current_d.Tf = config::kCurrentFilterS;
    motor.LPF_current_q.Tf = config::kCurrentFilterS;
    // 不设置R/L/KV或库前馈，避免限幅后的附加电压/电流绕过限制。
}
} // namespace

esp_err_t initialize()
{
    if (stopped) { return ESP_ERR_INVALID_STATE; }
    if (initialized) { return ESP_OK; }
    if (!config::kCurrentHardwareVerified) { return ESP_ERR_INVALID_STATE; }
    left_driver.voltage_power_supply = config::kMotorSupplyVoltageV;
    right_driver.voltage_power_supply = config::kMotorSupplyVoltageV;
    left_driver_ready = left_driver.init(0) != 0;
    if (!left_driver_ready) { disableOutputs(); return ESP_FAIL; }
    left_driver.disable();
    right_driver_ready = right_driver.init(1) != 0;
    if (!right_driver_ready) { disableOutputs(); return ESP_FAIL; }
    right_driver.disable();
    const esp_err_t result = initializeAdc();
    if (result != ESP_OK) { disableOutputs(); releaseAdc(); return result; }
    left_sensor.init();
    right_sensor.init();
    left_motor.linkSensor(&left_sensor);
    right_motor.linkSensor(&right_sensor);
    left_motor.linkDriver(&left_driver);
    right_motor.linkDriver(&right_driver);
    left_current.linkDriver(&left_driver);
    right_current.linkDriver(&right_driver);
    left_current.init();
    right_current.init();
    configureMotor(left_motor, left_current);
    configureMotor(right_motor, right_current);
    if (!right_motor.init() || !left_motor.init() ||
        !right_motor.initFOC() || !left_motor.initFOC() ||
        !left_sensor.healthy || !right_sensor.healthy) {
        disableOutputs();
        return ESP_FAIL;
    }
    left_sensor.cached = right_sensor.cached = true;
    initialized = true;
    return ESP_OK;
}

WheelState runFocAndReadWheelState()
{
    if (!initialized || stopped) { return {0, 0, false}; }
    const auto encoder_started_us = esp_timer_get_time();
    if (!left_sensor.refresh() || !right_sensor.refresh()) {
        disableOutputs(); return {0, 0, false};
    }
    if (!sampleCurrents()) { disableOutputs(); return {0, 0, false}; }
    // 显式消费上一姿态周期的目标，再运行电流PI；避免move放在PI之后多延迟一拍。
    left_motor.current_sp = left_motor.target;
    right_motor.current_sp = right_motor.target;
    if (esp_timer_get_time() - encoder_started_us > config::kCurrentSampleMaxAgeUs) {
        disableOutputs(); return {0, 0, false};
    }
    left_motor.loopFOC();
    if (esp_timer_get_time() - encoder_started_us > config::kCurrentSampleMaxAgeUs) {
        disableOutputs(); return {0, 0, false};
    }
    right_motor.loopFOC();
    left_motor.move();
    right_motor.move();
    const bool valid = std::isfinite(left_motor.shaft_velocity) &&
        std::isfinite(right_motor.shaft_velocity) &&
        esp_timer_get_time() - encoder_started_us <= config::kCurrentSampleMaxAgeUs;
    if (!valid) { disableOutputs(); }
    return {left_motor.shaft_velocity, right_motor.shaft_velocity, valid};
}
void stageTarget(const CurrentCommand &command)
{
    if (!initialized || stopped) { return; }
    if (!std::isfinite(command.left_target_a) || !std::isfinite(command.right_target_a)) {
        disableOutputs(); return;
    }
    left_motor.target = std::clamp(command.left_target_a, -config::kCurrentLimitA, config::kCurrentLimitA);
    right_motor.target = std::clamp(command.right_target_a, -config::kCurrentLimitA, config::kCurrentLimitA);
}
void disableOutputs()
{
    stopped = true;
    left_motor.target = right_motor.target = 0.0f;
    left_motor.current_sp = right_motor.current_sp = 0.0f;
    left_motor.enabled = 0;
    if (left_driver_ready) { left_driver.disable(); }
    right_motor.enabled = 0;
    if (right_driver_ready) { right_driver.disable(); }
}
} // namespace vehicle::motor
