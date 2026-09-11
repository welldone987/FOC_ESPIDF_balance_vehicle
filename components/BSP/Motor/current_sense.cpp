#include "current_sense.hpp"
#include <algorithm>
#include <cmath>
#include "board_pins.hpp"
#include "vehicle_config.hpp"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace vehicle::motor::current_sense {
namespace {
constexpr std::array<gpio_num_t, 4> current_pins{
    board::pins::kMotor0CurrentSenseOut1, board::pins::kMotor0CurrentSenseOut2,
    board::pins::kMotor1CurrentSenseOut1, board::pins::kMotor1CurrentSenseOut2};
adc_oneshot_unit_handle_t adc = nullptr;
adc_cali_handle_t calibration = nullptr;
bool ready=false;
std::array<adc_channel_t, 4> channels{};
std::array<float, 4> offsets_mv{};

std::int64_t sample_started_us = 0;
esp_err_t readMillivolts(std::array<int, 4> &values, ErrorInfo *error)
{
    sample_started_us = esp_timer_get_time();
    for (unsigned i = 0; i < values.size(); ++i) {
        int raw = 0;
        esp_err_t result = adc_oneshot_read(adc, channels[i], &raw);
        if (result != ESP_OK) { return VEHICLE_ERROR(error, result, current_raw, esp, result, raw, 0, i, 5); }
        result = adc_cali_raw_to_voltage(calibration, raw, &values[i]);
        if (result != ESP_OK) { return VEHICLE_ERROR(error, result, current_mv, esp, result, raw, 0, i, 5); }
        if (values[i] < config::kCurrentAdcMinMv || values[i] > config::kCurrentAdcMaxMv) {
            return VEHICLE_ERROR(error, ESP_ERR_INVALID_RESPONSE, current_range, application, 0, values[i], values[i] < config::kCurrentAdcMinMv ? config::kCurrentAdcMinMv : config::kCurrentAdcMaxMv, i, 7);
        }
    }
    return esp_timer_get_time() - sample_started_us <= config::kCurrentReadMaxDurationUs
        ? ESP_OK : VEHICLE_ERROR(error, ESP_ERR_TIMEOUT, current_timeout, application, 0, esp_timer_get_time()-sample_started_us, config::kCurrentReadMaxDurationUs, -1, 3, 1);
}

} // namespace

void release()
{
    ready=false;
    if (calibration) { adc_cali_delete_scheme_line_fitting(calibration); calibration = nullptr; }
    if (adc) { adc_oneshot_del_unit(adc); adc = nullptr; }
}

esp_err_t initialize(ErrorInfo *error)
{
    ready=false;
    adc_oneshot_unit_init_cfg_t unit{};
    unit.unit_id = ADC_UNIT_1;
    esp_err_t result = adc_oneshot_new_unit(&unit, &adc);
    if (result != ESP_OK) { return VEHICLE_ERROR(error, result, current_unit, esp, result); }
    adc_oneshot_chan_cfg_t channel{};
    channel.atten = ADC_ATTEN_DB_12;
    channel.bitwidth = ADC_BITWIDTH_12;
    for (unsigned i = 0; i < channels.size(); ++i) {
        adc_unit_t actual_unit{};
        result = adc_oneshot_io_to_channel(current_pins[i], &actual_unit, &channels[i]);
        if (result != ESP_OK || actual_unit != ADC_UNIT_1) { return VEHICLE_ERROR(error, result != ESP_OK ? result : ESP_ERR_INVALID_ARG, current_map, esp, result, 0, 0, i, 4); }
        result = adc_oneshot_config_channel(adc, channels[i], &channel);
        if (result != ESP_OK) { return VEHICLE_ERROR(error, result, current_channel, esp, result, 0, 0, i, 4); }
    }
    adc_cali_line_fitting_config_t cal{};
    cal.unit_id = ADC_UNIT_1;
    cal.atten = ADC_ATTEN_DB_12;
    cal.bitwidth = ADC_BITWIDTH_12;
    cal.default_vref = config::kAdcDefaultVrefMv;
    result = adc_cali_create_scheme_line_fitting(&cal, &calibration);
    if (result != ESP_OK) { return VEHICLE_ERROR(error, result, current_calibration, esp, result); }
    std::array<int, 4> low{};
    std::array<int, 4> high{};
    low.fill(config::kCurrentAdcMaxMv);
    offsets_mv.fill(0.0f);
    for (unsigned sample = 0; sample < config::kCurrentOffsetSamples; ++sample) {
        std::array<int, 4> mv{};
        result = readMillivolts(mv, error);
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
            offsets_mv[i] > config::kCurrentOffsetMaxMv) {
            return VEHICLE_ERROR(error, ESP_ERR_INVALID_RESPONSE, offset_mean, application, 0, offsets_mv[i], offsets_mv[i] < config::kCurrentOffsetMinMv ? config::kCurrentOffsetMinMv : config::kCurrentOffsetMaxMv, i, 7);
        }
    }
    for (unsigned i=0; i<offsets_mv.size(); ++i) {
        if (high[i]-low[i] > config::kCurrentOffsetNoiseMv) { return VEHICLE_ERROR(error, ESP_ERR_INVALID_RESPONSE, offset_noise, application, 0, high[i]-low[i], config::kCurrentOffsetNoiseMv, i, 7, 1); }
    }
    ready=true;
    return ESP_OK;
}

esp_err_t read(Sample *out, ErrorInfo *error)
{
    std::array<int, 4> mv{};
    if (!out) { return VEHICLE_ERROR(error, ESP_ERR_INVALID_ARG, current_raw, application, 0); }
    *out = {};
    if (!ready) { return VEHICLE_ERROR(error,ESP_ERR_INVALID_STATE,current_state,application,0); }
    const esp_err_t rc = readMillivolts(mv, error);
    if (rc != ESP_OK) { return rc; }
    std::array<float, 4> amps{};
    std::array<PhaseCurrents,2> phase_samples{};
    for (unsigned i = 0; i < amps.size(); ++i) {
        amps[i] = (mv[i] - offsets_mv[i]) * 0.001f * config::kCurrentPolarity /
            (config::kCurrentShuntOhm * config::kCurrentAmplifierGain);
        if (!std::isfinite(amps[i]) || std::abs(amps[i]) > config::kPhaseTripA) { return VEHICLE_ERROR(error, ESP_ERR_INVALID_RESPONSE, phase_limit, application, 0, amps[i], config::kPhaseTripA, i, 7, 1); }
    }
    for (unsigned i = 0; i < phase_samples.size(); ++i) {
        const float a = amps[2 * i];
        const float b = amps[2 * i + 1];
        if (std::abs(a + b) > config::kPhaseTripA) { return VEHICLE_ERROR(error, ESP_ERR_INVALID_RESPONSE, reconstructed_limit, application, 0, -a-b, config::kPhaseTripA, i, 7, 1); }
        // 两相测量重构第三相，仅用于保护，不建立Id环。
        phase_samples[i] = {a, b, -a - b};
    }
    *out = {phase_samples, sample_started_us, true};
    return ESP_OK;
}

} // namespace vehicle::motor::current_sense
