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
std::array<adc_channel_t, 4> channels{};
std::array<float, 4> offsets_mv{};
std::array<PhaseCurrents, 2> phase_samples{};
std::int64_t sample_started_us = 0;
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

} // namespace

void release()
{
    if (calibration) { adc_cali_delete_scheme_line_fitting(calibration); calibration = nullptr; }
    if (adc) { adc_oneshot_del_unit(adc); adc = nullptr; }
}

esp_err_t initialize()
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

Sample read()
{
    std::array<int, 4> mv{};
    if (readMillivolts(mv) != ESP_OK) { return {}; }
    std::array<float, 4> amps{};
    for (unsigned i = 0; i < amps.size(); ++i) {
        amps[i] = (mv[i] - offsets_mv[i]) * 0.001f * config::kCurrentPolarity /
            (config::kCurrentShuntOhm * config::kCurrentAmplifierGain);
        if (std::abs(amps[i]) > config::kPhaseTripA) { return {}; }
    }
    for (unsigned i = 0; i < phase_samples.size(); ++i) {
        const float a = amps[2 * i];
        const float b = amps[2 * i + 1];
        if (std::abs(a + b) > config::kPhaseTripA) { return {}; }
        // 两相测量重构第三相，仅用于保护，不建立Id环。
        phase_samples[i] = {a, b, -a - b};
    }
    return {phase_samples, sample_started_us, true};
}

} // namespace vehicle::motor::current_sense
