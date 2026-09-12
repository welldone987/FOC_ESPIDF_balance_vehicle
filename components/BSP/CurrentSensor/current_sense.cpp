#include "current_sense.hpp"
#include <algorithm>
#include <cmath>
#include "board_pins.hpp"
#include "current_sensor_config.hpp"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace vehicle {
namespace current_sensor {
namespace {
/*
 * 静态资源保存ADC1单元、线性校准句柄和四路零偏。
 * ReadMillivolts()按current_pins顺序读取并把校准后的mV写入调用者缓冲。
 */
// current_pins按M0 A/B、M1 A/B顺序排列四路相电流采样引脚。
constexpr std::array<gpio_num_t, 4> current_pins{
    pins::current_sensor_out_a_M0, pins::current_sensor_out_b_M0,
    pins::current_sensor_out_a_M1, pins::current_sensor_out_b_M1};
// adc保存ADC1一次性采样单元句柄。
adc_oneshot_unit_handle_t adc = nullptr;
// calibration保存line-fitting线性校准句柄。
adc_cali_handle_t calibration = nullptr;
// ready为true时Read()才允许访问ADC。
bool ready=false;
// channels保存四路GPIO解析出的ADC1通道号。
std::array<adc_channel_t, 4> channels{};
// offsets_mv保存零偏校准得到的每路静态电压，单位mV。
std::array<float, 4> offsets_mv{};

// sample_started_us记录本批读取的开始时刻，单位us。
std::int64_t sample_started_us = 0;
// ReadMillivolts()按current_pins顺序读取四路电压并检查范围。
esp_err_t ReadMillivolts(std::array<int, 4> &values, ErrorInfo *error)
{
    sample_started_us = esp_timer_get_time();
    for (unsigned i = 0; i < values.size(); ++i) {
        int raw = 0;
        esp_err_t result = adc_oneshot_read(adc, channels[i], &raw);
        if (result != ESP_OK) { return VEHICLE_ERROR(error, result, current_raw, esp, result, raw, 0, i, 5); }
        result = adc_cali_raw_to_voltage(calibration, raw, &values[i]);
        if (result != ESP_OK) { return VEHICLE_ERROR(error, result, current_mv, esp, result, raw, 0, i, 5); }
        if (values[i] < AdcMin_mV || values[i] > AdcMax_mV) {
            return VEHICLE_ERROR(error, ESP_ERR_INVALID_RESPONSE, current_range, application, 0, values[i], values[i] < AdcMin_mV ? AdcMin_mV : AdcMax_mV, i, 7);
        }
    }
    return esp_timer_get_time() - sample_started_us <= ReadMaxDuration_us
        ? ESP_OK : VEHICLE_ERROR(error, ESP_ERR_TIMEOUT, current_timeout, application, 0, esp_timer_get_time()-sample_started_us, ReadMaxDuration_us, -1, 3, 1);
}

} // namespace

void Release()
{
    ready=false;
    if (calibration) { adc_cali_delete_scheme_line_fitting(calibration); calibration = nullptr; }
    if (adc) { adc_oneshot_del_unit(adc); adc = nullptr; }
}

esp_err_t Initialize(ErrorInfo *error)
{
    ready=false;
    adc_oneshot_unit_init_cfg_t unit{};
    // 四路采样引脚全部属于ADC1。
    unit.unit_id = ADC_UNIT_1;
    esp_err_t result = adc_oneshot_new_unit(&unit, &adc);
    if (result != ESP_OK) { return VEHICLE_ERROR(error, result, current_unit, esp, result); }
    adc_oneshot_chan_cfg_t channel{};
    // 四路通道统一使用12dB衰减和12位分辨率。
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
    cal.default_vref = AdcDefaultVref_mV;
    result = adc_cali_create_scheme_line_fitting(&cal, &calibration);
    if (result != ESP_OK) { return VEHICLE_ERROR(error, result, current_calibration, esp, result); }
    // low和high记录校准期间逐路电压极值，用于检查峰峰噪声。
    std::array<int, 4> low{};
    std::array<int, 4> high{};
    low.fill(AdcMax_mV);
    offsets_mv.fill(0.0f);
    for (unsigned sample = 0; sample < OffsetSamples; ++sample) {
        std::array<int, 4> mv{};
        result = ReadMillivolts(mv, error);
        if (result != ESP_OK) { return result; }
        for (unsigned i = 0; i < mv.size(); ++i) {
            offsets_mv[i] += mv[i];
            low[i] = std::min(low[i], mv[i]);
            high[i] = std::max(high[i], mv[i]);
        }
        // 仅启动零偏校准使用延时，此时驱动器保持关闭。
        vTaskDelay(1);
    }
    for (unsigned i = 0; i < offsets_mv.size(); ++i) {
        // offsets_mv保存OffsetSamples批次的平均电压，单位mV。
        offsets_mv[i] /= OffsetSamples;
        if (offsets_mv[i] < OffsetMin_mV ||
            offsets_mv[i] > OffsetMax_mV) {
            return VEHICLE_ERROR(error, ESP_ERR_INVALID_RESPONSE, offset_mean, application, 0, offsets_mv[i], offsets_mv[i] < OffsetMin_mV ? OffsetMin_mV : OffsetMax_mV, i, 7);
        }
    }
    for (unsigned i=0; i<offsets_mv.size(); ++i) {
        if (high[i]-low[i] > OffsetNoise_mV) { return VEHICLE_ERROR(error, ESP_ERR_INVALID_RESPONSE, offset_noise, application, 0, high[i]-low[i], OffsetNoise_mV, i, 7, 1); }
    }
    ready=true;
    return ESP_OK;
}

esp_err_t Read(Sample *out, ErrorInfo *error)
{
    std::array<int, 4> mv{};
    if (!out) { return VEHICLE_ERROR(error, ESP_ERR_INVALID_ARG, current_raw, application, 0); }
    *out = {};
    if (!ready) { return VEHICLE_ERROR(error,ESP_ERR_INVALID_STATE,current_state,application,0); }
    const esp_err_t rc = ReadMillivolts(mv, error);
    if (rc != ESP_OK) { return rc; }
    // amps把四路mV减去零偏后换算为相电流，单位A。
    std::array<float, 4> amps{};
    std::array<PhaseCurrents,2> phase_samples{};
    for (unsigned i = 0; i < amps.size(); ++i) {
        amps[i] = (mv[i] - offsets_mv[i]) * 0.001f * Polarity /
            (Shunt_Ohm * AmplifierGain);
        if (!std::isfinite(amps[i]) || std::abs(amps[i]) > PhaseTrip_A) { return VEHICLE_ERROR(error, ESP_ERR_INVALID_RESPONSE, phase_limit, application, 0, amps[i], PhaseTrip_A, i, 7, 1); }
    }
    for (unsigned i = 0; i < phase_samples.size(); ++i) {
        const float a = amps[2 * i];
        const float b = amps[2 * i + 1];
        if (std::abs(a + b) > PhaseTrip_A) { return VEHICLE_ERROR(error, ESP_ERR_INVALID_RESPONSE, reconstructed_limit, application, 0, -a-b, PhaseTrip_A, i, 7, 1); }
        // 两相测量重构第三相，仅用于保护，不建立Id环。
        phase_samples[i] = {a, b, -a - b};
    }
    *out = {phase_samples, sample_started_us, true};
    return ESP_OK;
}

} // namespace current_sensor
} // namespace vehicle
