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
esp_err_t ReadMillivolts(std::array<int, 4> &values_mv, ErrorInfo *error)
{
    sample_started_us = esp_timer_get_time();
    for (unsigned i = 0; i < values_mv.size(); ++i) {
        int raw_counts = 0;
        esp_err_t result = adc_oneshot_read(adc, channels[i], &raw_counts);
        if (result != ESP_OK) { return VEHICLE_ERROR(error, result, current_raw,result, raw_counts, 0, i, ErrorValue | ErrorChannel); }
        result = adc_cali_raw_to_voltage(calibration, raw_counts, &values_mv[i]);
        if (result != ESP_OK) { return VEHICLE_ERROR(error, result, current_mv,result, raw_counts, 0, i, ErrorValue | ErrorChannel); }
        if (values_mv[i] < AdcMin_mV || values_mv[i] > AdcMax_mV) {
            return VEHICLE_ERROR(error, ESP_ERR_INVALID_RESPONSE, current_range,0, values_mv[i], values_mv[i] < AdcMin_mV ? AdcMin_mV : AdcMax_mV, i, ErrorValue | ErrorThreshold | ErrorChannel);
        }
    }
    return esp_timer_get_time() - sample_started_us <= ReadMaxDuration_us
        ? ESP_OK : VEHICLE_ERROR(error, ESP_ERR_TIMEOUT, current_timeout,0, esp_timer_get_time()-sample_started_us, ReadMaxDuration_us, -1, ErrorValue | ErrorThreshold);
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
    if (result != ESP_OK) { return VEHICLE_ERROR(error, result, current_unit,result); }
    adc_oneshot_chan_cfg_t channel{};
    // 四路通道统一使用12dB衰减和12位分辨率。
    channel.atten = ADC_ATTEN_DB_12;
    channel.bitwidth = ADC_BITWIDTH_12;
    for (unsigned i = 0; i < channels.size(); ++i) {
        adc_unit_t actual_unit{};
        result = adc_oneshot_io_to_channel(current_pins[i], &actual_unit, &channels[i]);
        if (result != ESP_OK || actual_unit != ADC_UNIT_1) { return VEHICLE_ERROR(error, result != ESP_OK ? result : ESP_ERR_INVALID_ARG, current_map,result, 0, 0, i, ErrorChannel); }
        result = adc_oneshot_config_channel(adc, channels[i], &channel);
        if (result != ESP_OK) { return VEHICLE_ERROR(error, result, current_channel,result, 0, 0, i, ErrorChannel); }
    }
    adc_cali_line_fitting_config_t cal{};
    cal.unit_id = ADC_UNIT_1;
    cal.atten = ADC_ATTEN_DB_12;
    cal.bitwidth = ADC_BITWIDTH_12;
    cal.default_vref = AdcDefaultVref_mV;
    result = adc_cali_create_scheme_line_fitting(&cal, &calibration);
    if (result != ESP_OK) { return VEHICLE_ERROR(error, result, current_calibration,result); }
    // min_mv和max_mv记录校准期间逐路电压极值，用于检查峰峰噪声。
    std::array<int, 4> min_mv{};
    std::array<int, 4> max_mv{};
    min_mv.fill(AdcMax_mV);
    offsets_mv.fill(0.0f);
    for (unsigned sample_index = 0; sample_index < OffsetSamples; ++sample_index) {
        std::array<int, 4> batch_mv{};
        result = ReadMillivolts(batch_mv, error);
        if (result != ESP_OK) { return result; }
        for (unsigned i = 0; i < batch_mv.size(); ++i) {
            offsets_mv[i] += batch_mv[i];
            min_mv[i] = std::min(min_mv[i], batch_mv[i]);
            max_mv[i] = std::max(max_mv[i], batch_mv[i]);
        }
        // 仅启动零偏校准使用延时，此时驱动器保持关闭。
        vTaskDelay(1);
    }
    for (unsigned i = 0; i < offsets_mv.size(); ++i) {
        // offsets_mv保存OffsetSamples批次的平均电压，单位mV。
        offsets_mv[i] /= OffsetSamples;
        if (offsets_mv[i] < OffsetMin_mV ||
            offsets_mv[i] > OffsetMax_mV) {
            return VEHICLE_ERROR(error, ESP_ERR_INVALID_RESPONSE, offset_mean,0, offsets_mv[i], offsets_mv[i] < OffsetMin_mV ? OffsetMin_mV : OffsetMax_mV, i, ErrorValue | ErrorThreshold | ErrorChannel);
        }
    }
    for (unsigned i=0; i<offsets_mv.size(); ++i) {
        if (max_mv[i]-min_mv[i] > OffsetNoise_mV) { return VEHICLE_ERROR(error, ESP_ERR_INVALID_RESPONSE, offset_noise,0, max_mv[i]-min_mv[i], OffsetNoise_mV, i, ErrorValue | ErrorThreshold | ErrorChannel); }
    }
    ready=true;
    return ESP_OK;
}

esp_err_t Read(Sample *out, ErrorInfo *error)
{
    std::array<int, 4> phase_mv{};
    if (!out) { return VEHICLE_ERROR(error, ESP_ERR_INVALID_ARG, current_raw,0); }
    *out = {};
    if (!ready) { return VEHICLE_ERROR(error,ESP_ERR_INVALID_STATE,current_state,0); }
    const esp_err_t rc = ReadMillivolts(phase_mv, error);
    if (rc != ESP_OK) { return rc; }
    // currents_A把四路mV减去零偏后换算为相电流，单位A。
    std::array<float, 4> currents_A{};
    std::array<PhaseCurrents,2> phase_currents{};
    for (unsigned i = 0; i < currents_A.size(); ++i) {
        currents_A[i] = (phase_mv[i] - offsets_mv[i]) * 0.001f * Polarity /
            (Shunt_Ohm * AmplifierGain);
        if (!std::isfinite(currents_A[i]) || std::abs(currents_A[i]) > PhaseTrip_A) { return VEHICLE_ERROR(error, ESP_ERR_INVALID_RESPONSE, phase_limit,0, currents_A[i], PhaseTrip_A, i, ErrorValue | ErrorThreshold | ErrorChannel); }
    }
    for (unsigned i = 0; i < phase_currents.size(); ++i) {
        const float measured_a_A = currents_A[2 * i];
        const float measured_b_A = currents_A[2 * i + 1];
        if (std::abs(measured_a_A + measured_b_A) > PhaseTrip_A) { return VEHICLE_ERROR(error, ESP_ERR_INVALID_RESPONSE, reconstructed_limit,0, -measured_a_A-measured_b_A, PhaseTrip_A, i, ErrorValue | ErrorThreshold | ErrorChannel); }
        // 两相测量重构第三相，仅用于保护，不建立Id环。
        phase_currents[i] = {measured_a_A, measured_b_A, -measured_a_A - measured_b_A};
    }
    *out = {phase_currents, sample_started_us, true};
    return ESP_OK;
}

} // namespace current_sensor
} // namespace vehicle
