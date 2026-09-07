#include "power_monitor.hpp"

#include "board_pins.hpp"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "vehicle_config.hpp"

namespace vehicle {
namespace power {
namespace {

/*
 * 电源模块把GPIO13的ADC2原始采样经过线性校准和分压比例换算为母线电压。
 * initialize()只建立一次ADC资源，readBusVoltage()在启动检查时提供当前读数。
 */

constexpr adc_atten_t kAttenuation = ADC_ATTEN_DB_12;

// adc_handle和calibration_handle保存一次性ADC与线性校准资源。
adc_oneshot_unit_handle_t adc_handle = nullptr;
adc_cali_handle_t calibration_handle = nullptr;
adc_unit_t adc_unit = ADC_UNIT_2;
adc_channel_t adc_channel = ADC_CHANNEL_4;
bool initialized = false;

} // namespace

esp_err_t initialize()
{
    if (initialized) {
        return ESP_OK;
    }

    // 从板级GPIO映射解析ADC单元和通道，避免重复维护GPIO13对应关系。
    esp_err_t result = adc_oneshot_io_to_channel(
        static_cast<int>(board::pins::kBatteryVoltageAdc),
        &adc_unit,
        &adc_channel);
    if (result != ESP_OK) {
        return result;
    }

    if (adc_unit != ADC_UNIT_2) {
        return ESP_ERR_INVALID_STATE;
    }

    // ADC2只在Wi-Fi启动前的低频电源检查路径中使用。
    adc_oneshot_unit_init_cfg_t unit_config{};
    unit_config.unit_id = adc_unit;
    unit_config.ulp_mode = ADC_ULP_MODE_DISABLE;

    result = adc_oneshot_new_unit(&unit_config, &adc_handle);
    if (result != ESP_OK) {
        return result;
    }

    adc_oneshot_chan_cfg_t channel_config{};
    channel_config.atten = kAttenuation;
    channel_config.bitwidth = ADC_BITWIDTH_DEFAULT;

    result = adc_oneshot_config_channel(adc_handle, adc_channel, &channel_config);
    if (result != ESP_OK) {
        return result;
    }

    // 线性校准把ADC原始码转换为分压节点电压，输出单位mV。
    adc_cali_line_fitting_config_t calibration_config{};
    calibration_config.unit_id = adc_unit;
    calibration_config.atten = kAttenuation;
    calibration_config.bitwidth = ADC_BITWIDTH_DEFAULT;
    calibration_config.default_vref = config::kAdcDefaultVrefMv;

    result = adc_cali_create_scheme_line_fitting(
        &calibration_config,
        &calibration_handle);
    if (result != ESP_OK) {
        return result;
    }

    initialized = true;
    return ESP_OK;
}

esp_err_t readBusVoltage(float *voltage_v)
{
    if (!initialized || voltage_v == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    // raw是ADC原始码，millivolts是校准后的VIN_MEA节点电压。
    int raw = 0;
    esp_err_t result = adc_oneshot_read(adc_handle, adc_channel, &raw);
    if (result != ESP_OK) {
        return result;
    }

    int millivolts = 0;
    result = adc_cali_raw_to_voltage(calibration_handle, raw, &millivolts);
    if (result != ESP_OK) {
        return result;
    }

    // kBatteryVoltageScale恢复分压前的母线电压，最终单位为V。
    *voltage_v =
        static_cast<float>(millivolts) * config::kBatteryVoltageScale / 1000.0f;
    return ESP_OK;
}

bool isInitialized()
{
    return initialized;
}

} // namespace power
} // namespace vehicle
