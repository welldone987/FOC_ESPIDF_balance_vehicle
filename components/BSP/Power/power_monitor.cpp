#include "power_monitor.hpp"

#include "board_pins.hpp"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "power_config.hpp"

namespace vehicle {
namespace power {
namespace {

/*
 * 电源模块把GPIO13的ADC2原始采样经过线性校准和分压比例换算为母线电压。
 * Initialize()只建立一次ADC资源，ReadBusVoltage()在启动检查时提供当前读数。
 */

// Attenuation是VIN_MEA采样通道使用的ADC衰减档。
constexpr adc_atten_t Attenuation = ADC_ATTEN_DB_12;

// adc_handle保存ADC2一次性采样单元句柄。
adc_oneshot_unit_handle_t adc_handle = nullptr;
// calibration_handle保存line-fitting线性校准句柄。
adc_cali_handle_t calibration_handle = nullptr;
// adc_unit和adc_channel由bus_voltage_adc在Initialize()中解析得到。
adc_unit_t adc_unit = ADC_UNIT_2;
adc_channel_t adc_channel = ADC_CHANNEL_4;
// initialized为true后ReadBusVoltage()才允许访问ADC。
bool initialized = false;

} // namespace

esp_err_t Initialize(ErrorInfo *error)
{
    if (initialized) {
        return ESP_OK;
    }

    // 从板级GPIO映射解析ADC单元和通道，避免重复维护GPIO13对应关系。
    esp_err_t result = adc_oneshot_io_to_channel(
        static_cast<int>(pins::bus_voltage_adc),
        &adc_unit,
        &adc_channel);
    if (result != ESP_OK) {
        return VEHICLE_ERROR(error, result, power_map, esp, result);
    }

    if (adc_unit != ADC_UNIT_2) {
        return VEHICLE_ERROR(error, ESP_ERR_INVALID_STATE, power_map, application, 0);
    }

    // ADC2只在Wi-Fi启动前的低频电源检查路径中使用。
    adc_oneshot_unit_init_cfg_t unit_config{};
    unit_config.unit_id = adc_unit;
    unit_config.ulp_mode = ADC_ULP_MODE_DISABLE;

    result = adc_oneshot_new_unit(&unit_config, &adc_handle);
    if (result != ESP_OK) {
        return VEHICLE_ERROR(error, result, power_unit, esp, result);
    }

    adc_oneshot_chan_cfg_t channel_config{};
    channel_config.atten = Attenuation;
    channel_config.bitwidth = ADC_BITWIDTH_DEFAULT;

    result = adc_oneshot_config_channel(adc_handle, adc_channel, &channel_config);
    if (result != ESP_OK) {
        return VEHICLE_ERROR(error, result, power_channel, esp, result);
    }

    // 线性校准把ADC原始码转换为分压节点电压，输出单位mV。
    adc_cali_line_fitting_config_t calibration_config{};
    calibration_config.unit_id = adc_unit;
    calibration_config.atten = Attenuation;
    calibration_config.bitwidth = ADC_BITWIDTH_DEFAULT;
    calibration_config.default_vref = AdcDefaultVref_mV;

    result = adc_cali_create_scheme_line_fitting(
        &calibration_config,
        &calibration_handle);
    if (result != ESP_OK) {
        return VEHICLE_ERROR(error, result, power_calibration, esp, result);
    }

    initialized = true;
    return ESP_OK;
}

esp_err_t ReadBusVoltage(float *voltage_v, ErrorInfo *error)
{
    if (!initialized || voltage_v == nullptr) {
        return VEHICLE_ERROR(error, ESP_ERR_INVALID_STATE, power_map, application, 0);
    }

    // raw是ADC原始码，millivolts是校准后的VIN_MEA节点电压。
    int raw = 0;
    esp_err_t result = adc_oneshot_read(adc_handle, adc_channel, &raw);
    if (result != ESP_OK) {
        return VEHICLE_ERROR(error, result, power_raw, esp, result);
    }

    int millivolts = 0;
    result = adc_cali_raw_to_voltage(calibration_handle, raw, &millivolts);
    if (result != ESP_OK) {
        return VEHICLE_ERROR(error, result, power_mv, esp, result);
    }

    // BatteryVoltageScale恢复分压前的母线电压，最终单位为V。
    *voltage_v =
        static_cast<float>(millivolts) * BatteryVoltageScale / 1000.0f;
    return ESP_OK;
}

} // namespace power
} // namespace vehicle
