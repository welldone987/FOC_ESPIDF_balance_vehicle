#include "power_monitor.hpp"

#include "board_pins.hpp"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "vehicle_config.hpp"

namespace vehicle {
namespace power {
namespace {

constexpr char kTag[] = "power_monitor";
constexpr adc_atten_t kAttenuation = ADC_ATTEN_DB_12;

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

    esp_err_t result = adc_oneshot_io_to_channel(
        static_cast<int>(board::pins::kBatteryVoltageAdc),
        &adc_unit,
        &adc_channel);
    if (result != ESP_OK) {
        ESP_LOGE(kTag, "GPIO13 is not a valid ADC channel: %s", esp_err_to_name(result));
        return result;
    }

    if (adc_unit != ADC_UNIT_2) {
        ESP_LOGE(kTag, "VIN_MEA expected ADC2 but resolved to unit %d", static_cast<int>(adc_unit));
        return ESP_ERR_INVALID_STATE;
    }

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

    adc_cali_line_fitting_config_t calibration_config{};
    calibration_config.unit_id = adc_unit;
    calibration_config.atten = kAttenuation;
    calibration_config.bitwidth = ADC_BITWIDTH_DEFAULT;
    calibration_config.default_vref = config::kAdcDefaultVrefMv;

    result = adc_cali_create_scheme_line_fitting(
        &calibration_config,
        &calibration_handle);
    if (result != ESP_OK) {
        ESP_LOGE(kTag, "ADC line-fitting calibration unavailable: %s", esp_err_to_name(result));
        return result;
    }

    initialized = true;
    ESP_LOGI(kTag,
             "VIN ADC initialized on GPIO%d (ADC%d channel %d)",
             static_cast<int>(board::pins::kBatteryVoltageAdc),
             static_cast<int>(adc_unit) + 1,
             static_cast<int>(adc_channel));
    return ESP_OK;
}

esp_err_t readBusVoltage(float *voltage_v)
{
    if (!initialized || voltage_v == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

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
