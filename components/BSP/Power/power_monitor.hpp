#pragma once

#include "esp_err.h"

namespace vehicle {
namespace power {

/*
 * 电源监视模块通过DengFOC V4的VIN_MEA分压网络读取直流母线电压。
 * ESP-IDF ADC校准值把GPIO13的ADC原始码转换为分压节点电压，模块再恢复母线电压。
 */
// initialize()配置GPIO13/ADC2_CH4的一次性ADC和线性校准。
// Configures the DengFOC V4 VIN_MEA input (GPIO13 / ADC2_CH4) using the
// ESP-IDF oneshot ADC driver and line-fitting calibration.
esp_err_t initialize();

// readBusVoltage()返回经过7.5k/1k分压比例恢复后的母线电压，单位V。
// Returns the reconstructed DC-bus voltage after the 7.5k/1k divider.
esp_err_t readBusVoltage(float *voltage_v);

// isInitialized()报告ADC和校准句柄是否已准备完成。
bool isInitialized();

} // namespace power
} // namespace vehicle
