#pragma once

#include "error_info.hpp"

namespace vehicle {
namespace power {

/*
 * 电源监视模块通过DengFOC V4的VIN_MEA分压网络读取直流母线电压。
 * ESP-IDF ADC校准值把GPIO13的ADC原始码转换为分压节点电压，模块再恢复母线电压。
 */
// Initialize()配置GPIO13/ADC2_CH4的一次性ADC和线性校准。
esp_err_t Initialize(ErrorInfo *error=nullptr);

// ReadBusVoltage()返回经过7.5k/1k分压比例恢复后的母线电压，单位V。
esp_err_t ReadBusVoltage(float *voltage_V, ErrorInfo *error=nullptr);

// CheckStartupVoltage()读取母线电压并执行启动欠压门限检查；超限时填充undervoltage错误。
esp_err_t CheckStartupVoltage(ErrorInfo *error=nullptr);


} // namespace power
} // namespace vehicle
