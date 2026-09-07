#pragma once

#include "esp_err.h"

namespace vehicle {
namespace power {

// Configures the DengFOC V4 VIN_MEA input (GPIO13 / ADC2_CH4) using the
// ESP-IDF oneshot ADC driver and line-fitting calibration.
esp_err_t initialize();

// Returns the reconstructed DC-bus voltage after the 7.5k/1k divider.
esp_err_t readBusVoltage(float *voltage_v);

bool isInitialized();

} // namespace power
} // namespace vehicle
