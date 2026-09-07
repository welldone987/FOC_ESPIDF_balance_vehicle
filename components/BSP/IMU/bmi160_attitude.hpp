#pragma once

#include "esp_err.h"

namespace vehicle {
namespace imu {

struct AttitudeSample {
    float pitch_deg;
    float pitch_rate_deg_s;
    bool valid;
};

// Creates/reuses I2C0, initializes BMI160 at 0x69, performs the same gyro
// hardware offset calibration used by BMI160Gen, and configures ±2g / ±1000 dps.
esp_err_t initialize();

// Matches the reference preInterval assignment performed after motor FOC init.
void resetEstimator();

// Reads gyro+accelerometer in one burst and applies the reference 0.98/0.02
// complementary filter. Pitch is returned in degrees.
AttitudeSample readAttitude();

bool isInitialized();

} // namespace imu
} // namespace vehicle
