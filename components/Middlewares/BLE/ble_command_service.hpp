#pragma once

#include <cstdint>

#include "esp_err.h"

namespace vehicle {
namespace ble {

struct CommandSnapshot {
    float steering_voltage_v;
    float throttle_velocity_rad_s;
    std::uint32_t sequence;
    bool connected;
};

// Starts the native ESP-IDF NimBLE peripheral and publishes the same
// read/write command characteristic used by the Arduino reference.
esp_err_t initialize();

// Lock-free snapshot for the single-threaded control loop. Disconnecting does
// not clear the last command, matching the reference main.cpp behavior.
CommandSnapshot latestCommand();

bool isInitialized();

} // namespace ble
} // namespace vehicle
