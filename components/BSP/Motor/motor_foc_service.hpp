#pragma once

#include "esp_err.h"

namespace vehicle {
namespace motor {

struct WheelState {
    float left_velocity_rad_s;
    float right_velocity_rad_s;
    bool valid;
};

struct VoltageCommand {
    float left_target_v;
    float right_target_v;
};

// Initializes both AS5600 sensors, MCPWM-backed 3PWM drivers and SimpleFOC motors.
// This function performs FOC alignment and can energize the inverter; call it only
// from an explicitly authorized startup path with the vehicle safely supported.
esp_err_t initialize();

// Runs the same ordering as the Arduino reference: loopFOC() + move() consume the
// previously staged target, then the current wheel velocities are returned.
WheelState runFocAndReadWheelState();

// Stages voltage-mode torque targets for the next move() call.
void stageTarget(const VoltageCommand &command);

// Disables both motor outputs when initialization has completed.
void disableOutputs();

bool isInitialized();

} // namespace motor
} // namespace vehicle
