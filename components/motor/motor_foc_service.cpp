#include "motor_foc_service.hpp"

#include "board_pins.hpp"
#include "esp_log.h"
#include "esp_simplefoc.h"

namespace vehicle {
namespace motor {
namespace {

constexpr char kTag[] = "motor_foc";

// Values are intentionally kept identical to the minimal Arduino balancing-car reference.
constexpr int kMotorPolePairs = 7;
constexpr float kSupplyVoltageV = 12.0f;
constexpr float kSensorAlignmentVoltageV = 2.0f;
constexpr float kVelocityPidP = 0.01f;
constexpr float kVelocityPidI = 0.10f;
constexpr float kVelocityPidD = 0.0f;

// Classic ESP32 exposes two MCPWM groups; keep one complete group per motor.
constexpr int kMotor0McpwmGroup = 0;
constexpr int kMotor1McpwmGroup = 1;

// Espressif AS5600 constructor order is (I2C port, SCL, SDA).
AS5600 left_sensor{I2C_NUM_0, board::pins::kI2c0Scl, board::pins::kI2c0Sda};
AS5600 right_sensor{I2C_NUM_1, board::pins::kI2c1Scl, board::pins::kI2c1Sda};

BLDCMotor left_motor{kMotorPolePairs};
BLDCMotor right_motor{kMotorPolePairs};

BLDCDriver3PWM left_driver{
    board::pins::kMotor0PwmA,
    board::pins::kMotor0PwmB,
    board::pins::kMotor0PwmC,
    board::pins::kMotor0Enable,
};

BLDCDriver3PWM right_driver{
    board::pins::kMotor1PwmA,
    board::pins::kMotor1PwmB,
    board::pins::kMotor1PwmC,
    board::pins::kMotor1Enable,
};

bool initialized = false;

void disableDrivers()
{
    left_driver.disable();
    right_driver.disable();
}

} // namespace

esp_err_t initialize()
{
    if (initialized) {
        return ESP_OK;
    }

    ESP_LOGI(kTag, "Initializing AS5600 sensors and esp_simplefoc 1.4.1 motor layer");

    // AS5600 M0 uses I2C0; AS5600 M1 uses I2C1. BMI160 will later share I2C0.
    left_sensor.init();
    right_sensor.init();

    left_motor.linkSensor(&left_sensor);
    right_motor.linkSensor(&right_sensor);

    // Preserve the reference SimpleFOC velocity-estimator PID values even though
    // the balancing application currently commands voltage-mode torque.
    left_motor.PID_velocity.P = kVelocityPidP;
    left_motor.PID_velocity.I = kVelocityPidI;
    left_motor.PID_velocity.D = kVelocityPidD;
    right_motor.PID_velocity.P = kVelocityPidP;
    right_motor.PID_velocity.I = kVelocityPidI;
    right_motor.PID_velocity.D = kVelocityPidD;

    left_motor.voltage_sensor_align = kSensorAlignmentVoltageV;
    right_motor.voltage_sensor_align = kSensorAlignmentVoltageV;
    left_driver.voltage_power_supply = kSupplyVoltageV;
    right_driver.voltage_power_supply = kSupplyVoltageV;

    // Explicit group assignment makes the dual-motor hardware resource mapping deterministic.
    if (left_driver.init(kMotor0McpwmGroup) == 0) {
        ESP_LOGE(kTag, "M0 MCPWM group %d initialization failed", kMotor0McpwmGroup);
        return ESP_FAIL;
    }
    left_motor.linkDriver(&left_driver);

    if (right_driver.init(kMotor1McpwmGroup) == 0) {
        ESP_LOGE(kTag, "M1 MCPWM group %d initialization failed", kMotor1McpwmGroup);
        left_driver.disable();
        return ESP_FAIL;
    }
    right_motor.linkDriver(&right_driver);

    left_motor.torque_controller = TorqueControlType::voltage;
    right_motor.torque_controller = TorqueControlType::voltage;
    left_motor.controller = MotionControlType::torque;
    right_motor.controller = MotionControlType::torque;

    // Preserve the reference startup order: M1 then M0.
    if (right_motor.init() == 0 || left_motor.init() == 0) {
        ESP_LOGE(kTag, "BLDC motor initialization failed");
        disableDrivers();
        return ESP_FAIL;
    }

    // initFOC performs encoder-direction/electrical-zero alignment and can move the wheels.
    if (right_motor.initFOC() == 0 || left_motor.initFOC() == 0) {
        ESP_LOGE(kTag, "FOC alignment failed");
        disableDrivers();
        return ESP_FAIL;
    }

    initialized = true;
    ESP_LOGI(kTag, "Dual voltage-torque FOC initialized");
    return ESP_OK;
}

WheelState runFocAndReadWheelState()
{
    if (!initialized) {
        return WheelState{0.0f, 0.0f, false};
    }

    // Keep the Arduino reference ordering: the previously staged target is consumed here.
    left_motor.loopFOC();
    right_motor.loopFOC();
    left_motor.move();
    right_motor.move();

    return WheelState{
        left_motor.shaft_velocity,
        right_motor.shaft_velocity,
        true,
    };
}

void stageTarget(const VoltageCommand &command)
{
    if (!initialized) {
        return;
    }

    left_motor.target = command.left_target_v;
    right_motor.target = command.right_target_v;
}

void disableOutputs()
{
    if (!initialized) {
        return;
    }

    left_motor.target = 0.0f;
    right_motor.target = 0.0f;
    left_motor.disable();
    right_motor.disable();
}

bool isInitialized()
{
    return initialized;
}

} // namespace motor
} // namespace vehicle
