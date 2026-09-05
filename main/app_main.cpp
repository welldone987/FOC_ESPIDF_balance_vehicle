#include "balance_controller.hpp"
#include "ble_command_service.hpp"
#include "bmi160_attitude.hpp"
#include "motor_foc_service.hpp"
#include "power_monitor.hpp"
#include "vehicle_config.hpp"

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr char kTag[] = "balance_app";

void delayMs(std::uint32_t milliseconds)
{
    vTaskDelay(pdMS_TO_TICKS(milliseconds));
}

float waitForStartupVoltage()
{
    float bus_voltage_v = 0.0f;

    for (;;) {
        ESP_ERROR_CHECK(vehicle::power::readBusVoltage(&bus_voltage_v));
        if (bus_voltage_v > vehicle::config::kStartupUndervoltageThresholdV) {
            ESP_LOGI(kTag,
                     "Power ready: %.2f V, motor calibration may proceed",
                     static_cast<double>(bus_voltage_v));
            return bus_voltage_v;
        }

        ESP_LOGW(kTag,
                 "Waiting for power: %.2f V (threshold %.2f V)",
                 static_cast<double>(bus_voltage_v),
                 static_cast<double>(vehicle::config::kStartupUndervoltageThresholdV));
        delayMs(vehicle::config::kStartupPowerPollIntervalMs);
    }
}

[[noreturn]] void stopOnRuntimeFault(const char *reason)
{
    vehicle::motor::disableOutputs();
    ESP_LOGE(kTag, "Control stopped: %s", reason);
    for (;;) {
        delayMs(1000U);
    }
}

} // namespace

extern "C" void app_main(void)
{
    ESP_LOGI(kTag, "DengFOC V4 native ESP-IDF balance controller starting");

    // Construct before the blocking startup sequence so the PID/LPF first-call
    // timing behavior remains equivalent to the global SimpleFOC controllers in
    // the uploaded Arduino main.cpp.
    vehicle::control::BalanceController controller{};

    ESP_ERROR_CHECK(vehicle::power::initialize());
    const float startup_voltage_v = waitForStartupVoltage();
    (void)startup_voltage_v;

    // Native NimBLE replaces BLEDevice/BLEServer/BLECharacteristic. The BLE host
    // owns its internal stack task; this application still creates no control,
    // sensor, queue, or worker tasks.
    ESP_ERROR_CHECK(vehicle::ble::initialize());
    delayMs(100U);

    // BMI160 creates I2C0 first. esp_simplefoc's M0 AS5600 later requests the
    // same 400 kHz I2C0 configuration and reuses the i2c_bus singleton.
    ESP_ERROR_CHECK(vehicle::imu::initialize());

    // This is the first point that energizes the inverter: initFOC performs the
    // same M1 -> M0 alignment sequence as the reference implementation.
    ESP_ERROR_CHECK(vehicle::motor::initialize());

    // Reference setup() assigns preInterval only after motorInit().
    vehicle::imu::resetEstimator();

    ESP_LOGI(kTag, "Initialization complete; entering single-chain control loop");

    for (;;) {
        // Preserve the reference ordering: loopFOC() + move() consume the target
        // staged by the previous iteration before this iteration computes a new one.
        const vehicle::motor::WheelState wheels =
            vehicle::motor::runFocAndReadWheelState();
        if (!wheels.valid) {
            stopOnRuntimeFault("motor/encoder state invalid");
        }

        const vehicle::imu::AttitudeSample attitude =
            vehicle::imu::readAttitude();
        if (!attitude.valid) {
            stopOnRuntimeFault("BMI160 read or attitude estimate invalid");
        }

        const vehicle::ble::CommandSnapshot command =
            vehicle::ble::latestCommand();

        const vehicle::control::ControlOutput output = controller.update(
            vehicle::control::ControlInput{
                wheels.left_velocity_rad_s,
                wheels.right_velocity_rad_s,
                attitude.pitch_deg,
                command.steering_voltage_v,
                command.throttle_velocity_rad_s,
            });

        vehicle::motor::stageTarget(vehicle::motor::VoltageCommand{
            output.left_target_v,
            output.right_target_v,
        });
    }
}
