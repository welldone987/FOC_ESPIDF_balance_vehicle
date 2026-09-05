#include "board_pins.hpp"
#include "esp_log.h"
#include "motor_foc_service.hpp"

namespace {
constexpr char kTag[] = "app";
constexpr int kCppBuildProbe = 42;
} // namespace

extern "C" void app_main(void)
{
    ESP_LOGI(kTag, "ESP-IDF C++ build probe passed: %d", kCppBuildProbe);
    ESP_LOGI(kTag,
             "DengFOC V4 board map compiled: I2C0=%d/%d I2C1=%d/%d M0_EN=%d M1_EN=%d",
             static_cast<int>(board::pins::kI2c0Sda),
             static_cast<int>(board::pins::kI2c0Scl),
             static_cast<int>(board::pins::kI2c1Sda),
             static_cast<int>(board::pins::kI2c1Scl),
             static_cast<int>(board::pins::kMotor0Enable),
             static_cast<int>(board::pins::kMotor1Enable));

    // Phase 1 only links the native ESP-IDF motor service. Do not call
    // vehicle::motor::initialize() yet: it performs FOC alignment and energizes
    // the inverter. Hardware startup will be connected after the remaining
    // safety/IMU/power migration steps are in place.
    ESP_LOGI(kTag,
             "esp_simplefoc motor service linked; outputs remain disabled (initialized=%s)",
             vehicle::motor::isInitialized() ? "true" : "false");
}
