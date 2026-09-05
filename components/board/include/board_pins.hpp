#pragma once

#include "driver/gpio.h"

namespace board {
namespace pins {

// DengFOC V4 / ESP32-WROOM-32 board-level GPIO mapping.
// Keep physical routing here; drivers and application code must not hard-code GPIO numbers.

// I2C0: M0 AS5600 + BMI160.
inline constexpr gpio_num_t kI2c0Sda = GPIO_NUM_19;
inline constexpr gpio_num_t kI2c0Scl = GPIO_NUM_18;

// I2C1: M1 AS5600. GPIO5 is an ESP32 strapping pin.
inline constexpr gpio_num_t kI2c1Sda = GPIO_NUM_23;
inline constexpr gpio_num_t kI2c1Scl = GPIO_NUM_5;

// Motor 0 three-phase PWM and enable.
inline constexpr gpio_num_t kMotor0PwmA = GPIO_NUM_32;
inline constexpr gpio_num_t kMotor0PwmB = GPIO_NUM_33;
inline constexpr gpio_num_t kMotor0PwmC = GPIO_NUM_25;
inline constexpr gpio_num_t kMotor0Enable = GPIO_NUM_22;

// Motor 1 three-phase PWM and enable.
// GPIO12 is both a strapping pin and a default JTAG pin; GPIO14 is also a default JTAG pin.
inline constexpr gpio_num_t kMotor1PwmA = GPIO_NUM_26;
inline constexpr gpio_num_t kMotor1PwmB = GPIO_NUM_27;
inline constexpr gpio_num_t kMotor1PwmC = GPIO_NUM_14;
inline constexpr gpio_num_t kMotor1Enable = GPIO_NUM_12;

// Existing board routing for battery voltage sensing.
// GPIO13 is ADC2 on classic ESP32 and therefore conflicts with Wi-Fi ADC usage.
inline constexpr gpio_num_t kBatteryVoltageAdc = GPIO_NUM_13;

inline constexpr bool isStrappingPin(gpio_num_t pin)
{
    return pin == GPIO_NUM_0 || pin == GPIO_NUM_2 || pin == GPIO_NUM_5 ||
           pin == GPIO_NUM_12 || pin == GPIO_NUM_15;
}

static_assert(isStrappingPin(kI2c1Scl), "GPIO5 strapping constraint must remain visible");
static_assert(isStrappingPin(kMotor1Enable), "GPIO12 strapping constraint must remain visible");

} // namespace pins
} // namespace board
