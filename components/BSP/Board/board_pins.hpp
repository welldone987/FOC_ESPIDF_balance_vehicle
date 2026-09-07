#pragma once

#include "driver/gpio.h"

namespace board {
namespace pins {

/*
 * 板级映射把DengFOC V4原理图中的物理网络绑定到ESP32 GPIO。
 * BSP驱动从这里取得I2C、三相PWM、使能、母线电压和相电流采样引脚。
 * static_assert保留启动绑带脚和ADC1输入专用脚的硬件约束。
 */
// kI2c0Sda和kI2c0Scl把M0编码器与BMI160接入同一I2C0控制器。
inline constexpr gpio_num_t kI2c0Sda = GPIO_NUM_19;
inline constexpr gpio_num_t kI2c0Scl = GPIO_NUM_18;

// kI2c1Scl使用GPIO5；该引脚的复位电平同时受启动绑带采样约束。
inline constexpr gpio_num_t kI2c1Sda = GPIO_NUM_23;
inline constexpr gpio_num_t kI2c1Scl = GPIO_NUM_5;

// kMotor0PwmA/B/C和kMotor0Enable分别连接M0三相输入与硬件使能。
inline constexpr gpio_num_t kMotor0PwmA = GPIO_NUM_32;
inline constexpr gpio_num_t kMotor0PwmB = GPIO_NUM_33;
inline constexpr gpio_num_t kMotor0PwmC = GPIO_NUM_25;
inline constexpr gpio_num_t kMotor0Enable = GPIO_NUM_22;

// kMotor1Enable和kMotor1PwmC占用启动绑带脚及默认JTAG脚，运行时必须保持板级电气约束。
inline constexpr gpio_num_t kMotor1PwmA = GPIO_NUM_26;
inline constexpr gpio_num_t kMotor1PwmB = GPIO_NUM_27;
inline constexpr gpio_num_t kMotor1PwmC = GPIO_NUM_14;
inline constexpr gpio_num_t kMotor1Enable = GPIO_NUM_12;

// kBatteryVoltageAdc把分压后的VIN_MEA连接到ADC2；Wi-Fi运行时不能把ADC2读数当作连续测量依据。
inline constexpr gpio_num_t kBatteryVoltageAdc = GPIO_NUM_13;

// kMotor0CurrentSenseOut1和kMotor0CurrentSenseOut2接收M0两路相电流采样。
inline constexpr gpio_num_t kMotor0CurrentSenseOut1 = GPIO_NUM_39;
inline constexpr gpio_num_t kMotor0CurrentSenseOut2 = GPIO_NUM_36;

// 四路电流采样引脚均为ESP32输入专用GPIO，只能作为ADC输入使用。
inline constexpr gpio_num_t kMotor1CurrentSenseOut1 = GPIO_NUM_35;
inline constexpr gpio_num_t kMotor1CurrentSenseOut2 = GPIO_NUM_34;

inline constexpr bool isStrappingPin(gpio_num_t pin)
{
    // isStrappingPin标记复位期间会参与ESP32启动配置采样的GPIO。
    return pin == GPIO_NUM_0 || pin == GPIO_NUM_2 || pin == GPIO_NUM_5 ||
           pin == GPIO_NUM_12 || pin == GPIO_NUM_15;
}

inline constexpr bool isInputOnlyAdc1Pin(gpio_num_t pin)
{
    // isInputOnlyAdc1Pin标记GPIO34/35/36/39的ADC1输入专用约束。
    return pin == GPIO_NUM_34 || pin == GPIO_NUM_35 ||
           pin == GPIO_NUM_36 || pin == GPIO_NUM_39;
}

static_assert(isStrappingPin(kI2c1Scl), "GPIO5 strapping constraint must remain visible");
static_assert(isStrappingPin(kMotor1Enable), "GPIO12 strapping constraint must remain visible");
static_assert(isInputOnlyAdc1Pin(kMotor0CurrentSenseOut1));
static_assert(isInputOnlyAdc1Pin(kMotor0CurrentSenseOut2));
static_assert(isInputOnlyAdc1Pin(kMotor1CurrentSenseOut1));
static_assert(isInputOnlyAdc1Pin(kMotor1CurrentSenseOut2));

} // namespace pins
} // namespace board
