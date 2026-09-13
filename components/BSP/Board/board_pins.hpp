#pragma once

#include "driver/gpio.h"

namespace pins {

/*
 * 板级映射把DengFOC V4原理图中的物理网络绑定到ESP32 GPIO。
 * BSP驱动从这里取得I2C、三相PWM、使能、母线电压和相电流采样引脚。
 * static_assert保留启动绑带脚和ADC1输入专用脚的硬件约束。
 */
// i2c_sda_M0和i2c_scl_M0把M0编码器与BMI160接入同一I2C0控制器。
inline constexpr gpio_num_t i2c_sda_M0 = GPIO_NUM_19;
inline constexpr gpio_num_t i2c_scl_M0 = GPIO_NUM_18;

// i2c_scl_M1使用GPIO5。
// GPIO5的复位电平同时受启动绑带采样约束。
inline constexpr gpio_num_t i2c_sda_M1 = GPIO_NUM_23;
inline constexpr gpio_num_t i2c_scl_M1 = GPIO_NUM_5;

// M0三相PWM。
// GPIO22为编码器CS0，不是独立电机使能。
inline constexpr gpio_num_t motor_pwm_a_M0 = GPIO_NUM_32;
inline constexpr gpio_num_t motor_pwm_b_M0 = GPIO_NUM_33;
inline constexpr gpio_num_t motor_pwm_c_M0 = GPIO_NUM_25;

// M1三相PWM。
// GPIO14同时为默认JTAG脚。
inline constexpr gpio_num_t motor_pwm_a_M1 = GPIO_NUM_26;
inline constexpr gpio_num_t motor_pwm_b_M1 = GPIO_NUM_27;
inline constexpr gpio_num_t motor_pwm_c_M1 = GPIO_NUM_14;
// motor_enable连接两轮共享的10V栅极驱动电源M_EN，仅电机服务管理。
inline constexpr gpio_num_t motor_enable = GPIO_NUM_12;

// bus_voltage_adc把分压后的VIN_MEA连接到ADC2。
// ADC2与Wi-Fi共用射频资源，运行期间不能作为连续测量依据。
inline constexpr gpio_num_t bus_voltage_adc = GPIO_NUM_13;

// current_sensor_out_a_M0和current_sensor_out_b_M0接收M0两路相电流采样。
inline constexpr gpio_num_t current_sensor_out_a_M0 = GPIO_NUM_39;
inline constexpr gpio_num_t current_sensor_out_b_M0 = GPIO_NUM_36;

// current_sensor_out_a_M1和current_sensor_out_b_M1接收M1两路相电流采样。
inline constexpr gpio_num_t current_sensor_out_a_M1 = GPIO_NUM_35;
inline constexpr gpio_num_t current_sensor_out_b_M1 = GPIO_NUM_34;

// IsStrappingPin()判断引脚是否在ESP32复位期间参与启动配置采样。
inline constexpr bool IsStrappingPin(gpio_num_t pin)
{
    return pin == GPIO_NUM_0 || pin == GPIO_NUM_2 || pin == GPIO_NUM_5 ||
           pin == GPIO_NUM_12 || pin == GPIO_NUM_15;
}

// IsInputOnlyAdc1Pin()判断引脚是否属于GPIO34/35/36/39的ADC1输入专用组。
inline constexpr bool IsInputOnlyAdc1Pin(gpio_num_t pin)
{
    return pin == GPIO_NUM_34 || pin == GPIO_NUM_35 ||
           pin == GPIO_NUM_36 || pin == GPIO_NUM_39;
}

// 启动绑带脚和ADC1输入专用脚的约束在编译期保持可见。
static_assert(IsStrappingPin(i2c_scl_M1), "GPIO5 strapping constraint must remain visible");
static_assert(IsStrappingPin(motor_enable), "GPIO12 strapping constraint must remain visible");
static_assert(IsInputOnlyAdc1Pin(current_sensor_out_a_M0));
static_assert(IsInputOnlyAdc1Pin(current_sensor_out_b_M0));
static_assert(IsInputOnlyAdc1Pin(current_sensor_out_a_M1));
static_assert(IsInputOnlyAdc1Pin(current_sensor_out_b_M1));

} // namespace pins
