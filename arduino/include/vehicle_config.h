/*
 * vehicle_config集中保存BLE、I2C、BMI160、电机、电源和控制器的编译期参数。
 * config中的带单位constexpr直接连接硬件映射、量纲换算和控制器边界。
 */
#pragma once

#include <stdint.h>

namespace vehicle {
namespace config {

// 周期配置：速度环仅支持200/100Hz，1kHz基准分别分频5/10。
constexpr uint32_t control_rate_hz{1000U};
constexpr uint32_t speed_rate_hz{200U};
constexpr uint32_t control_period_us{1000000U / control_rate_hz};
constexpr uint32_t speed_divider{control_rate_hz / speed_rate_hz};
static_assert(speed_rate_hz == 200U || speed_rate_hz == 100U, "Speed rate must be 200 or 100 Hz");
static_assert(control_rate_hz % speed_rate_hz == 0U, "Integral speed divider required");
constexpr uint16_t i2c_timeout_ms{1U};
constexpr uint32_t command_timeout_ms{500U};
constexpr uint32_t health_period_ms{10U};
constexpr uint32_t heartbeat_timeout_us{20000U};
constexpr uint32_t consecutive_overrun_limit{5U};
// 1ms时alpha=0.98；这是迁移起点，仍需实机确认滤波响应。
constexpr float attitude_time_constant_s{0.049F};
constexpr uint32_t maximum_attitude_interval_us{20000U};
// TCP遥测默认50Hz；仅影响网络消费者，不改变控制周期。
constexpr uint32_t telemetry_period_ms{20U};
constexpr uint16_t telemetry_tcp_port{3333U};

// serial_baud提供串口初始化使用的波特率。
constexpr uint32_t serial_baud{115200U};

// ble_device_name、ble_service_uuid和ble_characteristic_uuid定义BLE身份与命令入口。
constexpr char ble_device_name[]{"平衡车"};
constexpr char ble_service_uuid[]{"6e400001-b5a3-f393-e0a9-e50e24dcca9e"};
constexpr char ble_characteristic_uuid[]{"6e400002-b5a3-f393-e0a9-e50e24dcca9e"};
// maximum_throttle_velocity_rad_s提供油门换算上限，单位rad/s。
constexpr float maximum_throttle_velocity_rad_s{10.0F};
// maximum_steering_voltage_v提供转向换算上限，单位V。
constexpr float maximum_steering_voltage_v{10.0F};
// ble_full_scale_steering提供转向原始量满量程。
constexpr float ble_full_scale_steering{1500.0F};
// ble_full_scale_throttle提供油门原始量满量程。
constexpr float ble_full_scale_throttle{40.0F};
// maximum_ble_command_length限定BLE命令缓冲区的输入长度。
constexpr uint32_t maximum_ble_command_length{63U};

// i2c0_sda_pin、i2c0_scl_pin、i2c1_sda_pin和i2c1_scl_pin映射两条I2C总线。
constexpr int i2c0_sda_pin{19};
constexpr int i2c0_scl_pin{18};
constexpr int i2c1_sda_pin{23};
constexpr int i2c1_scl_pin{5};
// i2c_frequency_hz提供两条I2C总线的工作速率，单位Hz。
constexpr uint32_t i2c_frequency_hz{400000U};
// bmi160_i2c_address指定BMI160的I2C地址。
constexpr uint8_t bmi160_i2c_address{0x69U};
// bmi160_gyro_range_deg_s提供陀螺仪量程，单位deg/s。
constexpr int bmi160_gyro_range_deg_s{1000};
// bmi160_gyro_rate_hz提供陀螺仪输出数据率，单位Hz。
constexpr int bmi160_gyro_rate_hz{1600};
// bmi160_acceleration_scale把BMI160原始加速度计数换算为g。
constexpr float bmi160_acceleration_scale{16384.0F};
// bmi160_gyro_scale把BMI160原始陀螺仪计数换算为deg/s。
constexpr float bmi160_gyro_scale{32.8F};

constexpr int motor0_pwm_a_pin{32};
constexpr int motor0_pwm_b_pin{33};
constexpr int motor0_pwm_c_pin{25};
constexpr int motor0_enable_pin{22};
constexpr int motor1_pwm_a_pin{26};
constexpr int motor1_pwm_b_pin{27};
constexpr int motor1_pwm_c_pin{14};
constexpr int motor1_enable_pin{12};
// motor_pole_pairs提供两台电机共用的极对数。
constexpr int motor_pole_pairs{7};
// motor0_direction提供左电机的方向系数。
constexpr int motor0_direction{1};
// motor1_direction提供右电机的方向系数。
constexpr int motor1_direction{1};
// motor_supply_voltage_v提供驱动器母线电压，单位V。
constexpr float motor_supply_voltage_v{12.0F};
// motor_sensor_alignment_voltage_v提供传感器校准电压，单位V。
constexpr float motor_sensor_alignment_voltage_v{2.0F};

constexpr int battery_voltage_adc_pin{13};
// battery_voltage_scale恢复电阻分压前的母线电压。
constexpr float battery_voltage_scale{8.5F};
// startup_undervoltage_threshold_v提供启动母线电压阈值，单位V。
constexpr float startup_undervoltage_threshold_v{9.0F};
// startup_power_poll_interval_ms提供启动电压轮询间隔，单位ms。
constexpr uint32_t startup_power_poll_interval_ms{100U};

// pitch_offset_deg提供姿态控制零点偏移，单位deg。
constexpr float pitch_offset_deg{1.8F};
constexpr float balance_pid_p{0.31F};
constexpr float balance_pid_i{0.0F};
constexpr float balance_pid_d{0.001F};
constexpr float balance_pid_ramp{100000.0F};
// balance_pid_limit_v限制姿态内环输出电压，单位V。
constexpr float balance_pid_limit_v{6.0F};
constexpr float speed_pid_p{1.50F};
constexpr float speed_pid_i{0.0F};
constexpr float speed_pid_d{0.05F};
constexpr float speed_pid_ramp{10000.0F};
// speed_pid_limit_deg限制速度外环生成的目标俯仰角，单位deg。
constexpr float speed_pid_limit_deg{6.0F};
// pitch_command_filter_time_constant_s提供目标俯仰角滤波时间常数，单位s。
constexpr float pitch_command_filter_time_constant_s{0.07F};
// throttle_filter_time_constant_s提供目标轮速滤波时间常数，单位s。
constexpr float throttle_filter_time_constant_s{0.5F};
// steering_filter_time_constant_s提供转向电压滤波时间常数，单位s。
constexpr float steering_filter_time_constant_s{0.1F};

// simplefoc_velocity_pid_p、simplefoc_velocity_pid_i和simplefoc_velocity_pid_d配置SimpleFOC速度环增益。
constexpr float simplefoc_velocity_pid_p{0.01F};
constexpr float simplefoc_velocity_pid_i{0.1F};
constexpr float simplefoc_velocity_pid_d{0.0F};

} // namespace config
} // namespace vehicle
