/*
 * app_types把命令输入、IMU采样、姿态估计、轮速和控制输出连接为固定大小值类型。
 * runVehicleControlOnce()使用这些值类型串联单周期控制数据，并构建TelemetrySnapshot。
 */
#pragma once

#include <stdint.h>

namespace vehicle {

// RemoteCommand保存BLE命令缩放后的转向电压、目标轮速和接收状态。
struct RemoteCommand {
  // steering_voltage_v保存转向控制电压，单位V。
  float steering_voltage_v;
  // throttle_velocity_rad_s保存目标轮速，单位rad/s。
  float throttle_velocity_rad_s;
  // sequence记录已提交命令的递增序号。
  uint32_t sequence;
  // received_ms记录最近命令的接收时刻，单位ms。
  uint32_t received_ms;
  // connected记录BLE客户端连接状态。
  bool connected;
};

// ImuSample保存BMI160换算后的三轴加速度、俯仰角速度和采样状态。
struct ImuSample {
  // acceleration_x_g保存X轴加速度，单位g。
  float acceleration_x_g;
  // acceleration_y_g保存Y轴加速度，单位g。
  float acceleration_y_g;
  // acceleration_z_g保存Z轴加速度，单位g。
  float acceleration_z_g;
  // pitch_rate_deg_s保存俯仰角速度，单位deg/s。
  float pitch_rate_deg_s;
  // read_started_us为主机开始读取时刻，不是传感器内部采样时刻。
  int64_t read_started_us;
  // valid标记本次IMU采样是否可用于姿态估计。
  bool valid;
};

// AttitudeEstimate保存互补滤波输出的俯仰状态。
struct AttitudeEstimate {
  // pitch_deg保存融合后的俯仰角，单位deg。
  float pitch_deg;
  // pitch_rate_deg_s保存融合状态中的俯仰角速度，单位deg/s。
  float pitch_rate_deg_s;
  // interval_s保存本次姿态更新的采样间隔，单位s。
  float interval_s;
  // valid标记姿态估计是否可用于控制计算。
  bool valid;
};

// WheelState保存左右轮当前角速度快照。
struct WheelState {
  // left_velocity_rad_s保存左轮角速度，单位rad/s。
  float left_velocity_rad_s;
  // right_velocity_rad_s保存右轮角速度，单位rad/s。
  float right_velocity_rad_s;
  bool valid;
};

// MotorVoltageCommand保存左右电机下一控制周期使用的目标电压。
struct MotorVoltageCommand {
  // left_target_v保存左电机目标电压，单位V。
  float left_target_v;
  // right_target_v保存右电机目标电压，单位V。
  float right_target_v;
};

// ControlOutput保存速度、姿态和转向控制链的中间结果与电机指令。
struct ControlOutput {
  // pitch_error_deg保存姿态控制使用的俯仰角误差，单位deg。
  float pitch_error_deg;
  // target_pitch_deg保存速度外环生成的目标俯仰角，单位deg。
  float target_pitch_deg;
  // balance_voltage_v保存姿态内环输出的平衡电压，单位V。
  float balance_voltage_v;
  // steering_voltage_v保存转向滤波后的转向电压，单位V。
  float steering_voltage_v;
  // motor_voltage保存混合器生成的左右电机目标电压。
  MotorVoltageCommand motor_voltage;
};

// 故障只由ControlTask锁存，恢复需要重启，普通命令不能清除。
enum class ControlFault : uint32_t { none = 0U, initialization = 1U,
  imu = 2U, encoder = 4U, timing = 8U, heartbeat = 16U };

// TelemetrySnapshot汇总单周期命令、传感器、姿态、轮速和控制数据。
struct TelemetrySnapshot {
  // device_time_us记录控制周期起点，64位单调微秒，不受millis回绕影响。
  int64_t device_time_us;
  // sequence记录快照对应的控制周期序号。
  uint32_t sequence;
  // sampled_ms记录快照采样时刻，单位ms。
  uint32_t sampled_ms;
  // control_interval_us记录相邻控制周期的间隔，单位us。
  uint32_t control_interval_us;
  RemoteCommand command;
  ImuSample imu;
  AttitudeEstimate attitude;
  WheelState wheels;
  ControlOutput control;
  // bus_voltage_v保存启动阶段读取的母线电压，单位V。
  float bus_voltage_v;
  // imu_ready标记BMI160是否完成初始化。
  bool imu_ready;
  int64_t completed_us;
  uint32_t execution_us;
  uint32_t deadline_misses;
  ControlFault fault;

};

} // namespace vehicle
