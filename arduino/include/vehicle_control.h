/*
 * 车辆控制模块把RemoteCommand、WheelState和AttitudeEstimate转换为左右电机目标电压。
 * VehicleController先由speed_pid_和pitch_command_filter_生成目标俯仰角。
 * balance_pid_把俯仰误差转换为平衡电压，steering_filter_生成转向电压。
 * VehicleController把平衡电压和转向电压组合为左右电机目标。
 * initializeVehicle()完成启动，runVehicleControlOnce()串联采样、计算和下一周期目标暂存。
 */
#pragma once

#include "app_types.h"
#include <SimpleFOC.h>

namespace vehicle {

// VehicleController把RemoteCommand、WheelState和AttitudeEstimate转换为ControlOutput。
class VehicleController {
public:
  VehicleController();

  // update()按速度外环、姿态内环和转向滤波生成本轮ControlOutput。
  ControlOutput update(const RemoteCommand &command,
                       const WheelState &wheels,
                       const AttitudeEstimate &attitude);

private:
  // mixMotorVoltages()把平衡电压和转向电压混合为左右电机目标电压。
  static MotorVoltageCommand mixMotorVoltages(float balance_voltage_v,
                                              float steering_voltage_v);
  // balance_pid_把姿态误差转换为共同平衡电压，单位V。
  PIDController balance_pid_;
  // speed_pid_把轮速误差转换为目标俯仰角，单位deg。
  PIDController speed_pid_;
  // pitch_command_filter_平滑速度外环生成的目标俯仰角，单位deg。
  LowPassFilter pitch_command_filter_;
  // throttle_filter_平滑遥控目标轮速，单位rad/s。
  LowPassFilter throttle_filter_;
  // steering_filter_平滑遥控转向电压，单位V。
  LowPassFilter steering_filter_;
};

// initializeVehicle()按电源、引脚与编码器、命令输入、IMU和电机顺序完成一次启动。
void initializeVehicle();
// runVehicleControlOnce()在启动后执行一轮采样、控制和遥测更新，仅由控制执行链调用。
void runVehicleControlOnce();
// latestTelemetry()返回模块内最近一轮快照的只读引用，仅供同一执行上下文读取。
const TelemetrySnapshot &latestTelemetry();

} // namespace vehicle
