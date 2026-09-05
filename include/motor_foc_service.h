/*
 * 电机模块在实现文件内保存两路AS5600、BLDCMotor和BLDCDriver3PWM。
 * initializePinsAndEncoders()连接Wire/Wire1与左右编码器。
 * runFocAndReadWheelState()执行当前周期FOC并返回轮速，stageTarget()准备下一周期目标。
 */
#pragma once

#include "app_types.h"

namespace vehicle {

// initializePinsAndEncoders()配置电机引脚、I2C总线和AS5600。
void initializePinsAndEncoders();
// initializeMotors()连接传感器与驱动器并启动FOC，在编码器初始化后调用。
void initializeMotors();
// runFocAndReadWheelState()运行左右电机FOC并返回更新后的轮速快照，仅由控制执行链调用。
WheelState runFocAndReadWheelState();
// stageTarget()写入左右电机下一周期使用的电压目标，与FOC更新在同一执行链调用。
void stageTarget(const MotorVoltageCommand &command);

} // namespace vehicle
