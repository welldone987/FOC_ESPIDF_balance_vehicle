#pragma once

#include "esp_err.h"

namespace vehicle {
namespace motor {

/*
 * 电机服务把两个AS5600轮速传感器、两个三相驱动器和SimpleFOC调用顺序封装在BSP边界内。
 * 控制任务先消耗上一周期目标，再读取轮速；stageTarget()保存下一周期的目标电压。
 */
struct WheelState {
    // left_velocity_rad_s保存左轮轴角速度，单位rad/s。
    float left_velocity_rad_s;
    // right_velocity_rad_s保存右轮轴角速度，单位rad/s。
    float right_velocity_rad_s;
    // valid标记本次FOC和编码器读数是否可用于控制。
    bool valid;
};

struct VoltageCommand {
    // left_target_v保存下一次move()使用的左电机目标电压，单位V。
    float left_target_v;
    // right_target_v保存下一次move()使用的右电机目标电压，单位V。
    float right_target_v;
};

// initialize()初始化编码器、三相驱动器和电机，并执行可能使车轮转动的FOC对齐。
esp_err_t initialize();

// runFocAndReadWheelState()执行loopFOC()、move()并返回本轮编码器速度。
WheelState runFocAndReadWheelState();

// stageTarget()把本轮控制输出保存为下一次move()使用的电压目标。
void stageTarget(const VoltageCommand &command);

// disableOutputs()把电机目标清零并关闭两个驱动器。
void disableOutputs();


} // namespace motor
} // namespace vehicle
