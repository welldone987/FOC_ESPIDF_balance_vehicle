#pragma once

#include <cstdint>

namespace vehicle {
namespace control {

/*
 * 控制器把统一坐标系下的轮速、姿态和BLE命令转换为左右电机目标电压。
 * update()先计算速度外环和目标俯仰角，再计算姿态内环并叠加转向差分量。
 */
struct ControlInput {
    // left_velocity_rad_s保存方向统一前的左轮角速度，单位rad/s。
    float left_velocity_rad_s;
    // right_velocity_rad_s保存方向统一前的右轮角速度，单位rad/s。
    float right_velocity_rad_s;
    // pitch_deg保存姿态估计输出的俯仰角，单位deg。
    float pitch_deg;
    // steering_voltage_v保存BLE缩放后的转向差分电压，单位V。
    float steering_voltage_v;
    // throttle_velocity_rad_s保存BLE缩放后的目标平均轮速，单位rad/s。
    float throttle_velocity_rad_s;
};

struct ControlOutput {
    // target_pitch_deg保存速度环要求的目标俯仰角，单位deg。
    float target_pitch_deg;
    // balance_voltage_v保存姿态环生成的左右共同电压，单位V。
    float balance_voltage_v;
    // steering_voltage_v保存滤波后的左右差分电压，单位V。
    float steering_voltage_v;
    // left_target_v保存左电机下一控制周期使用的目标电压，单位V。
    float left_target_v;
    // right_target_v保存右电机下一控制周期使用的目标电压，单位V。
    float right_target_v;
};

struct PidState {
    // p、i、d、ramp和limit保存PID增益、输出变化率限幅和输出限幅。
    float p;
    float i;
    float d;
    float ramp;
    float limit;
    // previous_error保存上一调用的误差，单位随输入误差变化。
    float previous_error;
    // previous_output保存上一调用的限幅后输出。
    float previous_output;
    // previous_integral保存梯形积分状态。
    float previous_integral;
    // previous_time_us保存上一次更新时刻，单位us。
    std::int64_t previous_time_us;
};

struct LowPassState {
    // time_constant_s保存一阶低通时间常数，单位s。
    float time_constant_s;
    // previous_output保存上一调用的滤波输出。
    float previous_output;
    // previous_time_us保存上一次滤波时刻，单位us。
    std::int64_t previous_time_us;
};

struct ControllerState {
    // balance_pid保存姿态内环状态。
    PidState balance_pid;
    // speed_pid保存轮速外环状态。
    PidState speed_pid;
    // 三个低通状态分别处理目标俯仰角、油门和转向命令。
    LowPassState pitch_filter;
    LowPassState throttle_filter;
    LowPassState steering_filter;
};

// initialize()用vehicle_config.hpp中的增益、限幅和滤波时间常数建立控制器状态。
void initialize(ControllerState &state);
// update()读取本周期输入并生成左右电机目标电压。
ControlOutput update(ControllerState &state, const ControlInput &input);

} // namespace control
} // namespace vehicle
