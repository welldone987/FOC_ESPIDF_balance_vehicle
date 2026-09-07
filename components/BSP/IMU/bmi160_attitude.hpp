#pragma once

#include "esp_err.h"

namespace vehicle {
namespace imu {

/*
 * 姿态模块从BMI160读取陀螺仪和加速度计原始数据。
 * readAttitude()把角速度积分与加速度俯仰角按0.98/0.02互补融合。
 * AttitudeSample把姿态角、角速度和本次读取有效性传给控制任务。
 */
struct AttitudeSample {
    // pitch_deg保存互补滤波后的俯仰角，单位deg。
    float pitch_deg;
    // pitch_rate_deg_s保存BMI160 Y轴角速度，单位deg/s。
    float pitch_rate_deg_s;
    // valid标记本次I2C读取与滤波结果是否可用于控制。
    bool valid;
};

// initialize()创建或复用I2C0，配置BMI160并完成陀螺仪硬件偏置校准。
// Creates/reuses I2C0, initializes BMI160 at 0x69, performs the same gyro
// hardware offset calibration used by BMI160Gen, and configures ±2g / ±1000 dps.
esp_err_t initialize();

// resetEstimator()把滤波角度清零，并从当前时刻重新开始积分计时。
// Matches the reference preInterval assignment performed after motor FOC init.
void resetEstimator();

// readAttitude()读取一帧陀螺仪和加速度计数据并返回互补滤波结果。
// Reads gyro+accelerometer in one burst and applies the reference 0.98/0.02
// complementary filter. Pitch is returned in degrees.
AttitudeSample readAttitude();

// isInitialized()报告BMI160初始化状态。
bool isInitialized();

} // namespace imu
} // namespace vehicle
