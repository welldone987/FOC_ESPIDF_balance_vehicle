#pragma once

#include "error_info.hpp"

namespace vehicle {
namespace imu {

/*
 * 姿态模块从BMI160读取陀螺仪和加速度计原始数据。
 * ReadAttitude()按实际采样间隔与配置时间常数融合角速度积分和加速度俯仰角。
 * AttitudeSample把姿态角、角速度和本次读取有效性传给控制任务。
 */
// AttitudeSample保存一帧融合后的俯仰角和角速度。
struct AttitudeSample {
    // pitch_deg保存互补滤波后的俯仰角，单位deg。
    float pitch_deg;
    // pitch_rate_deg_s保存BMI160 Y轴角速度，单位deg/s。
    float pitch_rate_deg_s;
    // valid标记本次I2C读取与滤波结果是否可用于控制。
    bool valid;
};

// Initialize()创建或复用I2C0，配置BMI160并完成陀螺仪硬件偏置校准。
esp_err_t Initialize(ErrorInfo *error=nullptr);

// ResetEstimator()清除历史状态。
// 下个有效样本以实测加速度计倾角建立基准。
void ResetEstimator();

// ReadAttitude()读取一帧陀螺仪和加速度计数据并返回互补滤波结果。
esp_err_t ReadAttitude(AttitudeSample *out, ErrorInfo *error=nullptr);


} // namespace imu
} // namespace vehicle
