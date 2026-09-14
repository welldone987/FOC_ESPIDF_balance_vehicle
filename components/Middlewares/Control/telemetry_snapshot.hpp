#pragma once

#include <cstdint>

namespace vehicle {
namespace control {

/*
 * TelemetrySnapshot保存控制任务发布的一帧定长遥测数据。
 * BLE遥测特征与Wi-Fi TCP消费同一类型，通过长度1静态队列传递最新值。
 */
struct TelemetrySnapshot {
    // device_time_us保存控制周期采样时刻，单位us。
    std::int64_t device_time_us;
    // sequence标识控制任务发布的快照版本。
    std::uint32_t sequence;
    // pitch_deg保存姿态估计俯仰角，单位deg。
    float pitch_deg;
    // velocity_M0_rad_s保存M0轮角速度，单位rad/s。
    float velocity_M0_rad_s;
    // velocity_M1_rad_s保存M1轮角速度，单位rad/s。
    float velocity_M1_rad_s;
    // target_M0_A保存M0电机目标电流，单位A。
    float target_M0_A;
    // target_M1_A保存M1电机目标电流，单位A。
    float target_M1_A;
    // iq_measured_M0_A和uq_applied_M0_V保存M0的Iq反馈与Uq输出，单位A、V。
    float iq_measured_M0_A;
    float iq_measured_M1_A;
    float uq_applied_M0_V;
    float uq_applied_M1_V;
    // phase_a_M0_A、phase_b_M0_A、phase_c_M0_A等六项保存两轮三相采样电流，单位A。
    float phase_a_M0_A;
    float phase_b_M0_A;
    float phase_c_M0_A;
    float phase_a_M1_A;
    float phase_b_M1_A;
    float phase_c_M1_A;
    // current_dt_s是电流环实际周期，单位s。
    float current_dt_s;
    // current_sample_age_us是电流采样到发布的年龄，单位us。
    std::int64_t current_sample_age_us;
    // current_saturated标记本周期电流环是否饱和。
    bool current_saturated;
    // current_valid停机时为false，零值不代表有效测量。
    bool current_valid;
};

} // namespace control
} // namespace vehicle
