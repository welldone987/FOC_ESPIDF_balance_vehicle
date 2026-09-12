#pragma once

#include <cstdint>

#include "esp_err.h"

namespace vehicle {
namespace wifi_telemetry {

/*
 * Wi-Fi遥测模块把控制任务发布的最新快照编码为LF分隔的TCP文本帧。
 * Service()使用非阻塞socket，只维护一个客户端和一份待发送缓冲区。
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
    float iq_measured_M0_A;
    float iq_measured_M1_A;
    float uq_applied_M0_V;
    float uq_applied_M1_V;
    float phase_a_M0_A;
    float phase_b_M0_A;
    float phase_c_M0_A;
    float phase_a_M1_A;
    float phase_b_M1_A;
    float phase_c_M1_A;
    float current_dt_s;
    std::int64_t current_sample_age_us;
    bool current_saturated;
    bool current_valid; // 停机时false，零值不代表有效测量。
};

// Initialize()初始化Wi-Fi station、事件回调和TCP遥测所需网络资源。
esp_err_t Initialize();
// Service()在服务任务中推进连接、发送待发送帧并编码新快照。
void Service(const TelemetrySnapshot *snapshot);

} // namespace wifi_telemetry
} // namespace vehicle
