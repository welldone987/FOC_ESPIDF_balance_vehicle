#pragma once

#include <cstdint>

#include "esp_err.h"

namespace vehicle {
namespace wifi_telemtry {

/*
 * Wi-Fi遥测模块把控制任务发布的最新快照编码为LF分隔的TCP文本帧。
 * service()使用非阻塞socket，只维护一个客户端和一份待发送缓冲区。
 */
struct TelemetrySnapshot {
    // device_time_us保存控制周期采样时刻，单位us。
    std::int64_t device_time_us;
    // sequence标识控制任务发布的快照版本。
    std::uint32_t sequence;
    // pitch_deg保存姿态估计俯仰角，单位deg。
    float pitch_deg;
    // left_velocity_rad_s保存左轮角速度，单位rad/s。
    float left_velocity_rad_s;
    // right_velocity_rad_s保存右轮角速度，单位rad/s。
    float right_velocity_rad_s;
    // left_target_a保存左电机目标电流，单位A。
    float left_target_a;
    // right_target_a保存右电机目标电流，单位A。
    float right_target_a;
};

// initialize()初始化Wi-Fi station、事件回调和TCP遥测所需网络资源。
esp_err_t initialize();
// service()在服务任务中推进连接、发送待发送帧并编码新快照。
void service(const TelemetrySnapshot *snapshot);

} // namespace wifi_telemtry
} // namespace vehicle
