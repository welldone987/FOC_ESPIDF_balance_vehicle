#pragma once

#include "esp_err.h"
#include "telemetry_snapshot.hpp"

namespace vehicle {
namespace wifi_telemetry {

/*
 * Wi-Fi遥测模块把控制任务发布的最新快照编码为LF分隔的TCP文本帧。
 * Service()使用非阻塞socket，只维护一个客户端和一份待发送缓冲区。
 */

// Initialize()初始化Wi-Fi station、事件回调和TCP遥测所需网络资源。
esp_err_t Initialize();
// Service()在服务任务中推进连接、发送待发送帧并编码新快照。
void Service(const control::TelemetrySnapshot *snapshot);

} // namespace wifi_telemetry
} // namespace vehicle
