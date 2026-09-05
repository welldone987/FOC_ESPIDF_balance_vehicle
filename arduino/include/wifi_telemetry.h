#pragma once
#include "app_types.h"
namespace vehicle {
// 仅CommunicationTask调用；ADC启动自检结束后才允许启动Wi-Fi。
bool beginWifiTelemetry();
// 非阻塞socket维护及七字段CSV发送；输入必须是任务自己的快照副本。
void serviceWifiTelemetry(const TelemetrySnapshot &snapshot);
}
