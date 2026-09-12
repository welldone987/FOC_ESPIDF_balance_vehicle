#pragma once
#include "error_info.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
namespace vehicle {
namespace ble {
/*
 * ble_command_service在BleTask中运行NimBLE主机并维护GATT服务。
 * GAP/GATT回调只复制原始报文，Run()解析X,Y并写入长度1命令队列。
 * 遥测与诊断特征从共享快照和统一事件环取值。
 */
// 仅BleTask调用。
// Initialize()后持续消费NimBLE原始报文队列。
esp_err_t Initialize(QueueHandle_t telemetry_queue, ErrorInfo *error=nullptr);
void Run(QueueHandle_t command_queue);
} // namespace ble
} // namespace vehicle
