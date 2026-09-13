#pragma once
#include "error_info.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
namespace vehicle {
namespace ble {
/*
 * ble_command_service是BleTask的公开入口：创建原始报文队列、
 * 编排传输/遥测/诊断三个服务，并在Run()中解析X,Y命令。
 * GATT回调只复制原始报文，量纲转换由BleTask执行。
 */
// 由BleTask调用。
// Initialize()创建共享NimBLE原始报文队列。
esp_err_t Initialize(QueueHandle_t telemetry_queue, ErrorInfo *error=nullptr);
void Run(QueueHandle_t command_queue);
} // namespace ble
} // namespace vehicle
