#pragma once
#include "error_config.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
namespace vehicle {
namespace ble {
/*
 * ble_command_service是BleTask的公开入口：创建原始报文队列、
 * 编排传输/遥测/诊断三个服务，并逐轮解析X,Y命令。
 * GATT回调只复制原始报文；物理量换算由control::MakeMotionCommand完成。
 */
// 由BleTask调用。
// Initialize()创建共享NimBLE原始报文队列。
esp_err_t Initialize(QueueHandle_t telemetry_queue, ErrorInfo *error=nullptr);
// Run()处理一轮原始报文（最长等待100ms），由BleTask循环调用。
void Run(QueueHandle_t command_queue);
} // namespace ble
} // namespace vehicle
