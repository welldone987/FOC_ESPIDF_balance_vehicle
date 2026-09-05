/*
 * ble_idf_service集中管理ESP-IDF Bluetooth Controller、Bluedroid、GAP和GATTS生命周期。
 * 模块通过调用方提供的最新值Queue发布RemoteCommand，不创建项目自有BLE任务。
 */
#pragma once

#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <stdint.h>

namespace vehicle {

// BleServiceState描述原生BLE命令服务的异步生命周期。
enum class BleServiceState : uint8_t {
  kStopped,
  kStarting,
  kAdvertising,
  kConnected,
  kFault,
};

// beginBleIdfCommandService接收command_queue并提交原生Bluedroid服务初始化请求。
esp_err_t beginBleIdfCommandService(QueueHandle_t command_queue);
// bleIdfCommandServiceState返回可跨执行上下文读取的BLE服务状态。
BleServiceState bleIdfCommandServiceState();

} // namespace vehicle
