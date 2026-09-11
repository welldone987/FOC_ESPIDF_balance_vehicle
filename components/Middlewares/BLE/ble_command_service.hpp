#pragma once
#include "error_info.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
namespace vehicle {
namespace ble {
// 仅BleTask调用；初始化后持续消费NimBLE原始报文队列。
esp_err_t initialize(ErrorInfo *error=nullptr);
void run(QueueHandle_t command_queue);
} // namespace ble
} // namespace vehicle
