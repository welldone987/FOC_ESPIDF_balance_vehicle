#pragma once
#include <cstdint>
#include "sdkconfig.h"
#include "error_info.hpp"
#include "freertos_tasks_config.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
namespace vehicle {
namespace freertos_tasks {
struct BleStartup { esp_err_t result{}; ErrorInfo error{}; };
struct TaskContext {
    QueueHandle_t command_queue{};
    QueueHandle_t ble_startup_queue{};
    QueueHandle_t telemetry_queue{};
};
void BleTask(void *argument);
void ControlTask(void *argument);
#if CONFIG_VEHICLE_WIFI_ENABLED
void WifiTelemetryTask(void *argument);
#endif
} // namespace freertos_tasks
} // namespace vehicle
