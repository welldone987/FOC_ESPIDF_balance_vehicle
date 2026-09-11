#pragma once
#include <cstdint>
#include "sdkconfig.h"
#include "error_info.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
namespace vehicle {
namespace freertos_tasks {
inline constexpr BaseType_t kControlCore=1, kServiceCore=0;
inline constexpr UBaseType_t kControlPriority=20, kWifiPriority=4, kBlePriority=5;
inline constexpr std::uint32_t kControlStackBytes=8192, kWifiStackBytes=8192, kBleStackBytes=4096;
struct BleStartup { esp_err_t result{}; ErrorInfo error{}; };
struct TaskContext {
    QueueHandle_t command_queue{};
    QueueHandle_t ble_startup_queue{};
#if CONFIG_VEHICLE_WIFI_ENABLED
    QueueHandle_t telemetry_queue{};
#endif
};
void bleTask(void *argument);
void controlTask(void *argument);
#if CONFIG_VEHICLE_WIFI_ENABLED
void wifiTelemetryTask(void *argument);
#endif
} // namespace freertos_tasks
} // namespace vehicle
