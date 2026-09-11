#pragma once
#include <cstdint>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
namespace vehicle::freertos_tasks {
inline constexpr BaseType_t kControlCore=1, kServiceCore=0;
inline constexpr UBaseType_t kControlPriority=20, kWifiPriority=4;
inline constexpr std::uint32_t kControlStackBytes=8192, kWifiStackBytes=8192;
struct TaskContext {
#if CONFIG_VEHICLE_WIFI_ENABLED
    QueueHandle_t telemetry_queue{};
#endif
};
void controlTask(void *argument);
#if CONFIG_VEHICLE_WIFI_ENABLED
void wifiTelemetryTask(void *argument);
#endif
}
