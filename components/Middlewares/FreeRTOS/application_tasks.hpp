#pragma once

#include <atomic>
#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

namespace vehicle {
namespace freertos_tasks {

/*
 * FreeRTOS任务模块把控制、Wi-Fi遥测和诊断放到独立任务中。
 * TaskContext通过最新值队列传递遥测快照，并以原子句柄供诊断任务读取栈余量。
 */
// kControlCore和kServiceCore固定高频控制任务与服务任务的运行核心。
inline constexpr BaseType_t kControlCore = 1;
inline constexpr BaseType_t kServiceCore = 0;
inline constexpr UBaseType_t kControlPriority = 20U;
inline constexpr UBaseType_t kWifiPriority = 4U;
inline constexpr UBaseType_t kDiagnosticsPriority = 3U;
inline constexpr std::uint32_t kControlStackBytes = 8192U;
inline constexpr std::uint32_t kWifiStackBytes = 8192U;
inline constexpr std::uint32_t kDiagnosticsStackBytes = 4096U;

struct TaskContext {
    // telemetry_queue只保留最新TelemetrySnapshot，避免遥测数据在控制任务中积压。
    QueueHandle_t telemetry_queue{nullptr};
    // control_handle保存ControlTask句柄，供诊断任务查询栈余量。
    std::atomic<TaskHandle_t> control_handle{nullptr};
    // wifi_handle保存WifiTelemetryTask句柄，供诊断任务查询栈余量。
    std::atomic<TaskHandle_t> wifi_handle{nullptr};
};

// controlTask执行传感器读取、控制计算、电机目标暂存和遥测快照发布。
void controlTask(void *argument);
// wifiTelemetryTask周期性消费最新遥测快照并驱动Wi-Fi TCP服务。
void wifiTelemetryTask(void *argument);
// diagnosticsTask周期性输出各任务的FreeRTOS栈余量。
void diagnosticsTask(void *argument);

} // namespace freertos_tasks
} // namespace vehicle
