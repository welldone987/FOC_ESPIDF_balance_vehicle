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
/*
 * freertos_tasks保存任务共享的队列句柄与启动结果类型，声明各任务入口与静态创建函数。
 * 三级长度1静态队列分别传递原始报文、运动命令和遥测快照。
 * ControlTask独占电机与传感器；服务任务只消费队列。
 */
// BleStartup把BLE初始化结果和错误交给app_main。
struct BleStartup { esp_err_t result{}; ErrorInfo error{}; };
// TaskContext保存三个静态队列句柄，由app_main填充。
struct TaskContext {
    // command_queue接收BleTask解析后的MotionCommand。
    QueueHandle_t command_queue{};
    // ble_startup_queue向app_main报告BLE初始化结果。
    QueueHandle_t ble_startup_queue{};
    // telemetry_queue保存最近一帧TelemetrySnapshot。
    QueueHandle_t telemetry_queue{};
};
// CreateBleTask()创建静态BleTask并返回句柄；失败返回nullptr。
TaskHandle_t CreateBleTask(TaskContext &context);
void BleTask(void *argument);
// CreateControlTask()创建静态ControlTask并返回句柄；失败返回nullptr。
TaskHandle_t CreateControlTask(TaskContext &context);
void ControlTask(void *argument);
#if CONFIG_VEHICLE_WIFI_ENABLED
// CreateWifiTelemetryTask()创建静态WifiTelemetryTask并返回句柄；失败返回nullptr。
TaskHandle_t CreateWifiTelemetryTask(TaskContext &context);
void WifiTelemetryTask(void *argument);
#endif
} // namespace freertos_tasks
} // namespace vehicle
