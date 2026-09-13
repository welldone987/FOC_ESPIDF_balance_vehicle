#include "freertos_tasks.hpp"

#include "ble_config.hpp"
#include "motion_command.hpp"
#include "telemetry_snapshot.hpp"
#include "wifi_telemetry.hpp"

namespace vehicle {
namespace freertos_tasks {
namespace {

/*
 * freertos_tasks创建任务共享的长度1静态队列，并编排BLE与Wi-Fi的启动。
 * 队列缓冲在本文件静态分配，启动路径不引入动态内存。
 */
// 三组长度1静态队列的存储与缓冲。
StaticQueue_t command_storage{}, ble_startup_storage{}, telemetry_storage{};
std::uint8_t command_buffer[sizeof(control::MotionCommand)]{};
std::uint8_t ble_startup_buffer[sizeof(BleStartup)]{};
std::uint8_t telemetry_buffer[sizeof(control::TelemetrySnapshot)]{};

// CreateTaskQueues()创建三组静态队列并写入context；任一失败返回false。
bool CreateTaskQueues(TaskContext &context)
{
    context.command_queue=xQueueCreateStatic(TaskQueueLength,sizeof(control::MotionCommand),command_buffer,&command_storage);
    context.telemetry_queue=xQueueCreateStatic(TaskQueueLength,sizeof(control::TelemetrySnapshot),telemetry_buffer,&telemetry_storage);
    context.ble_startup_queue=xQueueCreateStatic(TaskQueueLength,sizeof(BleStartup),ble_startup_buffer,&ble_startup_storage);
    return context.command_queue && context.telemetry_queue && context.ble_startup_queue;
}

} // namespace

esp_err_t StartBle(TaskContext &context, ErrorInfo *error)
{
    if (!CreateTaskQueues(context) || !CreateBleTask(context)) {
        return VEHICLE_ERROR(error,ESP_ERR_NO_MEM,boot_resource,application,0);
    }
    // BleTask完成首次广播后才放行控制初始化。
    BleStartup startup{};
    if (xQueueReceive(context.ble_startup_queue,&startup,pdMS_TO_TICKS(ble::StartupWait_ms)) != pdTRUE) {
        return VEHICLE_ERROR(error,ESP_ERR_TIMEOUT,ble_ready_timeout,application,0);
    }
    if (startup.result != ESP_OK) {
        if (error) { *error=startup.error; }
        return startup.result;
    }
    return ESP_OK;
}

esp_err_t StartWifiTelemetry(TaskContext &context, ErrorInfo *error)
{
    const esp_err_t result=wifi_telemetry::Initialize();
    if (result == ESP_OK && !CreateWifiTelemetryTask(context)) {
        return VEHICLE_ERROR(error,ESP_ERR_NO_MEM,wifi_init,esp,ESP_ERR_NO_MEM);
    }
    if (result != ESP_OK) {
        return VEHICLE_ERROR(error,result,wifi_init,esp,result);
    }
    return ESP_OK;
}

} // namespace freertos_tasks
} // namespace vehicle
