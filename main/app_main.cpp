#include "application_tasks.hpp"
#include "ble_command_service.hpp"
#include "diagnostics.hpp"
#include "power_monitor.hpp"
#include "vehicle_config.hpp"
#include "wifi_telemtry.hpp"

#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

/*
 * app_main完成启动前置检查，建立静态队列和任务资源，再把实时工作交给三个FreeRTOS任务。
 * 初始化顺序为电源与欠压检查、BLE服务、诊断任务、控制任务和Wi-Fi遥测任务。
 */

namespace {

// telemetry_queue_storage和telemetry_queue_buffer为单元素最新值队列提供静态存储。
StaticQueue_t telemetry_queue_storage{};
std::uint8_t telemetry_queue_buffer[sizeof(vehicle::wifi_telemtry::TelemetrySnapshot)]{};

// 三组静态TCB和栈存储分别绑定ControlTask、WifiTelemetryTask和DiagnosticsTask。
StaticTask_t control_task_storage{};
StackType_t control_task_stack[vehicle::freertos_tasks::kControlStackBytes]{};
StaticTask_t wifi_task_storage{};
StackType_t wifi_task_stack[vehicle::freertos_tasks::kWifiStackBytes]{};
StaticTask_t diagnostics_task_storage{};
StackType_t diagnostics_task_stack[vehicle::freertos_tasks::kDiagnosticsStackBytes]{};

// task_context由所有任务共享，保存最新遥测队列和已创建任务句柄。
vehicle::freertos_tasks::TaskContext task_context{};

// initializeApplication()完成电源、启动母线电压和BLE初始化检查。
bool initializeApplication()
{
    esp_err_t result = vehicle::power::initialize();
    vehicle::diagnostics::logInitialization("Power", result);
    if (result != ESP_OK) {
        return false;
    }

    // bus_voltage_v保存启动时恢复后的直流母线电压，单位V。
    float bus_voltage_v = 0.0f;
    result = vehicle::power::readBusVoltage(&bus_voltage_v);
    if (result == ESP_OK &&
        bus_voltage_v <= vehicle::config::kStartupUndervoltageThresholdV) {
        result = ESP_ERR_INVALID_STATE;
    }
    vehicle::diagnostics::logInitialization("Startup voltage", result);
    if (result != ESP_OK) {
        return false;
    }

    result = vehicle::ble::initialize();
    vehicle::diagnostics::logInitialization("BLE", result);
    return result == ESP_OK;
}

// createTask()使用调用方提供的静态TCB和栈创建固定核心任务。
TaskHandle_t createTask(TaskFunction_t entry,
                        const char *name,
                        std::uint32_t stack_bytes,
                        UBaseType_t priority,
                        StackType_t *stack,
                        StaticTask_t *storage,
                        BaseType_t core)
{
    // 所有任务共享task_context作为入口参数，任务自身不需要动态分配上下文。
    const TaskHandle_t handle = xTaskCreateStaticPinnedToCore(
        entry,
        name,
        stack_bytes,
        &task_context,
        priority,
        stack,
        storage,
        core);
    vehicle::diagnostics::logInitialization(
        name, handle == nullptr ? ESP_ERR_NO_MEM : ESP_OK);
    return handle;
}

} // namespace

extern "C" void app_main(void)
{
    // 队列长度为1，控制任务只覆盖最新快照，Wi-Fi任务按需读取。
    task_context.telemetry_queue = xQueueCreateStatic(
        1U,
        sizeof(vehicle::wifi_telemtry::TelemetrySnapshot),
        telemetry_queue_buffer,
        &telemetry_queue_storage);
    vehicle::diagnostics::logInitialization(
        "Telemetry queue",
        task_context.telemetry_queue == nullptr ? ESP_ERR_NO_MEM : ESP_OK);
    // 资源或启动检查失败时删除app_main任务，不创建电机控制链路。
    if (task_context.telemetry_queue == nullptr || !initializeApplication()) {
        vTaskDelete(nullptr);
    }

    // 诊断任务运行在服务核心，先于控制任务创建以便记录后续初始化结果。
    const TaskHandle_t diagnostics = createTask(
        vehicle::freertos_tasks::diagnosticsTask,
        "DiagnosticsTask",
        vehicle::freertos_tasks::kDiagnosticsStackBytes,
        vehicle::freertos_tasks::kDiagnosticsPriority,
        diagnostics_task_stack,
        &diagnostics_task_storage,
        vehicle::freertos_tasks::kServiceCore);
    if (diagnostics == nullptr) {
        vTaskDelete(nullptr);
    }

    // 控制任务固定在Core 1运行，承担高频传感器、控制器和FOC调用链。
    const TaskHandle_t control = createTask(
        vehicle::freertos_tasks::controlTask,
        "ControlTask",
        vehicle::freertos_tasks::kControlStackBytes,
        vehicle::freertos_tasks::kControlPriority,
        control_task_stack,
        &control_task_storage,
        vehicle::freertos_tasks::kControlCore);
    if (control == nullptr) {
        vTaskDelete(nullptr);
    }
    // 发布控制任务句柄后，DiagnosticsTask才能读取其栈水位。
    task_context.control_handle.store(control, std::memory_order_release);

    // Wi-Fi任务固定在服务核心，避免网络服务进入控制任务的执行路径。
    const TaskHandle_t wifi = createTask(
        vehicle::freertos_tasks::wifiTelemetryTask,
        "WifiTelemetryTask",
        vehicle::freertos_tasks::kWifiStackBytes,
        vehicle::freertos_tasks::kWifiPriority,
        wifi_task_stack,
        &wifi_task_storage,
        vehicle::freertos_tasks::kServiceCore);
    if (wifi == nullptr) {
        vTaskDelete(nullptr);
    }
    // 发布Wi-Fi任务句柄供低频诊断查询栈水位。
    task_context.wifi_handle.store(wifi, std::memory_order_release);
    // app_main只负责一次性创建资源，完成后释放自身任务槽位。
    vTaskDelete(nullptr);
}
