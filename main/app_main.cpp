#include "application_tasks.hpp"
#include "ble_command_service.hpp"
#include "power_monitor.hpp"
#include "vehicle_config.hpp"

#include <cstdint>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

namespace {

constexpr char kTag[] = "balance_app";

// 诊断队列的控制块和存储区具有静态生命周期，直到应用结束都由app_main及任务共享。
StaticQueue_t diagnostics_queue_storage{};
// diagnostics_queue_buffer为16个DiagnosticEvent提供无动态分配的队列内存。
std::uint8_t diagnostics_queue_buffer[
    vehicle::freertos_tasks::kDiagnosticsQueueLength *
    sizeof(vehicle::freertos_tasks::DiagnosticEvent)]{};

// control_task_storage和control_task_stack在ControlTask存续期间提供静态TCB和栈。
StaticTask_t control_task_storage{};
StackType_t control_task_stack[
    vehicle::freertos_tasks::kControlStackSizeBytes]{};

// diagnostics_task_storage和diagnostics_task_stack在DiagnosticsTask存续期间提供静态TCB和栈。
StaticTask_t diagnostics_task_storage{};
StackType_t diagnostics_task_stack[
    vehicle::freertos_tasks::kDiagnosticsStackSizeBytes]{};

// task_runtime具有静态生命周期，队列句柄和原子计数器在任务退出前保持有效。
vehicle::freertos_tasks::TaskRuntime task_runtime{};

// initializeApplication完成母线电压检查和BLE初始化后再允许创建控制任务。
bool initializeApplication()
{
    using vehicle::freertos_tasks::DiagnosticCode;
    using vehicle::freertos_tasks::postDiagnosticEvent;

    postDiagnosticEvent(task_runtime, DiagnosticCode::ApplicationStarting);

    esp_err_t result = vehicle::power::initialize();
    if (result != ESP_OK) {
        postDiagnosticEvent(
            task_runtime, DiagnosticCode::PowerInitializationFailed, result);
        return false;
    }

    // bus_voltage_v保存上电检查读取的直流母线电压，单位V。
    float bus_voltage_v = 0.0f;
    result = vehicle::power::readBusVoltage(&bus_voltage_v);
    if (result != ESP_OK) {
        postDiagnosticEvent(task_runtime, DiagnosticCode::PowerReadFailed, result);
        return false;
    }
    if (bus_voltage_v <= vehicle::config::kStartupUndervoltageThresholdV) {
        postDiagnosticEvent(
            task_runtime,
            DiagnosticCode::StartupUndervoltage,
            ESP_OK,
            bus_voltage_v);
        return false;
    }
    postDiagnosticEvent(
        task_runtime, DiagnosticCode::PowerReady, ESP_OK, bus_voltage_v);

    result = vehicle::ble::initialize();
    if (result != ESP_OK) {
        postDiagnosticEvent(
            task_runtime, DiagnosticCode::BleInitializationFailed, result);
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(100U));
    return true;
}

} // namespace

// app_main创建静态诊断通道和两个固定核任务，并在启动任务完成后退出自身。
extern "C" void app_main(void)
{
    // xQueueCreateStatic()让诊断事件使用预分配存储，避免启动路径动态申请队列内存。
    task_runtime.diagnostics_queue = xQueueCreateStatic(
        vehicle::freertos_tasks::kDiagnosticsQueueLength,
        sizeof(vehicle::freertos_tasks::DiagnosticEvent),
        diagnostics_queue_buffer,
        &diagnostics_queue_storage);
    if (task_runtime.diagnostics_queue == nullptr) {
        ESP_LOGE(kTag, "Diagnostics queue creation failed");
        return;
    }

    // DiagnosticsTask固定在Core 0、低优先级运行，接收与任务运行时长相同的task_runtime。
    const TaskHandle_t diagnostics = xTaskCreateStaticPinnedToCore(
        vehicle::freertos_tasks::diagnosticsTask,
        "DiagnosticsTask",
        vehicle::freertos_tasks::kDiagnosticsStackSizeBytes,
        &task_runtime,
        vehicle::freertos_tasks::kDiagnosticsPriority,
        diagnostics_task_stack,
        &diagnostics_task_storage,
        vehicle::freertos_tasks::kDiagnosticsCore);
    if (diagnostics == nullptr) {
        ESP_LOGE(kTag, "DiagnosticsTask creation failed");
        return;
    }

    // 初始化失败时删除当前启动任务，保留DiagnosticsTask输出已发布的故障事件。
    if (!initializeApplication()) {
        vTaskDelete(nullptr);
    }

    // ControlTask固定在Core 1、较高优先级运行，独占高频传感器和电机控制调用链。
    const TaskHandle_t control = xTaskCreateStaticPinnedToCore(
        vehicle::freertos_tasks::controlTask,
        "ControlTask",
        vehicle::freertos_tasks::kControlStackSizeBytes,
        &task_runtime,
        vehicle::freertos_tasks::kControlPriority,
        control_task_stack,
        &control_task_storage,
        vehicle::freertos_tasks::kControlCore);
    if (control == nullptr) {
        vehicle::freertos_tasks::postDiagnosticEvent(
            task_runtime,
            vehicle::freertos_tasks::DiagnosticCode::ControlTaskCreationFailed);
        vTaskDelete(nullptr);
    }
    // 任务创建成功后发布句柄，DiagnosticsTask据此查询ControlTask栈余量。
    task_runtime.control_task_handle.store(control, std::memory_order_release);

    // app_main不再承担业务循环，任务创建完成后释放自身的动态任务槽位。
    vTaskDelete(nullptr);
}
