#include "freertos_tasks.hpp"

#include "ble_command_service.hpp"

namespace vehicle {
namespace freertos_tasks {
namespace {

// ble_storage和ble_stack是BleTask的静态TCB与栈。
StaticTask_t ble_storage{};
StackType_t ble_stack[BleStackBytes]{};

} // namespace

// CreateBleTask()创建静态BleTask并返回句柄；失败返回nullptr。
TaskHandle_t CreateBleTask(TaskContext &context)
{
    return xTaskCreateStaticPinnedToCore(BleTask,"BleTask",BleStackBytes,
        &context,BlePriority,ble_stack,&ble_storage,ServiceCore);
}

// BleTask初始化BLE并把结果写入启动队列，随后进入报文消费循环。
void BleTask(void *argument)
{
    auto &context=*static_cast<TaskContext *>(argument);
    BleStartup startup{};
    startup.result=ble::Initialize(context.telemetry_queue,&startup.error);
    xQueueOverwrite(context.ble_startup_queue,&startup);
    if (startup.result == ESP_OK) { ble::Run(context.command_queue); }
    // 初始化失败不重试，也不放行控制。
    // 静态任务资源保留供诊断。
    for (;;) { vTaskDelay(pdMS_TO_TICKS(1000)); }
}

} // namespace freertos_tasks
} // namespace vehicle
