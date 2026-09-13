#include "freertos_tasks.hpp"

#include "telemetry_snapshot.hpp"
#include "wifi_telemetry.hpp"
#include "wifi_telemetry_config.hpp"

namespace vehicle {
namespace freertos_tasks {
namespace {

// wifi_storage和wifi_stack是Wi-Fi服务任务的静态TCB与栈。
StaticTask_t wifi_storage{};
StackType_t wifi_stack[WifiStackBytes]{};

} // namespace

// CreateWifiTelemetryTask()创建静态WifiTelemetryTask并返回句柄；失败返回nullptr。
TaskHandle_t CreateWifiTelemetryTask(TaskContext &context)
{
    return xTaskCreateStaticPinnedToCore(WifiTelemetryTask,"WifiTelemetryTask",WifiStackBytes,
        &context,WifiPriority,wifi_stack,&wifi_storage,ServiceCore);
}

// WifiTelemetryTask按ServicePeriod_ms消费最新遥测快照。
void WifiTelemetryTask(void *argument)
{
    // Wi-Fi服务运行在低频服务任务中，与控制任务共享最新值队列。
    [[maybe_unused]] auto &context = *static_cast<TaskContext *>(argument);
    // release使用绝对唤醒时刻，减少服务周期随执行耗时漂移。
    TickType_t release = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&release, pdMS_TO_TICKS(wifi_telemetry::ServicePeriod_ms));
        // xQueuePeek()复制最新快照但不移除队列中的值。
        control::TelemetrySnapshot snapshot{};
        const control::TelemetrySnapshot *latest = nullptr;
        if (xQueuePeek(context.telemetry_queue, &snapshot, 0U) == pdPASS) {
            latest = &snapshot;
        }
        wifi_telemetry::Service(latest);
    }
}

} // namespace freertos_tasks
} // namespace vehicle
