#include "diagnostics.hpp"

#include "esp_log.h"

namespace vehicle {
namespace diagnostics {
namespace {

// kTag把初始化和任务栈诊断日志归入同一ESP-IDF日志标签。
constexpr char kTag[] = "diagnostics";

} // namespace

void logInitialization(const char *name, esp_err_t result)
{
    // ESP_OK表示阶段完成，其余错误码保留为可读名称。
    if (result == ESP_OK) {
        ESP_LOGI(kTag, "%s initialized", name);
        return;
    }
    ESP_LOGE(
        kTag, "%s initialization failed: %s", name, esp_err_to_name(result));
}

void logTaskStack(const char *task_name, TaskHandle_t task_handle)
{
    // uxTaskGetStackHighWaterMark()返回任务运行以来观察到的最小剩余栈水位。
    ESP_LOGI(
        kTag,
        "%s stack_free=%u bytes",
        task_name,
        static_cast<unsigned int>(uxTaskGetStackHighWaterMark(task_handle)));
}

} // namespace diagnostics
} // namespace vehicle
