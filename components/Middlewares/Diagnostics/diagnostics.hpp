#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace vehicle {
namespace diagnostics {

/*
 * 诊断模块把初始化结果和任务栈水位统一写入ESP-IDF日志。
 * 调用方负责提供阶段名称与任务句柄，模块不保存控制状态。
 */
// logInitialization()记录一个初始化阶段的成功或失败结果。
void logInitialization(const char *name, esp_err_t result);
// logTaskStack()记录任务的FreeRTOS剩余栈水位值。
void logTaskStack(const char *task_name, TaskHandle_t task_handle);

} // namespace diagnostics
} // namespace vehicle
