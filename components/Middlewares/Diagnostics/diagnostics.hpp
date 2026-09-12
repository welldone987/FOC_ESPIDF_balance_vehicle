#pragma once
#include "diagnostic_store.hpp"
extern vehicle::diagnostics::CrashState g_diag_crash;
namespace vehicle {
namespace diagnostics {
void initialize();
void boot(BootStep step, const char *state, esp_err_t rc=ESP_OK);
void record(const ErrorInfo &error, bool fatal=false);
bool readEvent(std::uint32_t after, Event &event, bool first_fault=false);
// 非控制任务调用：串口和BLE使用相同字段，缓冲由调用者提供。
int formatEvent(const Event &event, char *buffer, std::size_t capacity);
void controlSnapshot(const ControlSnapshot &snapshot);
void completeBoot();
void controlTiming(const control::ControlTiming &timing);
void printControlFault(const ErrorInfo &error);
// 复用app_main低频观察首帧和首次平衡成功；不创建额外诊断任务。
void observeControlStart();
} // namespace diagnostics
} // namespace vehicle
