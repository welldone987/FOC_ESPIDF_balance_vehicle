#pragma once
#include "diagnostic_store.hpp"
extern vehicle::diagnostics::CrashState g_diag_crash;
namespace vehicle {
namespace diagnostics {
// Initialize()清空RAM诊断区，供启动早期调用。
void Initialize();
// Boot()输出一行启动步骤日志。
void Boot(BootStep step, const char *state, esp_err_t rc=ESP_OK);
// BootSubstep()输出电机子步骤的ErrorPoint编号与状态（0x3xx）。
void BootSubstep(std::uint16_t point, const char *state);
// Record()在短临界区内提交一条诊断事件。
void Record(const ErrorInfo &error, bool fatal=false);
// ReadEvent()按序号读取下一个事件。
// first_fault为true时优先返回独立首故障。
bool ReadEvent(std::uint32_t after, Event &event, bool first_fault=false);
// 非控制任务调用：串口和BLE使用相同字段，缓冲由调用者提供。
int FormatEvent(const Event &event, char *buffer, std::size_t capacity);
// CommitControlSnapshot()保存最近一次控制快照。
void CommitControlSnapshot(const ControlSnapshot &snapshot);
// CommitControlTiming()保存计时并记录首轮与首次平衡。
void CommitControlTiming(const control::ControlTiming &timing);
// PrintControlFault()在输出禁能后格式化故障现场。
void PrintControlFault(const ErrorInfo &error);
// ObserveControlStart()复用app_main低频观察首帧和首次平衡成功。
// 不创建额外诊断任务。
void ObserveControlStart();
} // namespace diagnostics
} // namespace vehicle
