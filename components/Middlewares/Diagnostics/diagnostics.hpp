#pragma once
#include "diagnostic_store.hpp"
extern vehicle::diagnostics::CrashState g_diag_crash;
namespace vehicle::diagnostics {
void initialize();
void boot(BootStep step, const char *state, esp_err_t rc=ESP_OK);
void record(const ErrorInfo &error, bool fatal=false);
void bleError(const ErrorInfo &error);
void bleState(bool connected, bool status, bool diag, std::uint16_t mtu);
void controlSnapshot(const ControlSnapshot &snapshot);
void completeBoot();
void controlTiming(const ControlTiming &timing);
void printControlFault(const ErrorInfo &error);
// 复用app_main低频观察首帧和首次平衡成功；不创建额外诊断任务。
void observeControlStart();
bool replay(const ReplayCursor &cursor, Event &event, bool &first);
void metadata(std::uint8_t (&packet)[20]);
}
