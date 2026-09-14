#pragma once

#include <cstdint>

#include "error_config.hpp"

namespace vehicle {
namespace diagnostics {
namespace sink {
/*
 * sink是诊断的串口输出端：周期事件行、首次完整周期状态行与故障现场块。
 * 由低频任务（BleTask）驱动，不在控制路径格式化日志。
 */
// ReportSerial()按串口报告周期输出下一条事件；首次见到完整周期时输出一次启动状态行。
void ReportSerial(std::int64_t now_us);
// PrintControlFault()在输出禁能后格式化故障现场。
void PrintControlFault(const ErrorInfo &error);
} // namespace sink
} // namespace diagnostics
} // namespace vehicle
