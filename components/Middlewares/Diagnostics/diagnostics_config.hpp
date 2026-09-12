#pragma once

#include <cstddef>
#include <cstdint>

namespace vehicle {
namespace diagnostics {

// EventCapacity是RAM事件环容量，保持16个槽位的取模更新。
inline constexpr unsigned EventCapacity = 16U;
// SerialReportPeriod_us是低频串口转储周期；ObservePeriod_ms是启动观察轮询周期。
inline constexpr std::int64_t SerialReportPeriod_us = 100000;
inline constexpr std::uint32_t ObservePeriod_ms = 100U;
// DiagnosticTextCapacity是单条诊断事件格式化文本的缓冲上限。
inline constexpr std::size_t DiagnosticTextCapacity = 512U;

} // namespace diagnostics
} // namespace vehicle
