#pragma once

#include <cstddef>
#include <cstdint>

namespace vehicle {
namespace wifi_telemetry {

// 重连、发送阻塞和文本缓冲区参数的单位分别为ms、ms和字节。
inline constexpr std::uint32_t ReconnectPeriod_ms = 1000U;
inline constexpr std::uint32_t ClientTimeout_ms = 1000U;
inline constexpr std::size_t BufferSize = 768U;
// ServicePeriod_ms是遥测服务任务周期。
inline constexpr std::uint32_t ServicePeriod_ms = 100U;

} // namespace wifi_telemetry
} // namespace vehicle
