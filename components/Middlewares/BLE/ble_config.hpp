#pragma once

#include <cstdint>

namespace vehicle {
namespace ble {

// BLE名称、UUID和输入缩放保持网页控制端的整数协议不变。
inline constexpr char DeviceName[] = "平衡车";
inline constexpr char ServiceUuid[] = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
inline constexpr char CommandUuid[] = "6e400002-b5a3-f393-e0a9-e50e24dcca9e";
inline constexpr char TelemetryUuid[] = "6e400007-b5a3-f393-e0a9-e50e24dcca9e";

// ReadyTimeout_us是NimBLE主机就绪的内部等待上限；StartupWait_ms是app_main等待BleTask报告的上限。
inline constexpr std::int64_t ReadyTimeout_us = 5000000;
inline constexpr std::uint32_t StartupWait_ms = 6000U;
// NotifyPeriod_ms是.007遥测通知周期；NotifyErrorThrottle_us限制notify失败日志节流。
inline constexpr std::uint32_t NotifyPeriod_ms = 100U;
inline constexpr std::int64_t NotifyErrorThrottle_us = 1000000;
// TelemetryMaxAge_us是遥测快照可用于.007的新鲜度上限。
inline constexpr std::int64_t TelemetryMaxAge_us = 500000;

} // namespace ble
} // namespace vehicle
