#pragma once

namespace vehicle {
namespace ble {
namespace config {

// BLE名称、UUID和输入缩放保持网页控制端的整数协议不变。
inline constexpr char kDeviceName[] = "平衡车";
inline constexpr char kServiceUuid[] = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
inline constexpr char kCommandUuid[] = "6e400006-b5a3-f393-e0a9-e50e24dcca9e";
inline constexpr float kMaximumThrottleVelocityRadS = 10.0f;

} // namespace config
} // namespace ble
} // namespace vehicle
