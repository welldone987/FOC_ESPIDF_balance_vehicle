#pragma once

#include <cstdint>

#include "freertos/FreeRTOS.h"

namespace vehicle {
namespace freertos_tasks {

// 控制核心与服务核心的编号；栈单位按IDF为字节。
inline constexpr BaseType_t ControlCore = 1, ServiceCore = 0;
inline constexpr UBaseType_t ControlPriority = 20, WifiPriority = 4, BlePriority = 5;
inline constexpr std::uint32_t ControlStackBytes = 8192, WifiStackBytes = 8192, BleStackBytes = 4096;

} // namespace freertos_tasks
} // namespace vehicle
