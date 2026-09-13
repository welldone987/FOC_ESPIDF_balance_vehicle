#pragma once

#include <cstdint>

#include "freertos/FreeRTOS.h"

namespace vehicle {
namespace freertos_tasks {

// ControlCore和ServiceCore是控制任务与服务任务的核编号。
// 栈单位按IDF为字节。
inline constexpr BaseType_t ControlCore = 1, ServiceCore = 0;
// ControlPriority、WifiPriority和BlePriority是三个任务的静态优先级。
inline constexpr UBaseType_t ControlPriority = 20, WifiPriority = 4, BlePriority = 5;
inline constexpr std::uint32_t ControlStackBytes = 8192, WifiStackBytes = 8192, BleStackBytes = 4096;
// TaskQueueLength是三组任务间IPC队列的深度；长度1只保留最新值。
inline constexpr UBaseType_t TaskQueueLength = 1U;

} // namespace freertos_tasks
} // namespace vehicle
