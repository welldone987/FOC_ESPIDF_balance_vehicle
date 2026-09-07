#pragma once

#include <cstdint>

#include "esp_err.h"

namespace vehicle {
namespace ble {

/*
 * BLE服务把网页端的“转向,油门”写入转换为控制任务读取的命令快照。
 * NimBLE回调发布原始整数，latestCommand()在控制任务侧完成缩放和一致性读取。
 */
struct CommandSnapshot {
    // steering_voltage_v保存缩放后的转向差分电压，单位V。
    float steering_voltage_v;
    // throttle_velocity_rad_s保存缩放后的目标轮速，单位rad/s。
    float throttle_velocity_rad_s;
    // sequence在完整非空命令写入时递增，用于识别新命令。
    std::uint32_t sequence;
    // connected表示当前是否存在BLE连接。
    bool connected;
};

// initialize()启动兼容原网页协议的ESP-IDF NimBLE外设。
esp_err_t initialize();

// latestCommand()返回控制任务可一致读取的最近一份命令快照。
CommandSnapshot latestCommand();

// isInitialized()报告BLE服务是否已完成初始化。
bool isInitialized();

} // namespace ble
} // namespace vehicle
