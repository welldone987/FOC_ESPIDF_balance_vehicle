#pragma once

#include <cstdint>
#include "esp_err.h"
#include "remote_protocol.hpp"

namespace vehicle::ble {

struct CommandSnapshot {
    float steering_voltage_v;
    float throttle_velocity_rad_s;
    std::uint16_t sequence;
    bool connected;
    RemoteMode mode;
    std::int64_t command_age_us;
    bool legacy;
    bool sequence_valid;
    std::uint32_t connection_epoch;
};

struct StatusSnapshot {
    CommandSnapshot command{};
    std::int64_t sampled_us{};
    std::uint32_t sample_sequence{};
    float pitch_deg{};
    float left_velocity_rad_s{};
    float right_velocity_rad_s{};
    std::uint8_t fault{};
    bool sensors_valid{};
};

esp_err_t initialize();
// 仅由ControlTask调用：消费请求、判断超时并返回本周期有效命令。
CommandSnapshot latestCommand();
// 仅复制固定状态，通知由NimBLE主机发送。故障码：1初始化、2传感器、3定时器。
void publishStatus(const StatusSnapshot &status);
void publishFault(std::uint8_t fault, bool emergency = false);

} // namespace vehicle::ble
