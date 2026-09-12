#pragma once
#include <cstdint>
#include "control_config.hpp"
namespace vehicle {
namespace control {
// BleTask生产，ControlTask消费；速度/偏航均采用车辆坐标，时间戳不因转发刷新。
struct MotionCommand {
    float velocity_rad_s{};
    float yaw_rate_rad_s{};
    std::int64_t received_us{};
    bool valid{};
};
constexpr bool IsCommandFresh(const MotionCommand &command, std::int64_t now_us,
                              std::int64_t ready_us)
{
    return command.valid && command.received_us >= ready_us &&
        now_us >= command.received_us && now_us-command.received_us < CommandTimeout_us;
}

} // namespace control
} // namespace vehicle
