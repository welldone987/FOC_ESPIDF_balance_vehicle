#pragma once
#include <cstdint>
#include "control_config.hpp"
namespace vehicle {
namespace control {
/*
 * BleTask解析X,Y百分比并调用MakeMotionCommand()换算为车辆坐标目标，
 * 写入长度1队列；ControlTask非阻塞读取最新目标并独立检查接收时刻时效。
 * MotionCommand保留原始接收时间，排队不刷新寿命。
 */
// MotionCommand保存车辆坐标下的最新遥控目标。
struct MotionCommand {
    // velocity_rad_s保存目标轮速，单位rad/s。
    float velocity_rad_s{};
    // yaw_rate_rad_s保存目标偏航角速度，单位rad/s。
    float yaw_rate_rad_s{};
    // received_us保存BLE接收该命令的时刻，单位us。
    std::int64_t received_us{};
    // valid标记命令内容是否已通过解析。
    bool valid{};
};
// IsCommandFresh()判断命令是否在ready_us之后接收且未超过CommandTimeout_us。
constexpr bool IsCommandFresh(const MotionCommand &command, std::int64_t now_us,
                              std::int64_t ready_us)
{
    return command.valid && command.received_us >= ready_us &&
        now_us >= command.received_us && now_us-command.received_us < CommandTimeout_us;
}
// MakeMotionCommand()把遥控百分比（±100）换算为车辆坐标目标，限值取自控制配置。
constexpr MotionCommand MakeMotionCommand(float steering_percent, float throttle_percent,
                                          std::int64_t received_us)
{
    return {throttle_percent * DriveSpeedLimit_rad_s / 100.0f,
        steering_percent * YawRateLimit_rad_s / 100.0f, received_us, true};
}

} // namespace control
} // namespace vehicle
