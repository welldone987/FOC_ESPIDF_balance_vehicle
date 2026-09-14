#pragma once

#include <cstdint>

namespace vehicle {
namespace control {

/*
 * ControlSnapshot保存控制任务最近一次发布的定长现场，供诊断区存储与打印。
 * 与TelemetrySnapshot（协议视图）不同，本类型只服务诊断。
 */
struct ControlSnapshot {
    // sampled_us和sequence标识快照的采样时刻和序号。
    std::int64_t sampled_us{};
    std::uint32_t sequence{};
    // pitch_deg、两轮速度（rad/s）与目标电流（A）保存本周期观测值。
    float pitch_deg{}, velocity_M0_rad_s{}, velocity_M1_rad_s{}, target_M0_A{}, target_M1_A{};
    // iq_M0_A、iq_M1_A和dt_s保存实测电流与周期，单位A、s。
    float iq_M0_A{}, iq_M1_A{}, dt_s{};
    // valid标记快照是否来自完成的控制周期。
    bool valid{};
};

} // namespace control
} // namespace vehicle
