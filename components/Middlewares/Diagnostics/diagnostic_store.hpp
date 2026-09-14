#pragma once

#include <cstddef>

#include "control_timing.hpp"
#include "diagnostics_config.hpp"
#include "error_info.hpp"

namespace vehicle {
namespace diagnostics {

// BootStep按启动顺序标记当前初始化阶段。
// 保持既有编号不重排：原storage=2与随coredump删除的core_dump=6保留空缺，power=3、ble=7起的编号维持原值。
enum class BootStep : std::uint16_t {
    safe_output=1, power=3, voltage, nvs, ble=7,
    wifi, imu, motor, outputs_off, timer, complete
};

// ControlSnapshot保存控制任务最近一次发布的定长现场。
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

// Event保存一条ErrorInfo及其致命标志。
struct Event { std::uint32_t event_seq{}; ErrorInfo error{}; std::uint8_t flags{}; };

// CrashState是RAM诊断区的固定布局。
struct CrashState {
    // event_seq记录事件序号。
    std::uint32_t event_seq{};
    // first_fault保留独立首故障槽。
    Event first_fault{};
    // events以环形保存最近的EventCapacity条事件。
    Event events[EventCapacity]{};
    // count和next记录环内事件数与下一个写入槽。
    std::uint8_t count{}, next{};
    // last_control和fault_control保存最近与故障前控制快照。
    ControlSnapshot last_control{}, fault_control{};
    // last_timing、fault_timing及首轮/首次平衡计时用于时序诊断。
    control::ControlTiming last_timing{}, fault_timing{}, first_loop{}, first_balance{};
};

inline void Commit(CrashState &state, const ErrorInfo &error, bool fatal)
{
    Event event{++state.event_seq, error, static_cast<std::uint8_t>(fatal ? 1 : 0)};
    // 致命错误且无首故障时，同时保存首故障和现场快照。
    if (fatal && state.first_fault.event_seq == 0) {
        event.flags |= 2;
        state.first_fault = event;
        state.fault_control = state.last_control;
        state.fault_timing = state.last_timing;
    }
    state.events[state.next] = event;
    state.next = (state.next + 1) % EventCapacity;
    if (state.count < EventCapacity) { ++state.count; }
}

inline void CommitTiming(CrashState &state, const control::ControlTiming &timing)
{
    state.last_timing = timing;
    if (timing.stage != control::ControlStage::complete) { return; }
    if (state.first_loop.cycle == 0) { state.first_loop = timing; }
    if (timing.balancing && state.first_balance.cycle == 0) { state.first_balance = timing; }
}

inline bool NextEvent(const CrashState &state, std::uint32_t after, Event &out)
{
    // RAM历史按提交顺序遍历，首故障另有独立槽。
    for (unsigned index=0; index<state.count; ++index) {
        const auto &event = state.events[(state.next + EventCapacity - state.count + index) % EventCapacity];
        if (event.event_seq > after) { out = event; return true; }
    }
    return false;
}

} // namespace diagnostics
} // namespace vehicle
