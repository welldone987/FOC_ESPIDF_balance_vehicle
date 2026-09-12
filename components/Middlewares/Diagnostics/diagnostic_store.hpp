#pragma once

#include <cstddef>

#include "control_timing.hpp"
#include "diagnostics_config.hpp"
#include "error_info.hpp"

namespace vehicle {
namespace diagnostics {

enum class BootStep : std::uint16_t {
    safe_output=1, storage, power, voltage, nvs, core_dump, ble,
    wifi, imu, motor, outputs_off, timer, complete
};

struct ControlSnapshot {
    std::int64_t sampled_us{};
    std::uint32_t sequence{};
    float pitch_deg{}, velocity_M0{}, velocity_M1{}, target_M0{}, target_M1{};
    float iq_M0{}, iq_M1{}, dt_s{};
    bool valid{};
};

struct Event { std::uint32_t event_seq{}; ErrorInfo error{}; std::uint8_t flags{}; };

struct CrashState {
    // schema标识RAM诊断布局版本，离线解码必须使用匹配ELF。
    std::uint32_t schema{4};
    BootStep boot_step{};
    std::uint32_t event_seq{};
    Event first_fault{};
    Event events[EventCapacity]{};
    std::uint8_t count{}, next{};
    bool boot_complete{};
    ControlSnapshot last_control{}, fault_control{};
    control::ControlTiming last_timing{}, fault_timing{}, first_loop{}, first_balance{};
};

inline void Commit(CrashState &state, const ErrorInfo &error, bool fatal)
{
    Event event{++state.event_seq, error, static_cast<std::uint8_t>(fatal ? 1 : 0)};
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
