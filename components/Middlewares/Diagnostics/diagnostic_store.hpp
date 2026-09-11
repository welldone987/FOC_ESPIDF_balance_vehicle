#pragma once
#include "error_info.hpp"
#include "control_timing.hpp"
#include <cstddef>
namespace vehicle::diagnostics {
enum class BootStep : std::uint16_t { safe_output=1, storage, power, voltage, nvs, core_dump, ble,
    wifi, imu, motor, outputs_off, timer, complete };
struct ControlSnapshot {
    std::int64_t sampled_us{};
    std::uint32_t sequence{};
    float pitch_deg{}, left_velocity{}, right_velocity{}, left_target{}, right_target{};
    float left_iq{}, right_iq{}, dt_s{};
    bool valid{};
};
struct Event { std::uint32_t event_seq{}; ErrorInfo error{}; std::uint8_t flags{}; };
struct CrashState {
    std::uint32_t schema{4}; // RAM布局版本，使用匹配ELF解码。
    BootStep boot_step{};
    std::uint32_t event_seq{};
    Event first_fault{};
    Event events[16]{};
    std::uint8_t count{}, next{};
    bool boot_complete{};
    ControlSnapshot last_control{}, fault_control{};
    ControlTiming last_timing{}, fault_timing{}, first_loop{}, first_balance{};
};
inline void commit(CrashState &s, const ErrorInfo &error, bool fatal)
{
    Event event{++s.event_seq, error, static_cast<std::uint8_t>(fatal ? 1 : 0)};
    if (fatal && s.first_fault.event_seq == 0) {
        event.flags |= 2; s.first_fault = event; s.fault_control = s.last_control;
        s.fault_timing = s.last_timing;
    }
    s.events[s.next] = event; s.next = (s.next + 1) % 16;
    if (s.count < 16) { ++s.count; }
}
inline void commitTiming(CrashState &s, const ControlTiming &timing)
{
    s.last_timing=timing;
    if (timing.stage != ControlStage::complete) { return; }
    if (s.first_loop.cycle == 0) { s.first_loop=timing; }
    if (timing.balancing && s.first_balance.cycle == 0) { s.first_balance=timing; }
}
inline bool nextEvent(const CrashState &s, std::uint32_t after, Event &out)
{
    // RAM历史按提交顺序遍历，首故障另有独立槽。
    for (unsigned n=0; n<s.count; ++n) {
        const auto &event = s.events[(s.next + 16 - s.count + n) % 16];
        if (event.event_seq > after) { out = event; return true; }
    }
    return false;
}
}
