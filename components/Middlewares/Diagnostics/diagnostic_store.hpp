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
    std::uint32_t schema{3}; // RAM布局版本；BLE schema保持1。
    BootStep boot_step{};
    std::uint32_t event_seq{};
    Event first_fault{};
    Event events[16]{};
    std::uint8_t count{}, next{};
    ErrorInfo last_ble_error{};
    std::uint16_t mtu{23};
    bool connected{}, status_subscribed{}, diag_subscribed{}, boot_complete{};
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
    // First fault is replayed separately by the transport, history stays ordered.
    for (unsigned n=0; n<s.count; ++n) {
        const auto &event = s.events[(s.next + 16 - s.count + n) % 16];
        if (event.event_seq > after) { out = event; return true; }
    }
    return false;
}
struct ReplayCursor { std::uint32_t after{}; bool first_sent{}; };
inline bool prepareReplay(const CrashState &s, const ReplayCursor &cursor, Event &out, bool &first)
{
    first=!cursor.first_sent && s.first_fault.event_seq != 0;
    if (first) { out=s.first_fault; return true; }
    return nextEvent(s,cursor.after,out);
}
inline void acceptReplay(ReplayCursor &cursor, const Event &event, bool first, bool submitted)
{
    if (!submitted) { return; }
    if (first) { cursor.first_sent=true; } else { cursor.after=event.event_seq; }
}
inline void putLe(std::uint8_t *p, std::uint32_t v, unsigned count)
{ for (unsigned i=0; i<count; ++i) { p[i] = static_cast<std::uint8_t>(v >> (8*i)); } }
inline void encodeEvent(const Event &e, std::uint8_t (&p)[14])
{
    p[0]=1; putLe(p+1, static_cast<std::uint16_t>(e.error.point_id), 2);
    putLe(p+3,e.event_seq,4); p[7]=static_cast<std::uint8_t>(e.error.domain);
    p[8]=e.flags; putLe(p+9,static_cast<std::uint32_t>(e.error.raw_code),4);
    p[13]=static_cast<std::uint8_t>(e.error.channel);
}
inline void encodeMetadata(const CrashState &s, std::uint8_t (&p)[20])
{
    p[0]=1; p[1]=(s.boot_complete ? 1:0) | (s.first_fault.event_seq ? 2:0);
    putLe(p+2,static_cast<std::uint16_t>(s.boot_step),2); putLe(p+4,s.event_seq,4);
    putLe(p+8,s.first_fault.event_seq,4); putLe(p+12,static_cast<std::uint16_t>(s.first_fault.error.point_id),2);
    p[14]=s.count; p[15]=16; putLe(p+16,s.mtu,2); p[18]=s.connected; p[19]=s.diag_subscribed;
}
}
