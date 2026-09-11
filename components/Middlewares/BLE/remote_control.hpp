#pragma once
#include "remote_protocol.hpp"
namespace vehicle::ble {
// 纯协议状态：接收端只提交请求，stepRemote由控制任务执行授权和停止。
struct RemoteState {
    RemoteCommand command{};
    std::int64_t received_us{};
    std::uint16_t applied_sequence{};
    RemoteMode mode{RemoteMode::boot};
    RemoteMode stop_reason{RemoteMode::stopped};
    bool connected{};
    bool has_command{};
    bool has_applied{};
    bool version2{};
    bool pending{};
    bool stop_pending{};
    bool emergency{};
    bool fault{};
    bool boot_complete{};
    bool run_allowed{};
};

constexpr void remoteConnection(RemoteState &state, bool connected)
{
    const bool emergency = state.emergency;
    const bool fault=state.fault, boot=state.boot_complete, allowed=state.run_allowed;
    state = RemoteState{};
    state.connected = connected;
    state.emergency = emergency;
    state.fault=fault; state.boot_complete=boot; state.run_allowed=allowed;
    state.stop_pending = true;
    state.stop_reason = connected ? RemoteMode::idle : RemoteMode::disconnected;
}

constexpr bool acceptCommand(RemoteState &state, RemoteCommand command,
                             std::int64_t now_us)
{
    if (!state.connected || command.legacy) { return false; }
    // 急停幂等锁存，重复或旧序号也不能使其失效；不刷新驾驶有效时间。
    if (command.kind == 'E') {
        state.version2 = true;
        state.emergency = true;
        return true;
    }
    if (state.emergency || state.fault) { return false; }
    if (command.kind == 'A' && (!state.boot_complete || !state.run_allowed)) { return false; }
    if (!command.legacy && state.version2 && state.has_command &&
        !newerSequence(command.sequence, state.command.sequence)) { return false; }
    const bool expired = state.has_command &&
        now_us - state.received_us >= kCommandTimeoutUs;
    if (command.kind == 'A' &&
        (!state.has_command || expired || state.stop_pending ||
         state.command.steering != 0 || state.command.throttle != 0)) {
        return false;
    }
    // 即使控制任务尚未观察超时，新到达的D也不能覆盖失联事件。
    if (expired && state.mode == RemoteMode::active) {
        state.stop_pending = true;
        state.stop_reason = RemoteMode::timeout;
    }
    if (!command.legacy && !state.version2) {
        state.stop_pending = true;
        state.stop_reason = RemoteMode::idle;
    }
    if (command.kind == 'S') {
        state.stop_pending = true;
        state.stop_reason = RemoteMode::stopped;
    }
    if (command.legacy) {
        command.sequence = static_cast<std::uint16_t>(state.command.sequence + 1U);
    } else {
        state.version2 = true;
    }
    state.command = command;
    state.received_us = now_us;
    state.has_command = true;
    state.pending = true;
    return true;
}

constexpr void stepRemote(RemoteState &state, std::int64_t now_us)
{
    if (state.fault) { state.mode=RemoteMode::fault; return; }
    if (state.emergency) {
        state.mode = RemoteMode::emergency;
        return;
    }
    if (state.stop_pending) {
        state.mode = state.stop_reason;
        state.stop_pending = false;
        // S可能被后续D覆盖，因此不宣称那个D已执行。
        if (state.command.kind == 'S') {
            state.applied_sequence = state.command.sequence;
            state.has_applied = true;
        }
        state.pending = false;
        return;
    }
    if (!state.connected) { state.mode = RemoteMode::disconnected; return; }
    if (state.has_command && now_us - state.received_us >= kCommandTimeoutUs) {
        if (state.mode == RemoteMode::active) { state.mode = RemoteMode::timeout; }
        state.pending = false;
        return;
    }
    if (!state.pending) { return; }
    state.pending = false;
    if (state.command.kind == 'A' && state.boot_complete && state.run_allowed) {
        state.mode = RemoteMode::active;
    }
    if (state.mode == RemoteMode::active) {
        state.applied_sequence = state.command.sequence;
        state.has_applied = true;
    }
}

} // namespace vehicle::ble
