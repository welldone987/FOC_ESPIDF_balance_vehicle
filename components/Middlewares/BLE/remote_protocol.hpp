#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>

namespace vehicle::ble {

inline constexpr std::int64_t kCommandTimeoutUs = 300000;
enum class RemoteMode : std::uint8_t {
    boot, idle, active, timeout, disconnected, emergency, fault, stopped
};

struct RemoteCommand {
    char kind{'D'};
    std::uint16_t sequence{};
    std::int16_t steering{};
    std::int16_t throttle{};
    bool legacy{};
};

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
};

constexpr bool readInteger(std::string_view text, std::size_t &offset,
                           int limit, int &value)
{
    bool negative = false;
    if (offset < text.size() && text[offset] == '-') {
        negative = true;
        ++offset;
    }
    const std::size_t begin = offset;
    int magnitude = 0;
    while (offset < text.size() && text[offset] >= '0' && text[offset] <= '9') {
        const int digit = text[offset++] - '0';
        if (magnitude > (limit - digit) / 10 || digit > limit) {
            return false;
        }
        magnitude = magnitude * 10 + digit;
        if (magnitude > limit) {
            return false;
        }
    }
    value = negative ? -magnitude : magnitude;
    return offset != begin;
}

constexpr bool comma(std::string_view text, std::size_t &offset)
{
    return offset < text.size() && text[offset++] == ',';
}

constexpr bool parseCommand(std::string_view text, bool legacy, RemoteCommand &out)
{
    RemoteCommand parsed{};
    parsed.legacy = legacy;
    std::size_t offset = 0;
    int steering = 0;
    int throttle = 0;
    if (legacy) {
        if (!text.empty() && text.back() == '\n') { text.remove_suffix(1); }
        if (!text.empty() && text.back() == '\r') { text.remove_suffix(1); }
        if (!readInteger(text, offset, 1500, steering) || !comma(text, offset) ||
            !readInteger(text, offset, 100, throttle)) {
            return false;
        }
    } else {
        if (text.size() < 3 || text.size() > 20) { return false; }
        parsed.kind = text[offset++];
        if (parsed.kind != 'D' && parsed.kind != 'A' &&
            parsed.kind != 'S' && parsed.kind != 'E') { return false; }
        int sequence = 0;
        if (!comma(text, offset) || offset == text.size() || text[offset] == '-' ||
            !readInteger(text, offset, 65535, sequence)) { return false; }
        parsed.sequence = static_cast<std::uint16_t>(sequence);
        if (parsed.kind == 'D' &&
            (!comma(text, offset) || !readInteger(text, offset, 100, steering) ||
             !comma(text, offset) || !readInteger(text, offset, 100, throttle))) {
            return false;
        }
    }
    if (offset != text.size()) { return false; }
    parsed.steering = static_cast<std::int16_t>(steering);
    parsed.throttle = static_cast<std::int16_t>(throttle);
    out = parsed;
    return true;
}

constexpr bool newerSequence(std::uint16_t next, std::uint16_t previous)
{
    const auto delta = static_cast<std::uint16_t>(next - previous);
    return delta != 0 && delta < 0x8000U;
}

constexpr void remoteConnection(RemoteState &state, bool connected)
{
    const bool emergency = state.emergency;
    state = RemoteState{};
    state.connected = connected;
    state.emergency = emergency;
    state.stop_pending = true;
    state.stop_reason = connected ? RemoteMode::idle : RemoteMode::disconnected;
}

constexpr bool acceptCommand(RemoteState &state, RemoteCommand command,
                             std::int64_t now_us)
{
    if (!state.connected || (command.legacy && state.version2)) { return false; }
    // 急停幂等锁存，重复或旧序号也不能使其失效；不刷新驾驶有效时间。
    if (command.kind == 'E') {
        state.version2 = true;
        state.emergency = true;
        return true;
    }
    if (state.emergency) { return false; }
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
    if (state.command.kind == 'A' || state.command.legacy) {
        state.mode = RemoteMode::active;
    }
    if (state.mode == RemoteMode::active) {
        state.applied_sequence = state.command.sequence;
        state.has_applied = true;
    }
}

} // namespace vehicle::ble
