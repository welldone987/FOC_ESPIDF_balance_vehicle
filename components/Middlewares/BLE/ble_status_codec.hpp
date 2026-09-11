#pragma once
#include "ble_command_service.hpp"
#include <algorithm>
#include <bit>
namespace vehicle::ble {
// 手动编码小端字段，不把C++结构体内存布局当作无线协议。
inline void put16(std::uint8_t *packet, std::size_t offset, std::uint16_t value)
{
    packet[offset] = static_cast<std::uint8_t>(value);
    packet[offset + 1] = static_cast<std::uint8_t>(value >> 8);
}

// 按IEEE-754位模式检查，避免fast-math下isfinite被消除及NaN转整数。
constexpr bool finiteTelemetry(float value)
{
    return (std::bit_cast<std::uint32_t>(value) & 0x7f800000U) != 0x7f800000U;
}
static_assert(!finiteTelemetry(std::bit_cast<float>(0x7fc00000U)));
static_assert(!finiteTelemetry(std::bit_cast<float>(0x7f800000U)));
static_assert(finiteTelemetry(-12.34f));

inline std::int16_t scaled16(float value, float scale)
{
    return static_cast<std::int16_t>(std::clamp(value * scale, -32768.0f, 32767.0f));
}

inline void encodeStatusPacket(std::uint8_t (&packet)[20], const StatusSnapshot &status,
    const RemoteState &remote, std::uint32_t epoch, std::int64_t now_us)
{
    const bool same_session = status.command.connection_epoch == epoch;
    const bool stale = status.sampled_us == 0 || now_us - status.sampled_us >= kCommandTimeoutUs;
    const bool valid = status.sensors_valid && !stale && same_session &&
        finiteTelemetry(status.pitch_deg) && finiteTelemetry(status.left_velocity_rad_s) &&
        finiteTelemetry(status.right_velocity_rad_s) &&
        finiteTelemetry(status.command.throttle_velocity_rad_s) &&
        finiteTelemetry(status.command.steering_voltage_v);
    RemoteMode mode = status.command.mode;
    if (mode != RemoteMode::fault && mode != RemoteMode::emergency && !same_session) {
        mode = remote.connected ? RemoteMode::idle : RemoteMode::disconnected;
    }
    packet[0] = 2;
    packet[1] = static_cast<std::uint8_t>(mode);
    packet[2] = (remote.connected ? 1U : 0U) | (remote.command.legacy ? 2U : 0U) |
                (valid ? 4U : 0U) | (same_session && status.command.sequence_valid ? 8U : 0U) |
                (stale ? 16U : 0U);
    packet[3] = status.fault;
    put16(packet, 4, same_session ? status.command.sequence : 0);
    const auto age_ms = remote.has_command ? (now_us - remote.received_us) / 1000 : 65535;
    put16(packet, 6, static_cast<std::uint16_t>(std::clamp<std::int64_t>(age_ms, 0, 65535)));
    put16(packet, 8, valid ? static_cast<std::uint16_t>(scaled16(status.pitch_deg, 100.0f)) : 0);
    put16(packet, 10, valid ? static_cast<std::uint16_t>(scaled16(status.left_velocity_rad_s, 100.0f)) : 0);
    put16(packet, 12, valid ? static_cast<std::uint16_t>(scaled16(status.right_velocity_rad_s, 100.0f)) : 0);
    put16(packet, 14, valid ? static_cast<std::uint16_t>(scaled16(status.command.throttle_velocity_rad_s, 100.0f)) : 0);
    put16(packet, 16, valid ? static_cast<std::uint16_t>(scaled16(status.command.steering_voltage_v, 1000.0f)) : 0);
    put16(packet, 18, static_cast<std::uint16_t>(status.sample_sequence));
}

}
