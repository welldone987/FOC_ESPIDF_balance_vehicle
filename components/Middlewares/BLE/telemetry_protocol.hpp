#pragma once
#include <array>
#include <cstdint>
#include <cstring>
#include "ble_config.hpp"
#include "wifi_telemetry.hpp"

namespace vehicle::ble {
// 固定20字节、小端、IEEE-754；不直接发送具有填充字节的C++结构体。
using TelemetryPacket = std::array<std::uint8_t, 20>;
inline std::uint32_t FloatBits(float value)
{
    static_assert(sizeof(float) == sizeof(std::uint32_t));
    std::uint32_t bits{};
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}
inline void PutU32(TelemetryPacket &packet, unsigned offset, std::uint32_t value)
{
    for (unsigned i=0; i<4; ++i) { packet[offset+i]=static_cast<std::uint8_t>(value>>(8*i)); }
}
inline TelemetryPacket EncodeTelemetry(const wifi_telemetry::TelemetrySnapshot *sample,
                                       std::int64_t now_us)
{
    TelemetryPacket packet{};
    packet[0]=1;
    if (!sample) { return packet; }
    const auto pitch=FloatBits(sample->pitch_deg);
    const auto velocity_M0=FloatBits(sample->velocity_M0_rad_s);
    const auto velocity_M1=FloatBits(sample->velocity_M1_rad_s);
    // 使用位检查，避免-ffast-math消除NaN/Inf检查。
    const bool finite=(pitch & 0x7f800000U)!=0x7f800000U &&
        (velocity_M0 & 0x7f800000U)!=0x7f800000U && (velocity_M1 & 0x7f800000U)!=0x7f800000U;
    const bool fresh=now_us>=sample->device_time_us &&
        now_us-sample->device_time_us<TelemetryMaxAge_us;
    packet[1]=(finite && fresh && sample->current_valid) ? 1 : 0;
    packet[2]=static_cast<std::uint8_t>(sample->sequence);
    packet[3]=static_cast<std::uint8_t>(sample->sequence>>8);
    PutU32(packet,4,static_cast<std::uint32_t>(sample->device_time_us/1000));
    if (finite) { PutU32(packet,8,pitch); PutU32(packet,12,velocity_M0); PutU32(packet,16,velocity_M1); }
    return packet;
}
} // namespace vehicle::ble
