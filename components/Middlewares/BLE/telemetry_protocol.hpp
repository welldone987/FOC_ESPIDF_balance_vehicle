#pragma once
#include <array>
#include <cstdint>
#include <cstring>
#include "ble_config.hpp"
#include "telemetry_snapshot.hpp"

namespace vehicle::ble {
/*
 * 遥测特征把最新TelemetrySnapshot编码为固定20字节小端报文。
 * EncodeTelemetry()检查有限性和样本年龄后填充俯仰角与两轮速度。
 */
// 固定20字节、小端、IEEE-754。
// 不直接发送具有填充字节的C++结构体。
using TelemetryPacket = std::array<std::uint8_t, 20>;
// FloatBits()把float按IEEE-754位模式复制为uint32。
inline std::uint32_t FloatBits(float value)
{
    static_assert(sizeof(float) == sizeof(std::uint32_t));
    std::uint32_t bits{};
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}
// PutU32()把小端uint32写入报文指定偏移。
inline void PutU32(TelemetryPacket &packet, unsigned offset_bytes, std::uint32_t value)
{
    for (unsigned i=0; i<4; ++i) { packet[offset_bytes+i]=static_cast<std::uint8_t>(value>>(8*i)); }
}
// EncodeTelemetry()编码一帧遥测。
// 无快照或数据无效时返回版本1和全零。
inline TelemetryPacket EncodeTelemetry(const control::TelemetrySnapshot *sample,
                                       std::int64_t now_us)
{
    TelemetryPacket packet{};
    // packet[0]是协议版本号。
    packet[0]=1;
    if (!sample) { return packet; }
    const auto pitch_bits=FloatBits(sample->pitch_deg);
    const auto velocity_M0_bits=FloatBits(sample->velocity_M0_rad_s);
    const auto velocity_M1_bits=FloatBits(sample->velocity_M1_rad_s);
    // 使用位检查，避免-ffast-math消除NaN/Inf检查。
    const bool finite=(pitch_bits & 0x7f800000U)!=0x7f800000U &&
        (velocity_M0_bits & 0x7f800000U)!=0x7f800000U && (velocity_M1_bits & 0x7f800000U)!=0x7f800000U;
    const bool fresh=now_us>=sample->device_time_us &&
        now_us-sample->device_time_us<TelemetryMaxAge_us;
    // packet[1]的bit0标记本帧数据是否有效。
    packet[1]=(finite && fresh && sample->current_valid) ? 1 : 0;
    // packet[2..3]是控制序号低16位。
    packet[2]=static_cast<std::uint8_t>(sample->sequence);
    packet[3]=static_cast<std::uint8_t>(sample->sequence>>8);
    // packet[4..7]是采样时刻的毫秒低32位。
    PutU32(packet,4,static_cast<std::uint32_t>(sample->device_time_us/1000));
    if (finite) { PutU32(packet,8,pitch_bits); PutU32(packet,12,velocity_M0_bits); PutU32(packet,16,velocity_M1_bits); }
    return packet;
}
} // namespace vehicle::ble
