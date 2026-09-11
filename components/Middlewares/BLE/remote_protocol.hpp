#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>

namespace vehicle {
namespace ble {

struct RemoteCommand {
    std::uint16_t sequence{};
    std::int16_t steering{};
    std::int16_t throttle{};
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

constexpr bool parseCommand(std::string_view text, RemoteCommand &out)
{
    RemoteCommand parsed{};
    std::size_t offset = 0;
    int steering = 0;
    int throttle = 0;
    if (text.size() < 3 || text.size() > 20) { return false; }
    if (text[offset++] != 'D') { return false; }
    int sequence = 0;
    if (!comma(text, offset) || offset == text.size() || text[offset] == '-' ||
        !readInteger(text, offset, 65535, sequence)) { return false; }
    parsed.sequence = static_cast<std::uint16_t>(sequence);
    if (!comma(text, offset) || !readInteger(text, offset, 100, steering) ||
         !comma(text, offset) || !readInteger(text, offset, 100, throttle)) {
        return false;
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

} // namespace ble
} // namespace vehicle
