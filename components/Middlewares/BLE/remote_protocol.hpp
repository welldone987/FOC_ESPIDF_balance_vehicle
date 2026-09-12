#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>

namespace vehicle {
namespace ble {

struct RemoteCommand {
    std::int16_t steering{};
    std::int16_t throttle{};
};

constexpr bool ReadInteger(std::string_view text, std::size_t &offset,
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

constexpr bool ConsumeComma(std::string_view text, std::size_t &offset)
{
    return offset < text.size() && text[offset++] == ',';
}

constexpr bool ParseCommand(std::string_view text, RemoteCommand &out)
{
    RemoteCommand parsed{};
    std::size_t offset = 0;
    int steering = 0;
    int throttle = 0;
    if (text.size() < 3 || text.size() > 20) { return false; }
    // 兼容成功版本的X,Y、X,Y\n和X,Y\r\n；其它尾随内容一律拒绝。
    if (text.back() == '\n') {
        text.remove_suffix(1);
        if (!text.empty() && text.back() == '\r') { text.remove_suffix(1); }
    }
    if (!ReadInteger(text, offset, 100, steering) ||
         !ConsumeComma(text, offset) || !ReadInteger(text, offset, 100, throttle)) {
        return false;
    }
    if (offset != text.size()) { return false; }
    parsed.steering = static_cast<std::int16_t>(steering);
    parsed.throttle = static_cast<std::int16_t>(throttle);
    out = parsed;
    return true;
}

} // namespace ble
} // namespace vehicle
