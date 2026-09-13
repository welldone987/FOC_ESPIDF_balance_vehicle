#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>

namespace vehicle {
namespace ble {
/*
 * ReadInteger()和ConsumeComma()按字节解析X,Y载荷并拒绝溢出。
 * ParseCommand()兼容X,Y、X,Y\n和X,Y\r\n三种形式。
 */
// RemoteCommand保存网页控制端的转向与油门百分比（±100）。
struct RemoteCommand {
    // steering_percent是转向百分比，X正为右转。
    std::int16_t steering_percent{};
    // throttle_percent是油门百分比，Y正为前进。
    std::int16_t throttle_percent{};
};

// ReadInteger()读取可带符号的十进制整数并拒绝溢出。
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

// ConsumeComma()消费字段之间的逗号。
constexpr bool ConsumeComma(std::string_view text, std::size_t &offset)
{
    return offset < text.size() && text[offset++] == ',';
}

// ParseCommand()解析完整X,Y载荷，尾随内容一律拒绝。
constexpr bool ParseCommand(std::string_view text, RemoteCommand &out)
{
    RemoteCommand parsed{};
    std::size_t offset = 0;
    int steering_percent = 0;
    int throttle_percent = 0;
    if (text.size() < 3 || text.size() > 20) { return false; }
    // 兼容成功版本的X,Y、X,Y\n和X,Y\r\n。
    // 其它尾随内容一律拒绝。
    if (text.back() == '\n') {
        text.remove_suffix(1);
        if (!text.empty() && text.back() == '\r') { text.remove_suffix(1); }
    }
    if (!ReadInteger(text, offset, 100, steering_percent) ||
         !ConsumeComma(text, offset) || !ReadInteger(text, offset, 100, throttle_percent)) {
        return false;
    }
    if (offset != text.size()) { return false; }
    parsed.steering_percent = static_cast<std::int16_t>(steering_percent);
    parsed.throttle_percent = static_cast<std::int16_t>(throttle_percent);
    out = parsed;
    return true;
}

} // namespace ble
} // namespace vehicle
