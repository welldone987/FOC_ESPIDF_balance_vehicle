#pragma once

#include <cstddef>

#include "diagnostics.hpp"

namespace vehicle {
namespace diagnostics {
/*
 * diagnostic_text是纯渲染层：把事件渲染为固定键序的UTF-8文本。
 * 无状态、不持锁；串口与BLE诊断特征共用同一渲染结果。
 */
// FormatEvent()把事件渲染为固定键序文本；可选键缺值打印NULL，数值带错误点单位。
int FormatEvent(const Event &event, char *buffer, std::size_t capacity);
} // namespace diagnostics
} // namespace vehicle
