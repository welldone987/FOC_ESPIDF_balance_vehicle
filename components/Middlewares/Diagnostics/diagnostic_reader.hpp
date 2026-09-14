#pragma once

#include <cstdint>

namespace vehicle {
namespace diagnostics {
namespace reader {
/*
 * reader是BLE诊断特征的消费游标：定位当前待确认事件、按需渲染文本与序号确认。
 * 单实例（诊断特征），状态为文件内静态；BLE侧只做GATT读写。
 */
// Rewind()在新建连接时重置游标，使下次读取先重放首故障。
void Rewind();
// Ack()校验并确认事件序号；成功后推进游标。
bool Ack(std::uint32_t seq);
// Text()定位当前待读事件并按需渲染；无事件时为"none"。
const char *Text();
} // namespace reader
} // namespace diagnostics
} // namespace vehicle
