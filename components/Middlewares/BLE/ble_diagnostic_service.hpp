#pragma once

#include <cstdint>

#include "host/ble_gatt.h"

namespace vehicle {
namespace ble {
namespace diagnostic {

/*
 * ble_diagnostic_service只负责诊断特征的GATT IO：
 * 读文本、写4字节序号确认；事件游标与首故障重放由diagnostics::reader维护。
 */
// OnConnected()在新建连接时重置读取游标。
int OnConnected();
// OnAccess()处理诊断特征的READ与WRITE确认。
int OnAccess(std::uint16_t conn_handle, std::uint16_t attr_handle,
             ble_gatt_access_ctxt *context, void *argument);

} // namespace diagnostic
} // namespace ble
} // namespace vehicle
