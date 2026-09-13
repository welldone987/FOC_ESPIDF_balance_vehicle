#pragma once

#include <cstdint>

#include "host/ble_gatt.h"

namespace vehicle {
namespace ble {
namespace diagnostic {

/*
 * ble_diagnostic_service维护诊断特征的READ文本与WRITE序号确认。
 * 事件来自统一诊断区，同一事件的长读不会推进游标。
 */
// OnConnected()在新建连接时重置游标并预取首条事件。
int OnConnected();
// OnAccess()处理诊断特征的READ与WRITE确认。
int OnAccess(std::uint16_t conn_handle, std::uint16_t attr_handle,
             ble_gatt_access_ctxt *context, void *argument);
// Prepare()在通知定时或连接建立时预取下一条待读事件。
void Prepare();
// ReportSerial()按串口报告周期输出下一条事件。
void ReportSerial(std::int64_t now_us);

} // namespace diagnostic
} // namespace ble
} // namespace vehicle
