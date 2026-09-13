#include "ble_diagnostic_service.hpp"

#include <cstring>

#include "ble_transport.hpp"
#include "diagnostics.hpp"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#include "host/ble_hs_mbuf.h"
#include "os/os_mbuf.h"

namespace vehicle {
namespace ble {
namespace diagnostic {
namespace {

// 诊断特征READ返回固定事件文本，WRITE以小端seq确认。
// offered_event保存当前等待客户端确认的诊断事件。
diagnostics::Event offered_event{};
// diagnostic_cursor和serial_cursor分别记录特征确认与串口输出的进度。
std::uint32_t diagnostic_cursor{}, serial_cursor{};
// next_serial_us限制串口事件输出的最早时刻，单位us。
std::int64_t next_serial_us{};
// replay_first_fault为true时下次读取先返回首故障。
bool replay_first_fault=true;
// diagnostic_text保存待读取事件的UTF-8文本。
char diagnostic_text[diagnostics::DiagnosticTextCapacity]{"none"};

} // namespace

void Prepare()
{
    if (offered_event.event_seq) { return; }
    if (diagnostics::ReadEvent(diagnostic_cursor,offered_event,replay_first_fault)) {
        if (!(offered_event.flags & 2)) { replay_first_fault=false; }
        diagnostics::FormatEvent(offered_event,diagnostic_text,sizeof(diagnostic_text));
    } else { std::strcpy(diagnostic_text,"none"); }
}

int OnConnected()
{
    diagnostic_cursor=0;
    offered_event={};
    replay_first_fault=true;
    Prepare();
    return 0;
}

int OnAccess(std::uint16_t conn, std::uint16_t, ble_gatt_access_ctxt *context, void *)
{
    if (!context || !context->om || conn!=transport::ConnectionHandle()) { return BLE_ATT_ERR_UNLIKELY; }
    if (context->op==BLE_GATT_ACCESS_OP_READ_CHR) {
        return os_mbuf_append(context->om,diagnostic_text,std::strlen(diagnostic_text))==0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    if (context->op!=BLE_GATT_ACCESS_OP_WRITE_CHR) { return BLE_ATT_ERR_WRITE_NOT_PERMITTED; }
    std::uint8_t bytes[4]{};
    std::uint16_t copied{};
    if (OS_MBUF_PKTLEN(context->om)!=4 || ble_hs_mbuf_to_flat(context->om,bytes,4,&copied)!=0 || copied!=4) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    const std::uint32_t seq=bytes[0] | (std::uint32_t{bytes[1]}<<8) | (std::uint32_t{bytes[2]}<<16) | (std::uint32_t{bytes[3]}<<24);
    if (!seq || seq!=offered_event.event_seq) { return BLE_ATT_ERR_VALUE_NOT_ALLOWED; }
    // 首故障先重放，之后从环中最早保留事件开始。
    // 独立首故障不能跳过较早非致命事件。
    if (replay_first_fault) { replay_first_fault=false; }
    else { diagnostic_cursor=seq; }
    offered_event={};
    Prepare();
    return 0;
}

void ReportSerial(std::int64_t now_us)
{
    if (now_us < next_serial_us) { return; }
    next_serial_us=now_us+diagnostics::SerialReportPeriod_us;
    diagnostics::Event event{};
    if (diagnostics::ReadEvent(serial_cursor,event)) {
        char text[diagnostics::DiagnosticTextCapacity]{};
        diagnostics::FormatEvent(event,text,sizeof(text));
        ESP_LOGW("diagnostic","%s",text);
        serial_cursor=event.event_seq;
    }
}

} // namespace diagnostic
} // namespace ble
} // namespace vehicle
