#include "ble_diagnostic_service.hpp"

#include <cstring>

#include "ble_transport.hpp"
#include "diagnostic_reader.hpp"

#include "freertos/FreeRTOS.h"

#include "host/ble_hs_mbuf.h"
#include "os/os_mbuf.h"

namespace vehicle {
namespace ble {
namespace diagnostic {

int OnConnected()
{
    // 连接重建后的读取顺序由reader统一维护：先重放首故障，再遍历保留环。
    diagnostics::reader::Rewind();
    return 0;
}

int OnAccess(std::uint16_t conn, std::uint16_t, ble_gatt_access_ctxt *context, void *)
{
    if (!context || !context->om || conn!=transport::ConnectionHandle()) { return BLE_ATT_ERR_UNLIKELY; }
    if (context->op==BLE_GATT_ACCESS_OP_READ_CHR) {
        const char *text=diagnostics::reader::Text();
        return os_mbuf_append(context->om,text,std::strlen(text))==0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    if (context->op!=BLE_GATT_ACCESS_OP_WRITE_CHR) { return BLE_ATT_ERR_WRITE_NOT_PERMITTED; }
    std::uint8_t bytes[4]{};
    std::uint16_t copied{};
    if (OS_MBUF_PKTLEN(context->om)!=4 || ble_hs_mbuf_to_flat(context->om,bytes,4,&copied)!=0 || copied!=4) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    const std::uint32_t seq=bytes[0] | (std::uint32_t{bytes[1]}<<8) | (std::uint32_t{bytes[2]}<<16) | (std::uint32_t{bytes[3]}<<24);
    return diagnostics::reader::Ack(seq) ? 0 : BLE_ATT_ERR_VALUE_NOT_ALLOWED;
}

} // namespace diagnostic
} // namespace ble
} // namespace vehicle
