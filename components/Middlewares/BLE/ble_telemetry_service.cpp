#include "ble_telemetry_service.hpp"

#include "ble_config.hpp"
#include "ble_diagnostic_service.hpp"
#include "ble_transport.hpp"
#include "diagnostics.hpp"
#include "telemetry_protocol.hpp"

#include "esp_timer.h"

#include "host/ble_hs.h"
#include "nimble/nimble_port.h"

namespace vehicle {
namespace ble {
namespace telemetry {
namespace {

// telemetry_queue保存控制任务发布的最近遥测快照。
QueueHandle_t telemetry_queue{};
// telemetry_timer以NotifyPeriod_ms周期触发SendTelemetry()。
ble_npl_callout telemetry_timer{};
// last_notify_error_us和notify_failures用于notify失败日志节流。
std::int64_t last_notify_error_us{};
unsigned notify_failures{};

// LatestTelemetry()从共享队列peek最新快照并编码为报文。
TelemetryPacket LatestTelemetry()
{
    control::TelemetrySnapshot sample{};
    const bool have=xQueuePeek(telemetry_queue,&sample,0)==pdTRUE;
    return EncodeTelemetry(have ? &sample : nullptr,esp_timer_get_time());
}

// SendTelemetry()由notify定时器触发：预取诊断文本并发送最新遥测。
// 定时事件与GAP回调均在NimBLE主机执行，连接状态不跨任务共享。
void SendTelemetry(ble_npl_event *)
{
    diagnostic::Prepare();
    if (transport::Subscribed() && transport::ConnectionHandle()!=BLE_HS_CONN_HANDLE_NONE) {
        const auto packet=LatestTelemetry();
        const int rc=transport::Notify(packet.data(),packet.size());
        if (rc) {
            ++notify_failures;
            const auto now=esp_timer_get_time();
            if (!last_notify_error_us || now-last_notify_error_us>=NotifyErrorThrottle_us) {
                ErrorInfo error{};
                VEHICLE_ERROR(&error,ESP_FAIL,ble_notify,rc,static_cast<float>(notify_failures),0,-1, ErrorValue);
                diagnostics::Record(error);
                last_notify_error_us=now;
            }
        }
    }
    (void)ble_npl_callout_reset(&telemetry_timer,ble_npl_time_ms_to_ticks32(NotifyPeriod_ms));
}

} // namespace

void Initialize(QueueHandle_t queue)
{
    telemetry_queue=queue;
    ble_npl_callout_init(&telemetry_timer,nimble_port_get_dflt_eventq(),SendTelemetry,nullptr);
}

int Start()
{
    return ble_npl_callout_reset(&telemetry_timer,ble_npl_time_ms_to_ticks32(NotifyPeriod_ms));
}

int Stop()
{
    ble_npl_callout_stop(&telemetry_timer);
    return 0;
}

} // namespace telemetry
} // namespace ble
} // namespace vehicle
