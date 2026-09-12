#include "ble_command_service.hpp"
#include "diagnostics.hpp"
#include "remote_protocol.hpp"
#include "motion_command.hpp"
#include "telemetry_protocol.hpp"
#include <atomic>

#include <cstdint>
#include <cstring>
#include <string_view>

#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ble_config.hpp"
#include "control_config.hpp"

#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_hs_mbuf.h"
#include "host/util/util.h"
#include "nimble/ble.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "os/os_mbuf.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

namespace vehicle {
namespace ble {
namespace {

// .002兼容成功版本的X,Y命令；.007为独立的20字节遥测协议。
#define BALANCE_UUID_BYTES(id) \
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, \
    0x93, 0xf3, 0xa3, 0xb5, id, 0x00, 0x40, 0x6e
ble_uuid128_t service_uuid = BLE_UUID128_INIT(BALANCE_UUID_BYTES(0x01));
ble_uuid128_t command_uuid = BLE_UUID128_INIT(BALANCE_UUID_BYTES(0x02));
ble_uuid128_t telemetry_uuid = BLE_UUID128_INIT(BALANCE_UUID_BYTES(0x07));
ble_uuid128_t diagnostic_uuid = BLE_UUID128_INIT(BALANCE_UUID_BYTES(0x08));
ble_gatt_chr_def characteristics[4]{};
ble_gatt_svc_def services[2]{};
struct Incoming {
    char text[20]{};
    std::uint16_t length{};
    std::uint32_t epoch{};
    std::int64_t received_us{};
    bool connected{};
};
StaticQueue_t incoming_storage{};
std::uint8_t incoming_buffer[sizeof(Incoming)]{};
QueueHandle_t incoming_queue{};
// 连接句柄和epoch仅在NimBLE主机任务使用，跨任务只传值。
std::uint32_t connection_epoch{};
std::atomic<int> ready{0};
ErrorInfo ready_error{};
bool initialized=false;
std::uint8_t own_address_type=BLE_OWN_ADDR_PUBLIC;
std::uint16_t connection_handle=BLE_HS_CONN_HANDLE_NONE;
std::uint16_t telemetry_handle{};
bool subscribed=false;
QueueHandle_t telemetry_queue{};
ble_npl_callout telemetry_timer{};
// .008 READ固定事件文本，WRITE小端seq确认；同一事件的长读不会推进游标。
diagnostics::Event offered_event{};
std::uint32_t diagnostic_cursor{}, serial_cursor{};
bool replay_first_fault=true;
char diagnostic_text[512]{"none"};
std::int64_t last_notify_error_us{};
unsigned notify_failures{};
void rememberBle(int rc, ErrorPoint point);

void prepareDiagnostic()
{
    if (offered_event.event_seq) { return; }
    if (diagnostics::readEvent(diagnostic_cursor,offered_event,replay_first_fault)) {
        if (!(offered_event.flags & 2)) { replay_first_fault=false; }
        diagnostics::formatEvent(offered_event,diagnostic_text,sizeof(diagnostic_text));
    } else { std::strcpy(diagnostic_text,"none"); }
}

int diagnosticAccess(std::uint16_t conn, std::uint16_t, ble_gatt_access_ctxt *context, void *)
{
    if (!context || !context->om || conn!=connection_handle) { return BLE_ATT_ERR_UNLIKELY; }
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
    // 首故障先重放，之后从环中最早保留事件开始；独立首故障不能跳过较早非致命事件。
    if (replay_first_fault) { replay_first_fault=false; }
    else { diagnostic_cursor=seq; }
    offered_event={};
    prepareDiagnostic();
    return 0;
}

TelemetryPacket latestTelemetry()
{
    wifi_telemtry::TelemetrySnapshot sample{};
    const bool have=xQueuePeek(telemetry_queue,&sample,0)==pdTRUE;
    return encodeTelemetry(have ? &sample : nullptr,esp_timer_get_time());
}

// 定时事件与GAP回调均在NimBLE主机执行，句柄/订阅状态不跨任务共享。
void sendTelemetry(ble_npl_event *)
{
    prepareDiagnostic();
    if (subscribed && connection_handle!=BLE_HS_CONN_HANDLE_NONE) {
        const auto packet=latestTelemetry();
        auto *buffer=ble_hs_mbuf_from_flat(packet.data(),packet.size());
        // NimBLE接管mbuf，发送失败也不能再次释放。
        const int rc=buffer ? ble_gatts_notify_custom(connection_handle,telemetry_handle,buffer) : BLE_HS_ENOMEM;
        if (rc) {
            ++notify_failures;
            const auto now=esp_timer_get_time();
            if (!last_notify_error_us || now-last_notify_error_us>=1000000) {
                ErrorInfo error{};
                VEHICLE_ERROR(&error,ESP_FAIL,ble_notify,nimble,rc,static_cast<float>(notify_failures),0,-1,1);
                diagnostics::record(error);
                last_notify_error_us=now;
            }
        }
    }
    (void)ble_npl_callout_reset(&telemetry_timer,ble_npl_time_ms_to_ticks32(100));
}

int telemetryAccess(std::uint16_t, std::uint16_t, ble_gatt_access_ctxt *context, void *)
{
    if (!context || !context->om || context->op!=BLE_GATT_ACCESS_OP_READ_CHR) {
        return BLE_ATT_ERR_READ_NOT_PERMITTED;
    }
    const auto packet=latestTelemetry();
    return os_mbuf_append(context->om,packet.data(),packet.size())==0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}
void publishConnection(bool connected)
{
    Incoming input{};
    input.epoch=++connection_epoch;
    input.connected=connected;
    input.received_us=esp_timer_get_time();
    xQueueOverwrite(incoming_queue,&input);
}
void rememberBle(int rc, ErrorPoint point)
{
    if (!rc) { return; }
    ErrorInfo error{};
    errorAt(&error,ESP_FAIL,point,ErrorDomain::nimble,rc,__FILE__,__func__,__LINE__);
    diagnostics::record(error);
}

int startAdvertising(ErrorInfo *error=nullptr);

// gapEvent()处理连接生命周期和广播结束事件。
int gapEvent(ble_gap_event *event, void *)
{
    if (event == nullptr) {
        return 0;
    }

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            if (connection_handle != BLE_HS_CONN_HANDLE_NONE) {
                (void)ble_gap_terminate(event->connect.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
                return 0;
            }
            connection_handle = event->connect.conn_handle;
            diagnostic_cursor=0; offered_event={}; replay_first_fault=true; prepareDiagnostic();
            ble_gap_conn_desc desc{};
            if (ble_gap_conn_find(connection_handle,&desc)==0) {
                ESP_LOGI("ble","CONNECTED handle=%u interval_ms=%g latency=%u supervision_ms=%u",connection_handle,
                    desc.conn_itvl*1.25,desc.conn_latency,desc.supervision_timeout*10);
            }
            subscribed=false;
            publishConnection(true);
        } else {
            rememberBle(event->connect.status,ErrorPoint::ble_connect);
            (void)startAdvertising();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        if (event->disconnect.conn.conn_handle != connection_handle) { return 0; }
        {
            ErrorInfo error{};
            const auto &conn=event->disconnect.conn;
            VEHICLE_ERROR(&error,ESP_FAIL,ble_disconnect,nimble,event->disconnect.reason,
                conn.conn_itvl*1.25f,conn.supervision_timeout*10.0f,conn.conn_latency,7);
            diagnostics::record(error);
        }
        connection_handle = BLE_HS_CONN_HANDLE_NONE;
        subscribed=false;
        publishConnection(false);
        (void)startAdvertising();
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        (void)startAdvertising();
        return 0;

    case BLE_GAP_EVENT_CONN_UPDATE:
        rememberBle(event->conn_update.status,ErrorPoint::ble_conn_update);
        {
            ble_gap_conn_desc desc{};
            if (ble_gap_conn_find(event->conn_update.conn_handle,&desc)==0) {
                ESP_LOGI("ble","CONN_UPDATE interval_ms=%g latency=%u supervision_ms=%u",desc.conn_itvl*1.25,
                    desc.conn_latency,desc.supervision_timeout*10);
            }
        }
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.conn_handle==connection_handle &&
            event->subscribe.attr_handle==telemetry_handle) {
            subscribed=event->subscribe.cur_notify!=0;
        }
        return 0;

    default:
        return 0;
    }
}

// startAdvertising()发布设备名和服务UUID，并启动可连接广播。
int startAdvertising(ErrorInfo *error)
{
    ble_hs_adv_fields advertising_fields{};
    advertising_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    advertising_fields.name = reinterpret_cast<std::uint8_t *>(
        const_cast<char *>(config::kDeviceName));
    advertising_fields.name_len = std::strlen(config::kDeviceName);
    advertising_fields.name_is_complete = 1U;

    int rc = ble_gap_adv_set_fields(&advertising_fields);
    if (rc != 0) {
        VEHICLE_ERROR(error,ESP_FAIL,ble_adv_fields,nimble,rc);
        rememberBle(rc,ErrorPoint::ble_adv_fields);
        return rc;
    }

    ble_hs_adv_fields scan_response_fields{};
    scan_response_fields.uuids128 = &service_uuid;
    scan_response_fields.num_uuids128 = 1U;
    scan_response_fields.uuids128_is_complete = 1U;

    rc = ble_gap_adv_rsp_set_fields(&scan_response_fields);
    if (rc != 0) {
        VEHICLE_ERROR(error,ESP_FAIL,ble_scan_fields,nimble,rc);
        rememberBle(rc,ErrorPoint::ble_scan_fields);
        return rc;
    }

    ble_gap_adv_params parameters{};
    parameters.conn_mode = BLE_GAP_CONN_MODE_UND;
    parameters.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(
        own_address_type, nullptr, BLE_HS_FOREVER, &parameters, gapEvent, nullptr);
    if (rc == BLE_HS_EALREADY) { rc=0; }
    if (rc) { VEHICLE_ERROR(error,ESP_FAIL,ble_advertise,nimble,rc); }
    rememberBle(rc,ErrorPoint::ble_advertise);
    return rc;
}

// onReset()满足NimBLE复位回调接口；复位信息由上层诊断路径处理。
void onReset(int reason)
{
    rememberBle(reason,ErrorPoint::ble_reset);
    connection_handle = BLE_HS_CONN_HANDLE_NONE;
    subscribed=false;
    ble_npl_callout_stop(&telemetry_timer);
    publishConnection(false);
}

// onSync()取得本机地址类型后开始广播。
void onSync()
{
    ErrorInfo sync_error{};
    int rc = ble_hs_id_infer_auto(0, &own_address_type);
    if (rc) { VEHICLE_ERROR(&sync_error,ESP_FAIL,ble_address,nimble,rc); }
    else { rc=startAdvertising(&sync_error); }
    if (!rc) {
        rc=ble_npl_callout_reset(&telemetry_timer,ble_npl_time_ms_to_ticks32(100));
        if (rc) { VEHICLE_ERROR(&sync_error,ESP_FAIL,ble_notify,nimble,rc); }
    }
    if (ready.load(std::memory_order_acquire) == 0) {
        if (rc != 0) { ready_error=sync_error; }
        ready.store(rc == 0 ? 1 : -1,std::memory_order_release);
    }
    if (rc) { diagnostics::record(sync_error); }
}

// hostTask()运行NimBLE主机事件循环，退出后释放其FreeRTOS资源。
void hostTask(void *)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// 一个WRITE复制一个固定长度报文；内容由BleTask严格解析。
int characteristicAccess(std::uint16_t conn_handle,
                         std::uint16_t,
                         ble_gatt_access_ctxt *context,
                         void *)
{
    if (context == nullptr || context->om == nullptr) { return BLE_ATT_ERR_UNLIKELY; }
    if (context->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        constexpr char response[]="X,Y; range=-100..100";
        return os_mbuf_append(context->om,response,sizeof(response)-1)==0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    if (context->op != BLE_GATT_ACCESS_OP_WRITE_CHR || conn_handle != connection_handle) {
        return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
    }
    Incoming input{};
    input.length=OS_MBUF_PKTLEN(context->om);
    if (input.length == 0 || input.length > sizeof(input.text)) {
        rememberBle(BLE_HS_ATT_ERR(BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN),ErrorPoint::ble_command);
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    std::uint16_t copied{};
    if (ble_hs_mbuf_to_flat(context->om,input.text,input.length,&copied) != 0 || copied != input.length) {
        rememberBle(BLE_HS_ATT_ERR(BLE_ATT_ERR_UNLIKELY),ErrorPoint::ble_command);
        return BLE_ATT_ERR_UNLIKELY;
    }
    input.connected=true;
    input.epoch=connection_epoch;
    input.received_us=esp_timer_get_time();
    // GATT回调只复制原始报文，协议解析及量纲转换由BleTask执行。
    xQueueOverwrite(incoming_queue,&input);
    return 0;
}

} // namespace

esp_err_t initialize(QueueHandle_t snapshots, ErrorInfo *error)
{
    if (initialized) {
        return ESP_OK;
    }

    if (!snapshots) { return VEHICLE_ERROR(error,ESP_ERR_INVALID_ARG,boot_resource,application,0); }
    telemetry_queue=snapshots;
    incoming_queue=xQueueCreateStatic(1,sizeof(Incoming),incoming_buffer,&incoming_storage);
    if (!incoming_queue) { return VEHICLE_ERROR(error,ESP_ERR_NO_MEM,boot_resource,application,0); }
    const esp_err_t nimble_result = nimble_port_init();
    if (nimble_result != ESP_OK) {
        return VEHICLE_ERROR(error, nimble_result, ble_init, esp, nimble_result);
    }

    ble_svc_gap_init();
    ble_svc_gatt_init();
    // 建立命令特征和主服务定义。
    characteristics[0].uuid = &command_uuid.u;
    characteristics[0].access_cb = characteristicAccess;
    characteristics[0].flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE;
    characteristics[1].uuid = &telemetry_uuid.u;
    characteristics[1].access_cb = telemetryAccess;
    characteristics[1].flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY;
    characteristics[1].val_handle = &telemetry_handle;
    characteristics[2].uuid = &diagnostic_uuid.u;
    characteristics[2].access_cb = diagnosticAccess;
    characteristics[2].flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE;

    services[0].type = BLE_GATT_SVC_TYPE_PRIMARY;
    services[0].uuid = &service_uuid.u;
    services[0].characteristics = characteristics;

    int rc = ble_svc_gap_device_name_set(config::kDeviceName);
    if (rc != 0) {
        return VEHICLE_ERROR(error, ESP_FAIL, ble_name, nimble, rc);
    }

    rc = ble_gatts_count_cfg(services);
    if (rc != 0) {
        return VEHICLE_ERROR(error, ESP_FAIL, ble_count, nimble, rc);
    }

    rc = ble_gatts_add_svcs(services);
    if (rc != 0) {
        return VEHICLE_ERROR(error, ESP_FAIL, ble_services, nimble, rc);
    }

    ble_hs_cfg.reset_cb = onReset;
    ble_hs_cfg.sync_cb = onSync;

    ble_npl_callout_init(&telemetry_timer,nimble_port_get_dflt_eventq(),sendTelemetry,nullptr);
    nimble_port_freertos_init(hostTask);
    const auto deadline=esp_timer_get_time()+5000000;
    while (ready.load(std::memory_order_acquire) == 0 && esp_timer_get_time()<deadline) { vTaskDelay(pdMS_TO_TICKS(10)); }
    const int state=ready.load(std::memory_order_acquire);
    if (state == 0) { return VEHICLE_ERROR(error, ESP_ERR_TIMEOUT, ble_ready_timeout, application, 0); }
    if (state < 0) { if (error) { *error=ready_error; } return ESP_FAIL; }
    initialized=true;
    return ESP_OK;
}

void run(QueueHandle_t command_queue)
{
    std::uint32_t epoch{};
    std::int64_t next_serial_us{};
    for (;;) {
        // 低优先级BleTask打印，避免串口吞吐占用NimBLE主机回调或ControlTask。
        const auto now=esp_timer_get_time();
        if (now>=next_serial_us) {
            next_serial_us=now+100000;
            diagnostics::Event event{};
            if (diagnostics::readEvent(serial_cursor,event)) {
                char text[512]{};
                diagnostics::formatEvent(event,text,sizeof(text));
                ESP_LOGW("diagnostic","%s",text);
                serial_cursor=event.event_seq;
            }
        }
        Incoming input{};
        if (xQueueReceive(incoming_queue,&input,pdMS_TO_TICKS(100)) != pdTRUE) { continue; }
        if (input.epoch != epoch) {
            epoch=input.epoch;
            const control::MotionCommand zero{};
            xQueueOverwrite(command_queue,&zero);
        }
        if (!input.connected || input.length == 0) { continue; }
        RemoteCommand parsed{};
        if (!parseCommand(std::string_view(input.text,input.length),parsed)) {
            ErrorInfo error{};
            VEHICLE_ERROR(&error,ESP_ERR_INVALID_ARG,ble_command,application,0,static_cast<float>(input.length),20,-1,3);
            diagnostics::record(error);
            // 无效输入不能让旧非零目标继续有效。
            const control::MotionCommand zero{};
            xQueueOverwrite(command_queue,&zero);
            continue;
        }
        const control::MotionCommand command{
            parsed.throttle * control::config::kDriveSpeedLimitRadS / 100.0f,
            parsed.steering * control::config::kYawRateLimitRadS / 100.0f,
            input.received_us,true};
        // 长度1最新值语义；过期判断使用接收时刻，不能因排队延长寿命。
        xQueueOverwrite(command_queue,&command);
    }
}

} // namespace ble
} // namespace vehicle
