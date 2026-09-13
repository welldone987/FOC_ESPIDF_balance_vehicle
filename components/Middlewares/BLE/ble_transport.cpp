#include "ble_transport.hpp"

#include <atomic>
#include <cstring>

#include "ble_config.hpp"
#include "diagnostics.hpp"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "host/ble_gap.h"
#include "host/ble_hs.h"
#include "host/ble_hs_mbuf.h"
#include "host/util/util.h"
#include "nimble/ble.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

namespace vehicle {
namespace ble {
namespace transport {
namespace {

/*
 * NimBLE主机维护主服务与命令、遥测、诊断三个特征。
 * GAP回调管理连接生命周期并复制原始报文到长度1队列；
 * 命令解析、遥测编码与诊断确认由各服务文件处理。
 */
// 命令、遥测与诊断特征共用同一128位UUID前缀，仅编号字节不同。
#define BALANCE_UUID_BYTES(id) \
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, \
    0x93, 0xf3, 0xa3, 0xb5, id, 0x00, 0x40, 0x6e
// service_uuid是主服务UUID，其余三个UUID对应命令、遥测和诊断特征。
ble_uuid128_t service_uuid = BLE_UUID128_INIT(BALANCE_UUID_BYTES(0x01));
ble_uuid128_t command_uuid = BLE_UUID128_INIT(BALANCE_UUID_BYTES(0x02));
ble_uuid128_t telemetry_uuid = BLE_UUID128_INIT(BALANCE_UUID_BYTES(0x07));
ble_uuid128_t diagnostic_uuid = BLE_UUID128_INIT(BALANCE_UUID_BYTES(0x08));
// characteristics和services保存GATT注册表。
ble_gatt_chr_def characteristics[4]{};
ble_gatt_svc_def services[2]{};
// incoming_queue是GATT回调与服务任务之间的长度1静态队列。
QueueHandle_t incoming_queue{};
// access_handlers和lifecycle_hooks由命令服务在Create()/Start()之前注册。
AccessHandlers access_handlers{};
LifecycleHooks lifecycle_hooks{};
// connection_epoch在每次连接状态变化时自增。
std::uint32_t connection_epoch{};
// ready和ready_error是NimBLE同步回调向Start()交接的静态结果。
std::atomic<int> ready{0};
ErrorInfo ready_error{};
// created避免重复初始化NimBLE。
bool created=false;
// own_address_type保存广播使用的本机地址类型。
std::uint8_t own_address_type=BLE_OWN_ADDR_PUBLIC;
// connection_handle和telemetry_handle保存当前连接与遥测特征值句柄。
std::uint16_t connection_handle=BLE_HS_CONN_HANDLE_NONE;
std::uint16_t telemetry_handle{};
// subscribed标记客户端是否已订阅遥测通知。
bool subscribed=false;

int StartAdvertising(ErrorInfo *error=nullptr);

// PublishConnection()向BleTask队列写入连接状态变化。
void PublishConnection(bool connected)
{
    Incoming input{};
    input.epoch=++connection_epoch;
    input.connected=connected;
    input.received_us=esp_timer_get_time();
    xQueueOverwrite(incoming_queue,&input);
}

// GapEvent()处理连接生命周期和广播结束事件。
int GapEvent(ble_gap_event *event, void *)
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
            if (lifecycle_hooks.connected) { (void)lifecycle_hooks.connected(); }
            ble_gap_conn_desc desc{};
            if (ble_gap_conn_find(connection_handle,&desc)==0) {
                ESP_LOGI("ble","CONNECTED handle=%u interval_ms=%g latency=%u supervision_ms=%u",connection_handle,
                    desc.conn_itvl*1.25,desc.conn_latency,desc.supervision_timeout*10);
            }
            subscribed=false;
            PublishConnection(true);
        } else {
            RememberBle(event->connect.status,ErrorPoint::ble_connect);
            (void)StartAdvertising();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        if (event->disconnect.conn.conn_handle != connection_handle) { return 0; }
        {
            ErrorInfo error{};
            const auto &conn=event->disconnect.conn;
            VEHICLE_ERROR(&error,ESP_FAIL,ble_disconnect,nimble,event->disconnect.reason,
                conn.conn_itvl*1.25f,conn.supervision_timeout*10.0f,conn.conn_latency,7);
            diagnostics::Record(error);
        }
        connection_handle = BLE_HS_CONN_HANDLE_NONE;
        subscribed=false;
        PublishConnection(false);
        (void)StartAdvertising();
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        (void)StartAdvertising();
        return 0;

    case BLE_GAP_EVENT_CONN_UPDATE:
        RememberBle(event->conn_update.status,ErrorPoint::ble_conn_update);
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

// StartAdvertising()发布设备名和服务UUID，并启动可连接广播。
int StartAdvertising(ErrorInfo *error)
{
    ble_hs_adv_fields advertising_fields{};
    advertising_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    advertising_fields.name = reinterpret_cast<std::uint8_t *>(
        const_cast<char *>(DeviceName));
    advertising_fields.name_len = std::strlen(DeviceName);
    advertising_fields.name_is_complete = 1U;

    int rc = ble_gap_adv_set_fields(&advertising_fields);
    if (rc != 0) {
        VEHICLE_ERROR(error,ESP_FAIL,ble_adv_fields,nimble,rc);
        RememberBle(rc,ErrorPoint::ble_adv_fields);
        return rc;
    }

    ble_hs_adv_fields scan_response_fields{};
    scan_response_fields.uuids128 = &service_uuid;
    scan_response_fields.num_uuids128 = 1U;
    scan_response_fields.uuids128_is_complete = 1U;

    rc = ble_gap_adv_rsp_set_fields(&scan_response_fields);
    if (rc != 0) {
        VEHICLE_ERROR(error,ESP_FAIL,ble_scan_fields,nimble,rc);
        RememberBle(rc,ErrorPoint::ble_scan_fields);
        return rc;
    }

    ble_gap_adv_params parameters{};
    parameters.conn_mode = BLE_GAP_CONN_MODE_UND;
    parameters.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(
        own_address_type, nullptr, BLE_HS_FOREVER, &parameters, GapEvent, nullptr);
    if (rc == BLE_HS_EALREADY) { rc=0; }
    if (rc) { VEHICLE_ERROR(error,ESP_FAIL,ble_advertise,nimble,rc); }
    RememberBle(rc,ErrorPoint::ble_advertise);
    return rc;
}

// OnReset()满足NimBLE复位回调接口。
// 复位信息由上层诊断路径处理。
void OnReset(int reason)
{
    RememberBle(reason,ErrorPoint::ble_reset);
    connection_handle = BLE_HS_CONN_HANDLE_NONE;
    subscribed=false;
    if (lifecycle_hooks.reset) { (void)lifecycle_hooks.reset(); }
    PublishConnection(false);
}

// OnSync()取得本机地址类型后开始广播，并通知生命周期钩子。
void OnSync()
{
    ErrorInfo sync_error{};
    int rc = ble_hs_id_infer_auto(0, &own_address_type);
    if (rc) { VEHICLE_ERROR(&sync_error,ESP_FAIL,ble_address,nimble,rc); }
    else { rc=StartAdvertising(&sync_error); }
    if (!rc && lifecycle_hooks.synced) {
        rc=lifecycle_hooks.synced();
        if (rc) { VEHICLE_ERROR(&sync_error,ESP_FAIL,ble_notify,nimble,rc); }
    }
    if (ready.load(std::memory_order_acquire) == 0) {
        if (rc != 0) { ready_error=sync_error; }
        ready.store(rc == 0 ? 1 : -1,std::memory_order_release);
    }
    if (rc) { diagnostics::Record(sync_error); }
}

// HostTask()运行NimBLE主机事件循环，退出后释放其FreeRTOS资源。
void HostTask(void *)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

} // namespace

esp_err_t Create(const AccessHandlers &handlers, QueueHandle_t queue, ErrorInfo *error)
{
    if (created) { return ESP_OK; }
    if (!queue) { return VEHICLE_ERROR(error,ESP_ERR_INVALID_ARG,boot_resource,application,0); }
    incoming_queue=queue;
    access_handlers=handlers;

    const esp_err_t nimble_result = nimble_port_init();
    if (nimble_result != ESP_OK) {
        return VEHICLE_ERROR(error, nimble_result, ble_init, esp, nimble_result);
    }

    ble_svc_gap_init();
    ble_svc_gatt_init();
    // 建立命令、遥测和诊断特征与主服务定义。
    characteristics[0].uuid = &command_uuid.u;
    characteristics[0].access_cb = access_handlers.command;
    characteristics[0].flags = BLE_GATT_CHR_F_WRITE;
    characteristics[1].uuid = &telemetry_uuid.u;
    characteristics[1].flags = BLE_GATT_CHR_F_NOTIFY;
    characteristics[1].val_handle = &telemetry_handle;
    characteristics[2].uuid = &diagnostic_uuid.u;
    characteristics[2].access_cb = access_handlers.diagnostic;
    characteristics[2].flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE;

    services[0].type = BLE_GATT_SVC_TYPE_PRIMARY;
    services[0].uuid = &service_uuid.u;
    services[0].characteristics = characteristics;

    int rc = ble_svc_gap_device_name_set(DeviceName);
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

    ble_hs_cfg.reset_cb = OnReset;
    ble_hs_cfg.sync_cb = OnSync;
    created=true;
    return ESP_OK;
}

void SetLifecycleHooks(const LifecycleHooks &hooks)
{
    lifecycle_hooks=hooks;
}

esp_err_t Start(ErrorInfo *error)
{
    nimble_port_freertos_init(HostTask);
    const auto deadline_us=esp_timer_get_time()+ReadyTimeout_us;
    while (ready.load(std::memory_order_acquire) == 0 && esp_timer_get_time()<deadline_us) { vTaskDelay(pdMS_TO_TICKS(10)); }
    const int state=ready.load(std::memory_order_acquire);
    if (state == 0) { return VEHICLE_ERROR(error, ESP_ERR_TIMEOUT, ble_ready_timeout, application, 0); }
    if (state < 0) { if (error) { *error=ready_error; } return ESP_FAIL; }
    return ESP_OK;
}

std::uint32_t ConnectionEpoch() { return connection_epoch; }

std::uint16_t ConnectionHandle() { return connection_handle; }

bool Subscribed() { return subscribed; }

int Notify(const std::uint8_t *data, std::size_t size)
{
    if (!subscribed || connection_handle==BLE_HS_CONN_HANDLE_NONE) { return BLE_HS_ENOTCONN; }
    auto *buffer=ble_hs_mbuf_from_flat(data,size);
    // NimBLE接管mbuf，发送失败也不能再次释放。
    return buffer ? ble_gatts_notify_custom(connection_handle,telemetry_handle,buffer) : BLE_HS_ENOMEM;
}

void RememberBle(int rc, ErrorPoint point)
{
    if (!rc) { return; }
    ErrorInfo error{};
    ErrorAt(&error,ESP_FAIL,point,ErrorDomain::nimble,rc,__FILE__,__func__,__LINE__);
    diagnostics::Record(error);
}

} // namespace transport
} // namespace ble
} // namespace vehicle
