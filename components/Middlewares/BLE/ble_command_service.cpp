#include "ble_command_service.hpp"
#include "diagnostics.hpp"
#include "ble_status_codec.hpp"
#include <atomic>

#include <algorithm>
#include <bit>
#include <cstdint>
#include <cstring>
#include <string_view>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nvs_flash.h"
#include "vehicle_config.hpp"

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

// 保留旧命令特征；新命令使用独立UUID，避免静默改变旧协议量纲。
#define BALANCE_UUID_BYTES(id) \
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, \
    0x93, 0xf3, 0xa3, 0xb5, id, 0x00, 0x40, 0x6e
ble_uuid128_t service_uuid = BLE_UUID128_INIT(BALANCE_UUID_BYTES(0x01));
ble_uuid128_t command_uuid = BLE_UUID128_INIT(BALANCE_UUID_BYTES(0x02));
ble_uuid128_t command_v2_uuid = BLE_UUID128_INIT(BALANCE_UUID_BYTES(0x04));
ble_uuid128_t status_uuid = BLE_UUID128_INIT(BALANCE_UUID_BYTES(0x03));
ble_uuid128_t diag_uuid = BLE_UUID128_INIT(BALANCE_UUID_BYTES(0x05));
ble_gatt_chr_def characteristics[5]{};
ble_gatt_svc_def services[2]{};

// 临界区只复制/更新固定长度状态，不在锁内格式化、分配内存或调用NimBLE。
portMUX_TYPE state_lock = portMUX_INITIALIZER_UNLOCKED;
RemoteState remote_state{};
StatusSnapshot latest_status{};
std::uint32_t connection_epoch{};
// 以下连接/订阅/通知资源仅由NimBLE主机访问。
ble_npl_callout status_timer{};
std::uint16_t status_handle{}, diag_handle{}, mtu{23};
bool diag_subscribed{};
diagnostics::ReplayCursor diag_cursor{};
std::atomic<int> ready{0};
ErrorInfo ready_error{};
bool subscribed{};
bool notify_pending{};
bool initialized = false;
std::uint8_t own_address_type = BLE_OWN_ADDR_PUBLIC;
std::uint16_t connection_handle = BLE_HS_CONN_HANDLE_NONE;

void publishConnection(bool connected)
{
    portENTER_CRITICAL(&state_lock);
    remoteConnection(remote_state, connected);
    ++connection_epoch;
    portEXIT_CRITICAL(&state_lock);
    diagnostics::bleState(connected, subscribed, diag_subscribed, mtu);
}

void encodeStatus(std::uint8_t (&packet)[20])
{
    const auto now=esp_timer_get_time();
    portENTER_CRITICAL(&state_lock);
    const auto status=latest_status;
    const auto remote=remote_state;
    const auto epoch=connection_epoch;
    portEXIT_CRITICAL(&state_lock);
    encodeStatusPacket(packet,status,remote,epoch,now);
}

int statusAccess(std::uint16_t, std::uint16_t, ble_gatt_access_ctxt *context, void *)
{
    if (context == nullptr || context->op != BLE_GATT_ACCESS_OP_READ_CHR) {
        return BLE_ATT_ERR_READ_NOT_PERMITTED;
    }
    std::uint8_t packet[20]{};
    encodeStatus(packet);
    return os_mbuf_append(context->om, packet, sizeof(packet)) == 0
        ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}


int diagAccess(std::uint16_t, std::uint16_t, ble_gatt_access_ctxt *context, void *)
{
    if (!context || context->op != BLE_GATT_ACCESS_OP_READ_CHR) { return BLE_ATT_ERR_READ_NOT_PERMITTED; }
    std::uint8_t packet[20]{};
    diagnostics::metadata(packet);
    return os_mbuf_append(context->om, packet, sizeof(packet)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}
void rememberBle(int rc, ErrorPoint point)
{
    if (!rc) { return; }
    ErrorInfo error{};
    errorAt(&error, ESP_FAIL, point, ErrorDomain::nimble, rc, __FILE__, __func__, __LINE__);
    diagnostics::bleError(error);
}
void notifyDiag()
{
    if (!diag_subscribed || connection_handle == BLE_HS_CONN_HANDLE_NONE) { return; }
    diagnostics::Event event{};
    bool first{};
    if (!diagnostics::replay(diag_cursor,event,first)) { return; }
    std::uint8_t packet[14]{};
    diagnostics::encodeEvent(event,packet);
    auto *buffer=ble_hs_mbuf_from_flat(packet,sizeof(packet));
    const int rc=buffer ? ble_gatts_notify_custom(connection_handle,diag_handle,buffer) : BLE_HS_ENOMEM;
    rememberBle(rc,ErrorPoint::ble_diag_notify);
    diagnostics::acceptReplay(diag_cursor,event,first,rc == 0);
}
// 回调运行在NimBLE主机事件队列；不为遥测新增任务或积压历史帧。
void notifyStatus(ble_npl_event *)
{
    if (subscribed && !notify_pending && connection_handle != BLE_HS_CONN_HANDLE_NONE) {
        std::uint8_t packet[20]{};
        encodeStatus(packet);
        os_mbuf *buffer = ble_hs_mbuf_from_flat(packet, sizeof(packet));
        if (buffer != nullptr) {
            notify_pending = true;
            // notify_custom无论成功与否都会消费mbuf。
            const int rc=ble_gatts_notify_custom(connection_handle, status_handle, buffer);
            rememberBle(rc,ErrorPoint::ble_notify);
            if (rc != 0) {
                notify_pending = false;
            }
        }
    }
    notifyDiag();
    (void)ble_npl_callout_reset(&status_timer, ble_npl_time_ms_to_ticks32(100));
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
            publishConnection(true);
        } else {
            (void)startAdvertising();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        if (event->disconnect.conn.conn_handle != connection_handle) { return 0; }
        connection_handle = BLE_HS_CONN_HANDLE_NONE;
        subscribed = false; diag_subscribed=false; mtu=23; diag_cursor={};
        notify_pending = false;
        publishConnection(false);
        (void)startAdvertising();
        return 0;

    case BLE_GAP_EVENT_MTU:
        mtu=event->mtu.value;
        diagnostics::bleState(connection_handle != BLE_HS_CONN_HANDLE_NONE,subscribed,diag_subscribed,mtu);
        return 0;
    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.conn_handle == connection_handle && event->subscribe.attr_handle == diag_handle) {
            diag_subscribed=event->subscribe.cur_notify != 0; diag_cursor={};
        }
        if (event->subscribe.conn_handle == connection_handle &&
            event->subscribe.attr_handle == status_handle) {
            subscribed = event->subscribe.cur_notify != 0;
        }
        diagnostics::bleState(connection_handle != BLE_HS_CONN_HANDLE_NONE,subscribed,diag_subscribed,mtu);
        return 0;
    case BLE_GAP_EVENT_NOTIFY_TX:
        if (event->notify_tx.conn_handle == connection_handle &&
            event->notify_tx.attr_handle == status_handle) {
            notify_pending = false;
        }
        return 0;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        (void)startAdvertising();
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
        const_cast<char *>(config::kBleDeviceName));
    advertising_fields.name_len = std::strlen(config::kBleDeviceName);
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
    diag_subscribed=false; diag_cursor={}; mtu=23;
    subscribed = false;
    notify_pending = false;
    ble_npl_callout_stop(&status_timer);
    publishConnection(false);
}

// onSync()取得本机地址类型后开始广播。
void onSync()
{
    ErrorInfo sync_error{};
    int rc = ble_hs_id_infer_auto(0, &own_address_type);
    if (rc) { VEHICLE_ERROR(&sync_error,ESP_FAIL,ble_address,nimble,rc); }
    else { rc=startAdvertising(&sync_error); }
    if (ready.load(std::memory_order_acquire) == 0) {
        if (rc != 0) { ready_error=sync_error; }
        ready.store(rc == 0 ? 1 : -1,std::memory_order_release);
    }
    if (rc) { diagnostics::bleError(sync_error); }
    (void)ble_npl_callout_reset(&status_timer, ble_npl_time_ms_to_ticks32(100));
}

// hostTask()运行NimBLE主机事件循环，退出后释放其FreeRTOS资源。
void hostTask(void *)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// 一个WRITE就是一条完整命令；严格长度解析拒绝尾随字符和嵌入NUL。
int characteristicAccess(std::uint16_t conn_handle,
                         std::uint16_t,
                         ble_gatt_access_ctxt *context,
                         void *)
{
    if (context == nullptr || context->om == nullptr) { return BLE_ATT_ERR_UNLIKELY; }
    const bool legacy = ble_uuid_cmp(context->chr->uuid, &command_uuid.u) == 0;
    if (context->op == BLE_GATT_ACCESS_OP_READ_CHR && legacy) {
        constexpr char unsupported[]="legacy control unsupported";
        return os_mbuf_append(context->om, unsupported, sizeof(unsupported)-1) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    if (legacy) { return BLE_ATT_ERR_WRITE_NOT_PERMITTED; }
    if (context->op != BLE_GATT_ACCESS_OP_WRITE_CHR || conn_handle != connection_handle) {
        return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
    }
    const std::uint16_t length = OS_MBUF_PKTLEN(context->om);
    if (length == 0 || length > config::kMaximumBleCommandLength) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    char incoming[config::kMaximumBleCommandLength]{};
    std::uint16_t copied = 0;
    if (ble_hs_mbuf_to_flat(context->om, incoming, length, &copied) != 0 || copied != length) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    RemoteCommand command{};
    if (!parseCommand(std::string_view(incoming, length), legacy, command)) {
        return BLE_ATT_ERR_VALUE_NOT_ALLOWED;
    }
    const std::int64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL(&state_lock);
    const bool accepted = acceptCommand(remote_state, command, now_us);
    portEXIT_CRITICAL(&state_lock);
    if (!accepted) { return BLE_ATT_ERR_VALUE_NOT_ALLOWED; }

    return 0;
}

} // namespace

esp_err_t initialize(ErrorInfo *error)
{
    if (initialized) {
        return ESP_OK;
    }

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

    characteristics[1].uuid = &command_v2_uuid.u;
    characteristics[1].access_cb = characteristicAccess;
    characteristics[1].flags = BLE_GATT_CHR_F_WRITE;

    characteristics[2].uuid = &status_uuid.u;
    characteristics[2].access_cb = statusAccess;
    characteristics[2].flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY;
    characteristics[2].val_handle = &status_handle;

    characteristics[3].uuid=&diag_uuid.u;
    characteristics[3].access_cb=diagAccess;
    characteristics[3].flags=BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY;
    characteristics[3].val_handle=&diag_handle;

    services[0].type = BLE_GATT_SVC_TYPE_PRIMARY;
    services[0].uuid = &service_uuid.u;
    services[0].characteristics = characteristics;

    int rc = ble_svc_gap_device_name_set(config::kBleDeviceName);
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

    rc=ble_npl_callout_init(&status_timer, nimble_port_get_dflt_eventq(), notifyStatus, nullptr);
    if (rc != 0) {
        return VEHICLE_ERROR(error, ESP_FAIL, ble_callout, nimble, rc);
    }
    ble_hs_cfg.reset_cb = onReset;
    ble_hs_cfg.sync_cb = onSync;

    nimble_port_freertos_init(hostTask);
    const auto deadline=esp_timer_get_time()+5000000;
    while (ready.load(std::memory_order_acquire) == 0 && esp_timer_get_time()<deadline) { vTaskDelay(pdMS_TO_TICKS(10)); }
    const int state=ready.load(std::memory_order_acquire);
    if (state == 0) { return VEHICLE_ERROR(error, ESP_ERR_TIMEOUT, ble_ready_timeout, application, 0); }
    if (state < 0) { if (error) { *error=ready_error; } return ESP_FAIL; }
    initialized=true;
    return ESP_OK;
}

void allowControl()
{
    portENTER_CRITICAL(&state_lock);
    remote_state.boot_complete=true;
    remote_state.run_allowed=!remote_state.fault && !remote_state.emergency;
    remote_state.pending=false;
    portEXIT_CRITICAL(&state_lock);
}

CommandSnapshot latestCommand()
{
    const std::int64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL(&state_lock);
    stepRemote(remote_state, now_us);
    const RemoteState state = remote_state;
    const std::uint32_t epoch = connection_epoch;
    portEXIT_CRITICAL(&state_lock);
    const bool active = state.mode == RemoteMode::active;
    const float steering_scale = state.command.legacy
        ? config::kMaximumSteeringVoltageV / config::kBleFullScaleSteering
        : config::kRemoteSteeringVoltageV / 100.0f;
    const float throttle_scale = state.command.legacy
        ? config::kMaximumThrottleVelocityRadS / config::kBleFullScaleThrottle
        : config::kMaximumThrottleVelocityRadS / 100.0f;
    return CommandSnapshot{
        active ? std::clamp(state.command.steering * steering_scale,
                           -config::kMaximumSteeringVoltageV, config::kMaximumSteeringVoltageV) : 0.0f,
        active ? std::clamp(state.command.throttle * throttle_scale,
                           -config::kMaximumThrottleVelocityRadS, config::kMaximumThrottleVelocityRadS) : 0.0f,
        state.applied_sequence, state.connected, state.mode,
        state.has_command ? now_us - state.received_us : -1,
        state.command.legacy, state.has_applied, epoch, state.stop_generation,
    };
}

void publishStatus(const StatusSnapshot &status)
{
    portENTER_CRITICAL(&state_lock);
    if (status.command.connection_epoch == connection_epoch) { latest_status = status; }
    portEXIT_CRITICAL(&state_lock);
}

void publishFault(std::uint8_t fault, bool emergency)
{
    portENTER_CRITICAL(&state_lock);
    remote_state.fault=!emergency;
    remote_state.emergency=remote_state.emergency || emergency;
    remote_state.run_allowed=false;
    latest_status = StatusSnapshot{};
    latest_status.command.mode = emergency ? RemoteMode::emergency : RemoteMode::fault;
    latest_status.command.connection_epoch = connection_epoch;
    latest_status.fault = fault;
    portEXIT_CRITICAL(&state_lock);
}

} // namespace ble
} // namespace vehicle
