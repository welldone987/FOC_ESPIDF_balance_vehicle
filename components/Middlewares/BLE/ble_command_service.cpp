#include "ble_command_service.hpp"

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
ble_gatt_chr_def characteristics[4]{};
ble_gatt_svc_def services[2]{};

// 临界区只复制/更新固定长度状态，不在锁内格式化、分配内存或调用NimBLE。
portMUX_TYPE state_lock = portMUX_INITIALIZER_UNLOCKED;
RemoteState remote_state{};
StatusSnapshot latest_status{};
std::uint32_t connection_epoch{};
// 以下连接/订阅/通知资源仅由NimBLE主机访问。
ble_npl_callout status_timer{};
std::uint16_t status_handle{};
bool subscribed{};
bool notify_pending{};
bool initialized = false;
std::uint8_t own_address_type = BLE_OWN_ADDR_PUBLIC;
std::uint16_t connection_handle = BLE_HS_CONN_HANDLE_NONE;
std::uint8_t readable_value[config::kMaximumBleCommandLength + 1U]{};
std::uint16_t readable_value_length = 0U;

void publishConnection(bool connected)
{
    portENTER_CRITICAL(&state_lock);
    remoteConnection(remote_state, connected);
    ++connection_epoch;
    portEXIT_CRITICAL(&state_lock);
}

// 手动编码小端字段，不把C++结构体内存布局当作无线协议。
void put16(std::uint8_t *packet, std::size_t offset, std::uint16_t value)
{
    packet[offset] = static_cast<std::uint8_t>(value);
    packet[offset + 1] = static_cast<std::uint8_t>(value >> 8);
}

// 按IEEE-754位模式检查，避免fast-math下isfinite被消除及NaN转整数。
constexpr bool finiteTelemetry(float value)
{
    return (std::bit_cast<std::uint32_t>(value) & 0x7f800000U) != 0x7f800000U;
}
static_assert(!finiteTelemetry(std::bit_cast<float>(0x7fc00000U)));
static_assert(!finiteTelemetry(std::bit_cast<float>(0x7f800000U)));
static_assert(finiteTelemetry(-12.34f));

std::int16_t scaled16(float value, float scale)
{
    return static_cast<std::int16_t>(std::clamp(value * scale, -32768.0f, 32767.0f));
}

void encodeStatus(std::uint8_t (&packet)[20])
{
    const std::int64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL(&state_lock);
    const StatusSnapshot status = latest_status;
    const RemoteState remote = remote_state;
    const std::uint32_t epoch = connection_epoch;
    portEXIT_CRITICAL(&state_lock);
    const bool same_session = status.command.connection_epoch == epoch;
    const bool stale = status.sampled_us == 0 || now_us - status.sampled_us >= kCommandTimeoutUs;
    const bool valid = status.sensors_valid && !stale && same_session &&
        finiteTelemetry(status.pitch_deg) && finiteTelemetry(status.left_velocity_rad_s) &&
        finiteTelemetry(status.right_velocity_rad_s) &&
        finiteTelemetry(status.command.throttle_velocity_rad_s) &&
        finiteTelemetry(status.command.steering_voltage_v);
    RemoteMode mode = status.command.mode;
    if (mode != RemoteMode::fault && mode != RemoteMode::emergency && !same_session) {
        mode = remote.connected ? RemoteMode::idle : RemoteMode::disconnected;
    }
    packet[0] = 2;
    packet[1] = static_cast<std::uint8_t>(mode);
    packet[2] = (remote.connected ? 1U : 0U) | (remote.command.legacy ? 2U : 0U) |
                (valid ? 4U : 0U) | (same_session && status.command.sequence_valid ? 8U : 0U) |
                (stale ? 16U : 0U);
    packet[3] = status.fault;
    put16(packet, 4, same_session ? status.command.sequence : 0);
    const auto age_ms = remote.has_command ? (now_us - remote.received_us) / 1000 : 65535;
    put16(packet, 6, static_cast<std::uint16_t>(std::clamp<std::int64_t>(age_ms, 0, 65535)));
    put16(packet, 8, valid ? static_cast<std::uint16_t>(scaled16(status.pitch_deg, 100.0f)) : 0);
    put16(packet, 10, valid ? static_cast<std::uint16_t>(scaled16(status.left_velocity_rad_s, 100.0f)) : 0);
    put16(packet, 12, valid ? static_cast<std::uint16_t>(scaled16(status.right_velocity_rad_s, 100.0f)) : 0);
    put16(packet, 14, valid ? static_cast<std::uint16_t>(scaled16(status.command.throttle_velocity_rad_s, 100.0f)) : 0);
    put16(packet, 16, valid ? static_cast<std::uint16_t>(scaled16(status.command.steering_voltage_v, 1000.0f)) : 0);
    put16(packet, 18, static_cast<std::uint16_t>(status.sample_sequence));
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
            if (ble_gatts_notify_custom(connection_handle, status_handle, buffer) != 0) {
                notify_pending = false;
            }
        }
    }
    (void)ble_npl_callout_reset(&status_timer, ble_npl_time_ms_to_ticks32(100));
}

int startAdvertising();

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
        subscribed = false;
        notify_pending = false;
        publishConnection(false);
        (void)startAdvertising();
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.conn_handle == connection_handle &&
            event->subscribe.attr_handle == status_handle) {
            subscribed = event->subscribe.cur_notify != 0;
        }
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
int startAdvertising()
{
    ble_hs_adv_fields advertising_fields{};
    advertising_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    advertising_fields.name = reinterpret_cast<std::uint8_t *>(
        const_cast<char *>(config::kBleDeviceName));
    advertising_fields.name_len = std::strlen(config::kBleDeviceName);
    advertising_fields.name_is_complete = 1U;

    int rc = ble_gap_adv_set_fields(&advertising_fields);
    if (rc != 0) {
        return rc;
    }

    ble_hs_adv_fields scan_response_fields{};
    scan_response_fields.uuids128 = &service_uuid;
    scan_response_fields.num_uuids128 = 1U;
    scan_response_fields.uuids128_is_complete = 1U;

    rc = ble_gap_adv_rsp_set_fields(&scan_response_fields);
    if (rc != 0) {
        return rc;
    }

    ble_gap_adv_params parameters{};
    parameters.conn_mode = BLE_GAP_CONN_MODE_UND;
    parameters.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(
        own_address_type, nullptr, BLE_HS_FOREVER, &parameters, gapEvent, nullptr);
    return rc == BLE_HS_EALREADY ? 0 : rc;
}

// onReset()满足NimBLE复位回调接口；复位信息由上层诊断路径处理。
void onReset(int)
{
    connection_handle = BLE_HS_CONN_HANDLE_NONE;
    subscribed = false;
    notify_pending = false;
    ble_npl_callout_stop(&status_timer);
    publishConnection(false);
}

// onSync()取得本机地址类型后开始广播。
void onSync()
{
    const int rc = ble_hs_id_infer_auto(0, &own_address_type);
    if (rc != 0) {
        return;
    }

    (void)startAdvertising();
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
        return os_mbuf_append(context->om, readable_value, readable_value_length) == 0
                   ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
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
    if (legacy) {
        std::memcpy(readable_value, incoming, length);
        readable_value_length = length;
    }
    return 0;
}

} // namespace

esp_err_t initialize()
{
    if (initialized) {
        return ESP_OK;
    }

    // 先初始化NVS再启动NimBLE控制器；失败时不自动擦除NVS持久化数据。
    const esp_err_t nvs_result = nvs_flash_init();
    if (nvs_result != ESP_OK) {
        return nvs_result;
    }

    const esp_err_t nimble_result = nimble_port_init();
    if (nimble_result != ESP_OK) {
        return nimble_result;
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

    services[0].type = BLE_GATT_SVC_TYPE_PRIMARY;
    services[0].uuid = &service_uuid.u;
    services[0].characteristics = characteristics;

    int rc = ble_svc_gap_device_name_set(config::kBleDeviceName);
    if (rc != 0) {
        return ESP_FAIL;
    }

    rc = ble_gatts_count_cfg(services);
    if (rc != 0) {
        return ESP_FAIL;
    }

    rc = ble_gatts_add_svcs(services);
    if (rc != 0) {
        return ESP_FAIL;
    }

    std::memcpy(
        readable_value,
        config::kBleInitialValue,
        sizeof(config::kBleInitialValue) - 1U);
    readable_value_length = sizeof(config::kBleInitialValue) - 1U;

    if (ble_npl_callout_init(&status_timer, nimble_port_get_dflt_eventq(), notifyStatus, nullptr) != 0) {
        return ESP_FAIL;
    }
    ble_hs_cfg.reset_cb = onReset;
    ble_hs_cfg.sync_cb = onSync;

    initialized = true;
    nimble_port_freertos_init(hostTask);

    return ESP_OK;
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
        state.command.legacy, state.has_applied, epoch,
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
    latest_status = StatusSnapshot{};
    latest_status.command.mode = emergency ? RemoteMode::emergency : RemoteMode::fault;
    latest_status.command.connection_epoch = connection_epoch;
    latest_status.fault = fault;
    portEXIT_CRITICAL(&state_lock);
}

} // namespace ble
} // namespace vehicle
