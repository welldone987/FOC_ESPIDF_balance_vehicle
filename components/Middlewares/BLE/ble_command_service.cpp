#include "ble_command_service.hpp"
#include "diagnostics.hpp"
#include "remote_protocol.hpp"
#include "motion_command.hpp"
#include <atomic>

#include <cstdint>
#include <cstring>
#include <string_view>

#include "esp_timer.h"
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

// 独立命令UUID避免旧网页在缺少ARM握手时误发运动目标。
#define BALANCE_UUID_BYTES(id) \
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, \
    0x93, 0xf3, 0xa3, 0xb5, id, 0x00, 0x40, 0x6e
ble_uuid128_t service_uuid = BLE_UUID128_INIT(BALANCE_UUID_BYTES(0x01));
ble_uuid128_t command_uuid = BLE_UUID128_INIT(BALANCE_UUID_BYTES(0x06));
ble_gatt_chr_def characteristics[2]{};
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
            publishConnection(true);
        } else {
            (void)startAdvertising();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        if (event->disconnect.conn.conn_handle != connection_handle) { return 0; }
        connection_handle = BLE_HS_CONN_HANDLE_NONE;
        publishConnection(false);
        (void)startAdvertising();
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
    if (context->op != BLE_GATT_ACCESS_OP_WRITE_CHR || conn_handle != connection_handle) {
        return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
    }
    Incoming input{};
    input.length=OS_MBUF_PKTLEN(context->om);
    if (input.length == 0 || input.length > sizeof(input.text)) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    std::uint16_t copied{};
    if (ble_hs_mbuf_to_flat(context->om,input.text,input.length,&copied) != 0 || copied != input.length) {
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

esp_err_t initialize(ErrorInfo *error)
{
    if (initialized) {
        return ESP_OK;
    }

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
    characteristics[0].flags = BLE_GATT_CHR_F_WRITE;

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
    std::uint16_t sequence{};
    bool have_sequence=false;
    for (;;) {
        Incoming input{};
        if (xQueueReceive(incoming_queue,&input,portMAX_DELAY) != pdTRUE) { continue; }
        if (input.epoch != epoch) {
            epoch=input.epoch;
            have_sequence=false;
            const control::MotionCommand zero{};
            xQueueOverwrite(command_queue,&zero);
        }
        if (!input.connected || input.length == 0) { continue; }
        RemoteCommand parsed{};
        if (!parseCommand(std::string_view(input.text,input.length),parsed) ||
            (have_sequence && !newerSequence(parsed.sequence,sequence))) { continue; }
        sequence=parsed.sequence;
        have_sequence=true;
        const control::MotionCommand command{
            parsed.throttle * config::kMaximumThrottleVelocityRadS / 100.0f,
            -parsed.steering * control::config::kYawRateLimitRadS / 100.0f,
            input.received_us,true};
        // 长度1最新值语义；过期判断使用接收时刻，不能因排队延长寿命。
        xQueueOverwrite(command_queue,&command);
    }
}

} // namespace ble
} // namespace vehicle
