#include "ble_command_service.hpp"

#include <atomic>
#include <cstdint>
#include <cstdlib>
#include <cstring>

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

/*
 * BLE回调负责协议解析和连接状态发布，控制任务只读取CommandSnapshot。
 * state_epoch用序列锁保护多原子字段的一致快照，readable_value保留最近一次特征写入。
 */
// UUID宏按NimBLE字节序表达网页端使用的服务和命令特征。
#define BALANCE_SERVICE_UUID_BYTES \
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, \
    0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e
#define BALANCE_COMMAND_UUID_BYTES \
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, \
    0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e

ble_uuid128_t service_uuid = BLE_UUID128_INIT(BALANCE_SERVICE_UUID_BYTES);
ble_uuid128_t command_uuid = BLE_UUID128_INIT(BALANCE_COMMAND_UUID_BYTES);

ble_gatt_chr_def characteristics[2]{};
ble_gatt_svc_def services[2]{};

std::atomic<std::uint32_t> state_epoch{0U};
std::atomic<std::int32_t> raw_steering{0};
std::atomic<std::int32_t> raw_throttle{0};
std::atomic<std::uint32_t> command_sequence{0U};
std::atomic<bool> connected{false};

// initialized和own_address_type保存NimBLE主机初始化后的服务状态。
bool initialized = false;
std::uint8_t own_address_type = BLE_OWN_ADDR_PUBLIC;

// readable_value保存网页端读取特征时应返回的最近一次写入内容。
std::uint8_t readable_value[config::kMaximumBleCommandLength + 1U]{};
std::uint16_t readable_value_length = 0U;

// beginStateWrite()把快照版本置为写入态，使读取方跳过中间状态。
void beginStateWrite()
{
    state_epoch.fetch_add(1U, std::memory_order_acq_rel);
}

// endStateWrite()发布偶数版本，表示本次命令或连接状态写入完成。
void endStateWrite()
{
    state_epoch.fetch_add(1U, std::memory_order_release);
}

// publishConnection()原子发布BLE连接状态。
void publishConnection(bool is_connected)
{
    beginStateWrite();
    connected.store(is_connected, std::memory_order_relaxed);
    endStateWrite();
}

// publishCommand()原子发布协议中的原始整数，并按需递增命令序号。
void publishCommand(std::int32_t steering_value,
                    std::int32_t throttle_value,
                    bool increment_sequence)
{
    beginStateWrite();
    raw_steering.store(steering_value, std::memory_order_relaxed);
    raw_throttle.store(throttle_value, std::memory_order_relaxed);
    if (increment_sequence) {
        command_sequence.store(
            command_sequence.load(std::memory_order_relaxed) + 1U,
            std::memory_order_relaxed);
    }
    endStateWrite();
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
            publishConnection(true);
        } else {
            (void)startAdvertising();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
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
void onReset(int) {}

// onSync()取得本机地址类型后开始广播。
void onSync()
{
    const int rc = ble_hs_id_infer_auto(0, &own_address_type);
    if (rc != 0) {
        return;
    }

    (void)startAdvertising();
}

// hostTask()运行NimBLE主机事件循环，退出后释放其FreeRTOS资源。
void hostTask(void *)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// characteristicAccess()保持特征可读，并把写入的“转向,油门”解析为命令快照。
int characteristicAccess(std::uint16_t,
                         std::uint16_t,
                         ble_gatt_access_ctxt *context,
                         void *)
{
    if (context == nullptr || context->om == nullptr) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    if (context->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        return os_mbuf_append(
                   context->om, readable_value, readable_value_length) == 0
                   ? 0
                   : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    if (context->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    const std::uint16_t packet_length = OS_MBUF_PKTLEN(context->om);
    if (packet_length > config::kMaximumBleCommandLength) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    std::uint8_t incoming[config::kMaximumBleCommandLength + 1U]{};
    std::uint16_t copied = 0U;
    const int flatten_result = ble_hs_mbuf_to_flat(
        context->om, incoming, packet_length, &copied);
    if (flatten_result != 0 || copied != packet_length) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    // 保持可读特征与最近一次写入一致，兼容Arduino BLECharacteristic行为。
    if (copied > 0U) {
        std::memcpy(readable_value, incoming, copied);
    }
    readable_value_length = copied;

    // 空写入只更新可读值，不改变控制命令。
    if (copied == 0U) {
        return 0;
    }

    incoming[copied] = '\0';
    char *separator = std::strchr(reinterpret_cast<char *>(incoming), ',');

    // 非空写入缺少逗号时清零两路原始命令，保持参考实现语义。
    if (separator == nullptr) {
        publishCommand(0, 0, false);
        return 0;
    }

    *separator = '\0';
    const long steering_value = std::strtol(
        reinterpret_cast<char *>(incoming), nullptr, 10);
    const long throttle_value = std::strtol(separator + 1, nullptr, 10);

    publishCommand(
        static_cast<std::int32_t>(steering_value),
        static_cast<std::int32_t>(throttle_value),
        true);
    return 0;
}

// buildGattDatabase()建立一个可读写的命令特征和主服务定义。
void buildGattDatabase()
{
    characteristics[0].uuid = &command_uuid.u;
    characteristics[0].access_cb = characteristicAccess;
    characteristics[0].flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE;

    services[0].type = BLE_GATT_SVC_TYPE_PRIMARY;
    services[0].uuid = &service_uuid.u;
    services[0].characteristics = characteristics;
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
    buildGattDatabase();

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

    ble_hs_cfg.reset_cb = onReset;
    ble_hs_cfg.sync_cb = onSync;

    initialized = true;
    nimble_port_freertos_init(hostTask);

    return ESP_OK;
}

CommandSnapshot latestCommand()
{
    for (;;) {
        // 奇数版本表示BLE回调正在写入，读取方等待偶数版本后再采样字段。
        const std::uint32_t before = state_epoch.load(std::memory_order_acquire);
        if ((before & 1U) != 0U) {
            continue;
        }

        const std::int32_t steering_value =
            raw_steering.load(std::memory_order_relaxed);
        const std::int32_t throttle_value =
            raw_throttle.load(std::memory_order_relaxed);
        const std::uint32_t sequence =
            command_sequence.load(std::memory_order_relaxed);
        const bool is_connected = connected.load(std::memory_order_relaxed);

        const std::uint32_t after = state_epoch.load(std::memory_order_acquire);
        if (before != after) {
            continue;
        }

        // 原始整数按网页协议的满量程换算为控制器使用的物理量。
        return CommandSnapshot{
            config::kMaximumSteeringVoltageV *
                static_cast<float>(steering_value) /
                config::kBleFullScaleSteering,
            config::kMaximumThrottleVelocityRadS *
                static_cast<float>(throttle_value) /
                config::kBleFullScaleThrottle,
            sequence,
            is_connected,
        };
    }
}

bool isInitialized()
{
    return initialized;
}

} // namespace ble
} // namespace vehicle
