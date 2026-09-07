#include "ble_command_service.hpp"

#include <atomic>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "esp_log.h"
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

constexpr char kTag[] = "ble_command";

// Nordic-UART-compatible UUID byte order. These are exactly the UUIDs used by
// the reference Arduino BLE service.
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

bool initialized = false;
std::uint8_t own_address_type = BLE_OWN_ADDR_PUBLIC;

std::uint8_t readable_value[config::kMaximumBleCommandLength + 1U]{};
std::uint16_t readable_value_length = 0U;

void beginStateWrite()
{
    state_epoch.fetch_add(1U, std::memory_order_acq_rel);
}

void endStateWrite()
{
    state_epoch.fetch_add(1U, std::memory_order_release);
}

void publishConnection(bool is_connected)
{
    beginStateWrite();
    connected.store(is_connected, std::memory_order_relaxed);
    endStateWrite();
}

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

int gapEvent(ble_gap_event *event, void *)
{
    if (event == nullptr) {
        return 0;
    }

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            publishConnection(true);
            ESP_LOGI(kTag, "BLE client connected");
        } else {
            ESP_LOGW(kTag, "BLE connection failed, status=%d", event->connect.status);
            (void)startAdvertising();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        publishConnection(false);
        ESP_LOGI(kTag, "BLE client disconnected; restarting advertising");
        (void)startAdvertising();
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        (void)startAdvertising();
        return 0;

    default:
        return 0;
    }
}

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
        ESP_LOGE(kTag, "ble_gap_adv_set_fields failed: %d", rc);
        return rc;
    }

    ble_hs_adv_fields scan_response_fields{};
    scan_response_fields.uuids128 = &service_uuid;
    scan_response_fields.num_uuids128 = 1U;
    scan_response_fields.uuids128_is_complete = 1U;

    rc = ble_gap_adv_rsp_set_fields(&scan_response_fields);
    if (rc != 0) {
        ESP_LOGE(kTag, "ble_gap_adv_rsp_set_fields failed: %d", rc);
        return rc;
    }

    ble_gap_adv_params parameters{};
    parameters.conn_mode = BLE_GAP_CONN_MODE_UND;
    parameters.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(
        own_address_type, nullptr, BLE_HS_FOREVER, &parameters, gapEvent, nullptr);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(kTag, "ble_gap_adv_start failed: %d", rc);
    }
    return rc == BLE_HS_EALREADY ? 0 : rc;
}

void onReset(int reason)
{
    ESP_LOGW(kTag, "NimBLE host reset, reason=%d", reason);
}

void onSync()
{
    const int rc = ble_hs_id_infer_auto(0, &own_address_type);
    if (rc != 0) {
        ESP_LOGE(kTag, "ble_hs_id_infer_auto failed: %d", rc);
        return;
    }

    if (startAdvertising() == 0) {
        ESP_LOGI(kTag, "BLE advertising started as '%s'", config::kBleDeviceName);
    }
}

void hostTask(void *)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

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

    // Keep the characteristic readable value aligned with the most recent write,
    // as Arduino BLECharacteristic does after a client write.
    if (copied > 0U) {
        std::memcpy(readable_value, incoming, copied);
    }
    readable_value_length = copied;

    // The reference ignores an empty write.
    if (copied == 0U) {
        return 0;
    }

    incoming[copied] = '\0';
    char *separator = std::strchr(reinterpret_cast<char *>(incoming), ',');

    // The reference explicitly zeros both commands when a non-empty write has
    // no comma separator.
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

    // NimBLE examples initialize NVS before starting the controller. Do not erase
    // NVS automatically on failure: persistence must never be destroyed implicitly.
    const esp_err_t nvs_result = nvs_flash_init();
    if (nvs_result != ESP_OK) {
        ESP_LOGE(kTag, "nvs_flash_init failed: %s", esp_err_to_name(nvs_result));
        return nvs_result;
    }

    const esp_err_t nimble_result = nimble_port_init();
    if (nimble_result != ESP_OK) {
        ESP_LOGE(kTag, "nimble_port_init failed: %s", esp_err_to_name(nimble_result));
        return nimble_result;
    }

    ble_svc_gap_init();
    ble_svc_gatt_init();
    buildGattDatabase();

    int rc = ble_svc_gap_device_name_set(config::kBleDeviceName);
    if (rc != 0) {
        ESP_LOGE(kTag, "ble_svc_gap_device_name_set failed: %d", rc);
        return ESP_FAIL;
    }

    rc = ble_gatts_count_cfg(services);
    if (rc != 0) {
        ESP_LOGE(kTag, "ble_gatts_count_cfg failed: %d", rc);
        return ESP_FAIL;
    }

    rc = ble_gatts_add_svcs(services);
    if (rc != 0) {
        ESP_LOGE(kTag, "ble_gatts_add_svcs failed: %d", rc);
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

    ESP_LOGI(kTag,
             "Native NimBLE command service initialized (%s / %s)",
             config::kBleServiceUuid,
             config::kBleCommandUuid);
    return ESP_OK;
}

CommandSnapshot latestCommand()
{
    for (;;) {
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
