#include "ble_command_service.hpp"

#include <cstdint>
#include <string_view>

#include "ble_config.hpp"
#include "ble_diagnostic_service.hpp"
#include "ble_telemetry_service.hpp"
#include "ble_transport.hpp"
#include "control_config.hpp"
#include "diagnostics.hpp"
#include "motion_command.hpp"
#include "remote_protocol.hpp"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "host/ble_att.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_hs_mbuf.h"
#include "os/os_mbuf.h"

namespace vehicle {
namespace ble {
namespace {

/*
 * ble_command_service负责BleTask的公开入口与命令路径。
 * 协议解析在Run()中完成，GATT回调只把报文复制到长度1队列。
 */
// incoming_queue是GATT回调与BleTask之间的长度1静态队列。
StaticQueue_t incoming_storage{};
std::uint8_t incoming_buffer[sizeof(transport::Incoming)]{};
QueueHandle_t incoming_queue{};
// initialized避免重复初始化。
bool initialized=false;

// OnCommandAccess()把命令特征的WRITE报文复制到长度1队列。
int OnCommandAccess(std::uint16_t conn_handle,
                    std::uint16_t,
                    ble_gatt_access_ctxt *context,
                    void *)
{
    if (context == nullptr || context->om == nullptr) { return BLE_ATT_ERR_UNLIKELY; }
    if (context->op != BLE_GATT_ACCESS_OP_WRITE_CHR || conn_handle != transport::ConnectionHandle()) {
        return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
    }
    transport::Incoming input{};
    input.length_bytes=OS_MBUF_PKTLEN(context->om);
    if (input.length_bytes == 0 || input.length_bytes > sizeof(input.text)) {
        transport::RememberBle(BLE_HS_ATT_ERR(BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN),ErrorPoint::ble_command);
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    std::uint16_t copied{};
    if (ble_hs_mbuf_to_flat(context->om,input.text,input.length_bytes,&copied) != 0 || copied != input.length_bytes) {
        transport::RememberBle(BLE_HS_ATT_ERR(BLE_ATT_ERR_UNLIKELY),ErrorPoint::ble_command);
        return BLE_ATT_ERR_UNLIKELY;
    }
    input.connected=true;
    input.epoch=transport::ConnectionEpoch();
    input.received_us=esp_timer_get_time();
    // GATT回调只复制原始报文，协议解析及量纲转换由BleTask执行。
    xQueueOverwrite(incoming_queue,&input);
    return 0;
}

} // namespace

esp_err_t Initialize(QueueHandle_t telemetry_queue, ErrorInfo *error)
{
    if (initialized) {
        return ESP_OK;
    }

    if (!telemetry_queue) { return VEHICLE_ERROR(error,ESP_ERR_INVALID_ARG,boot_resource,0); }
    incoming_queue=xQueueCreateStatic(1,sizeof(transport::Incoming),incoming_buffer,&incoming_storage);
    if (!incoming_queue) { return VEHICLE_ERROR(error,ESP_ERR_NO_MEM,boot_resource,0); }

    const transport::AccessHandlers handlers{&OnCommandAccess,&diagnostic::OnAccess};
    esp_err_t rc=transport::Create(handlers,incoming_queue,error);
    if (rc != ESP_OK) { return rc; }
    telemetry::Initialize(telemetry_queue);
    const transport::LifecycleHooks hooks{&telemetry::Start,&telemetry::Stop,&diagnostic::OnConnected};
    transport::SetLifecycleHooks(hooks);
    rc=transport::Start(error);
    if (rc != ESP_OK) { return rc; }
    initialized=true;
    return ESP_OK;
}

void Run(QueueHandle_t command_queue)
{
    // epoch跟踪连接代次，变化时清空运动目标。
    std::uint32_t epoch{};
    for (;;) {
        // 低优先级BleTask打印，避免串口吞吐占用NimBLE主机回调或ControlTask。
        diagnostic::ReportSerial(esp_timer_get_time());
        transport::Incoming input{};
        if (xQueueReceive(incoming_queue,&input,pdMS_TO_TICKS(100)) != pdTRUE) { continue; }
        if (input.epoch != epoch) {
            epoch=input.epoch;
            const control::MotionCommand zero{};
            xQueueOverwrite(command_queue,&zero);
        }
        if (!input.connected || input.length_bytes == 0) { continue; }
        // parsed保存解析后的X,Y百分比。
        RemoteCommand parsed{};
        if (!ParseCommand(std::string_view(input.text,input.length_bytes),parsed)) {
            ErrorInfo error{};
            VEHICLE_ERROR(&error,ESP_ERR_INVALID_ARG,ble_command,0,static_cast<float>(input.length_bytes),20,-1, ErrorValue | ErrorThreshold);
            diagnostics::Record(error);
            // 无效输入不能让旧非零目标继续有效。
            const control::MotionCommand zero{};
            xQueueOverwrite(command_queue,&zero);
            continue;
        }
        const control::MotionCommand command{
            parsed.throttle_percent * control::DriveSpeedLimit_rad_s / 100.0f,
            parsed.steering_percent * control::YawRateLimit_rad_s / 100.0f,
            input.received_us,true};
        // 长度1队列只保留最新值。
        // 过期判断使用接收时刻，不能因排队延长寿命。
        xQueueOverwrite(command_queue,&command);
    }
}

} // namespace ble
} // namespace vehicle
