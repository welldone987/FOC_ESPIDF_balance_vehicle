#pragma once

#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "host/ble_gatt.h"

namespace vehicle {
namespace ble {
namespace telemetry {

/*
 * ble_telemetry_service把控制任务发布的最新快照编码为固定20字节报文，
 * 通过遥测特征发送周期通知，并处理该特征的READ请求。
 */
// Initialize()保存遥测队列并创建notify定时器，必须在transport::Create()之后调用。
void Initialize(QueueHandle_t telemetry_queue);
// Start()/Stop()是transport生命周期钩子：启动或停止周期通知。
int Start();
int Stop();
// OnAccess()处理遥测特征的READ请求。
int OnAccess(std::uint16_t conn_handle, std::uint16_t attr_handle,
             ble_gatt_access_ctxt *context, void *argument);

} // namespace telemetry
} // namespace ble
} // namespace vehicle
