#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

namespace vehicle {
namespace ble {
namespace telemetry {

/*
 * ble_telemetry_service把控制任务发布的最新快照编码为固定20字节报文，
 * 通过遥测特征的周期通知发送。
 */
// Initialize()保存遥测队列并创建notify定时器，必须在transport::Create()之后调用。
void Initialize(QueueHandle_t telemetry_queue);
// Start()/Stop()是transport生命周期钩子：启动或停止周期通知。
int Start();
int Stop();

} // namespace telemetry
} // namespace ble
} // namespace vehicle
