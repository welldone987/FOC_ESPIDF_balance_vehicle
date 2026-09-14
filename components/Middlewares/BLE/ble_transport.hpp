#pragma once

#include <cstddef>
#include <cstdint>

#include "error_config.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "host/ble_gatt.h"

namespace vehicle {
namespace ble {
namespace transport {

/*
 * ble_transport拥有NimBLE主机、GATT服务表和连接状态。
 * 命令、遥测与诊断服务通过访问回调和生命周期钩子接入，
 * transport不反向包含任何服务头文件。
 */

// Incoming保存GATT回调复制的一条原始报文或连接状态变化。
struct Incoming {
    // text和length_bytes保存载荷内容与字节长度。
    char text[20]{};
    std::uint16_t length_bytes{};
    // epoch标识连接代次，连接重建后旧命令失效。
    std::uint32_t epoch{};
    // received_us保存GATT写入时刻，单位us。
    std::int64_t received_us{};
    // connected标记报文来自已建立的连接。
    bool connected{};
};

// AccessHandler与NimBLE的ble_gatt_chr_def.access_cb签名一致。
using AccessHandler = int (*)(std::uint16_t conn_handle, std::uint16_t attr_handle,
                              ble_gatt_access_ctxt *context, void *argument);

// AccessHandlers把命令与诊断特征的访问回调交给transport注册；遥测特征只提供通知。
struct AccessHandlers {
    AccessHandler command{};
    AccessHandler diagnostic{};
};

// LifecycleHook在NimBLE同步、复位或新连接时触发，返回NimBLE风格返回码。
using LifecycleHook = int (*)();

// LifecycleHooks由命令服务在Start()之前一次性注册。
struct LifecycleHooks {
    LifecycleHook synced{};
    LifecycleHook reset{};
    LifecycleHook connected{};
};

// Create()初始化NimBLE端口与GATT注册表，并保存原始报文队列。
esp_err_t Create(const AccessHandlers &handlers, QueueHandle_t incoming_queue,
                 ErrorInfo *error=nullptr);
// SetLifecycleHooks()必须在Start()之前调用。
void SetLifecycleHooks(const LifecycleHooks &hooks);
// Start()开始广播、启动主机任务并等待同步握手完成。
esp_err_t Start(ErrorInfo *error=nullptr);

// 连接状态访问器由NimBLE主机任务写入，供各服务只读使用。
std::uint32_t ConnectionEpoch();
std::uint16_t ConnectionHandle();
bool Subscribed();
// Notify()通过遥测特征发送一帧数据；mbuf所有权交给NimBLE。
int Notify(const std::uint8_t *data, std::size_t size);
// RememberBle()把NimBLE返回码记录为ErrorInfo事件。
void RememberBle(int rc, ErrorPoint point);

} // namespace transport
} // namespace ble
} // namespace vehicle
