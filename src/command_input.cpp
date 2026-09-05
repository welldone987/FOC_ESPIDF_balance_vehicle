#include "command_input.h"

#include "ble_idf_service.h"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

namespace vehicle {
namespace {

// command_queue_control保存静态Queue的控制块，生命周期覆盖整个命令输入域。
StaticQueue_t command_queue_control{};
// command_queue_storage提供一个RemoteCommand槽位的静态存储。
uint8_t command_queue_storage[sizeof(RemoteCommand)]{};
// command_queue指向命令输入域唯一的长度1最新值Queue。
QueueHandle_t command_queue{nullptr};

} // namespace

// beginCommandInput按静态存储创建命令Queue，并提交原生BLE初始化请求。
bool initializeCommandChannel() {
  // 已创建Queue时不重复初始化命令通道。
  if (command_queue != nullptr) {
    return false;
  }

  // xQueueCreateStatic()创建长度1、元素类型为RemoteCommand的Queue。
  command_queue = xQueueCreateStatic(1U,
                                     sizeof(RemoteCommand),
                                     command_queue_storage,
                                     &command_queue_control);
  if (command_queue == nullptr) {
    return false;
  }

  // initial_command为BLE回调发布的初始空命令快照。
  const RemoteCommand initial_command{};
  // xQueueOverwrite()把初始快照写入唯一Queue槽位。
  (void)xQueueOverwrite(command_queue, &initial_command);
  // 原生BLE服务复用同一Queue发布回调中的最新命令。
  return true;
}

bool beginCommandInput() {
  return command_queue != nullptr && beginBleIdfCommandService(command_queue) == ESP_OK;
}

// latestRemoteCommand以零等待读取命令Queue中的最近快照。
RemoteCommand latestRemoteCommand() {
  // command作为Queue尚未就绪或读取失败时的零值结果。
  RemoteCommand command{};
  if (command_queue != nullptr) {
    // xQueuePeek()复制最近快照并保留Queue中的数据。
    (void)xQueuePeek(command_queue, &command, 0U);
  }
  return command;
}

} // namespace vehicle
