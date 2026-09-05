/*
 * command_input提供命令解析、原生BLE启动和最新RemoteCommand快照。
 * parseRemoteCommand把“steering,throttle”字节序列缩放为物理量。
 * beginCommandInput建立静态最新值通道并提交BLE初始化请求，latestRemoteCommand读取一致快照。
 */
#pragma once

#include "app_types.h"

#include <stddef.h>

namespace vehicle {

// CommandParseResult保存解析后的RemoteCommand和字段分隔状态。
struct CommandParseResult {
  // command保存协议字段缩放后的RemoteCommand。
  RemoteCommand command;
  // has_separator标记命令是否包含转向与油门字段分隔符。
  bool has_separator;
};

// parseRemoteCommand把BLE字节序列转换为RemoteCommand和分隔符状态。
CommandParseResult parseRemoteCommand(const uint8_t *data, size_t length);
// beginCommandInput建立静态命令Queue并提交原生BLE初始化请求，返回调用结果。
bool initializeCommandChannel();
bool beginCommandInput();
// latestRemoteCommand返回控制运行时使用的最新命令快照。
RemoteCommand latestRemoteCommand();

} // namespace vehicle
