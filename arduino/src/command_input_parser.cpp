#include "command_input.h"

#include "vehicle_config.h"

#include <cstdlib>
#include <cstring>

namespace vehicle {

// parseRemoteCommand在定长缓冲区中分隔字段并应用BLE缩放参数。
CommandParseResult parseRemoteCommand(const uint8_t *data, size_t length) {
  CommandParseResult result{};

  if (data == nullptr || length == 0U ||
      length > config::maximum_ble_command_length) {
    return result;
  }

  // buffer保存以空字符结尾的BLE命令副本。
  char buffer[config::maximum_ble_command_length + 1U]{};
  std::memcpy(buffer, data, length);
  // buffer[length]终止本次命令副本，供后续字符串解析读取。
  buffer[length] = '\0';

  // separator定位转向字段与油门字段之间的逗号。
  char *separator{std::strchr(buffer, ',')};
  if (separator == nullptr) {
    return result;
  }

  // 逗号把转向字段和油门字段拆成两个十进制前缀。
  *separator = '\0';
  // raw_steering保存命令中的转向原始整数。
  const long raw_steering{std::strtol(buffer, nullptr, 10)};
  // raw_throttle保存命令中的油门原始整数。
  const long raw_throttle{std::strtol(separator + 1, nullptr, 10)};

  // steering_voltage_v按maximum_steering_voltage_v和ble_full_scale_steering缩放，单位V。
  result.command.steering_voltage_v =
      config::maximum_steering_voltage_v *
      static_cast<float>(raw_steering) /
      config::ble_full_scale_steering;
  // throttle_velocity_rad_s按maximum_throttle_velocity_rad_s和ble_full_scale_throttle缩放，单位rad/s。
  result.command.throttle_velocity_rad_s =
      config::maximum_throttle_velocity_rad_s *
      static_cast<float>(raw_throttle) /
      config::ble_full_scale_throttle;
  // has_separator标记转向与油门字段已经完成拆分。
  result.has_separator = true;
  return result;
}

} // namespace vehicle
