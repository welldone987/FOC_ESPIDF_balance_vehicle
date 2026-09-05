/*
 * arduino_bt_startup对接Arduino-ESP32 2.0.17的启动期蓝牙资源保留钩子。
 * 原生BLE服务仍独立负责Controller和Bluedroid的初始化。
 */
#if defined(ARDUINO_ARCH_ESP32)
#include <sdkconfig.h>

#if CONFIG_IDF_TARGET_ESP32 && CONFIG_BT_ENABLED
// initArduino在setup之前调用此C链接强符号，保留后续BLE初始化所需内存。
extern "C" bool btInUse() {
  return true;
}
#endif
#endif
