#include "freertos_tasks.hpp"
#include "motion_command.hpp"
#include "diagnostics.hpp"
#include "motor_foc_service.hpp"
#include "power_config.hpp"
#include "power_monitor.hpp"
#include "wifi_telemetry.hpp"
#include "telemetry_snapshot.hpp"
#include "ble_config.hpp"
#include "esp_core_dump.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

namespace {
/*
 * app_main建立静态任务与队列，按固定顺序完成启动检查。
 * 必要步骤失败时禁用输出并锁存诊断；可选Wi-Fi失败只降级。
 * BleTask就绪后才创建ControlTask并进入低频观察。
 */
// command_storage等四组缓冲是命令队列与BLE启动结果的长度1静态队列。
StaticQueue_t command_storage{}, ble_startup_storage{};
std::uint8_t command_buffer[sizeof(vehicle::control::MotionCommand)]{};
std::uint8_t ble_startup_buffer[sizeof(vehicle::freertos_tasks::BleStartup)]{};
// context保存三个队列句柄，供任务创建时传入。
vehicle::freertos_tasks::TaskContext context{};
// queue_storage和queue_buffer是遥测快照的长度1静态队列。
StaticQueue_t queue_storage{};
std::uint8_t queue_buffer[sizeof(vehicle::control::TelemetrySnapshot)]{};
// Finish()在启动步骤失败时禁用输出、记录诊断并返回false。
bool Finish(vehicle::diagnostics::BootStep step, esp_err_t rc,
    const vehicle::ErrorInfo &error, bool required=true)
{
    if (rc == ESP_OK) { vehicle::diagnostics::Boot(step,"OK"); return true; }
    if (!required) { vehicle::diagnostics::Boot(step,"DEGRADED",rc); vehicle::diagnostics::Record(error); return false; }
    vehicle::ErrorInfo secondary{};
    const auto off=vehicle::motor::InhibitOutputs(&secondary);
    vehicle::diagnostics::Record(error,true);
    if (off != ESP_OK) { vehicle::diagnostics::Record(secondary); }
    vehicle::motor::DisableOutputs();
    vehicle::diagnostics::Boot(step,"FAIL",rc);
    ESP_LOGE("boot","BOOT_SUMMARY FAIL point=%u raw=%ld at %s:%lu",static_cast<unsigned>(error.point_id),static_cast<long>(error.raw_code),error.file,static_cast<unsigned long>(error.line));
    return false;
}
} // namespace
extern "C" void app_main(void)
{
    using vehicle::diagnostics::BootStep;
    vehicle::ErrorInfo error{};
    // 首个硬件操作先建立安全输出，静态诊断区无需动态分配。
    auto rc=vehicle::motor::InhibitOutputs(&error);
    vehicle::diagnostics::Initialize();
    vehicle::diagnostics::Boot(BootStep::safe_output,"BEGIN");
    if (!Finish(BootStep::safe_output,rc,error)) { return; }
    vehicle::diagnostics::Boot(BootStep::power,"BEGIN");
    if (!Finish(BootStep::power,vehicle::power::Initialize(&error),error)) { return; }
    vehicle::diagnostics::Boot(BootStep::voltage,"BEGIN");
    float voltage_V{};
    rc=vehicle::power::ReadBusVoltage(&voltage_V,&error);
    if (rc == ESP_OK && voltage_V <= vehicle::power::StartupUndervoltageThreshold_V) {
        rc=VEHICLE_ERROR(&error,ESP_ERR_INVALID_STATE,undervoltage,application,0,
            voltage_V,vehicle::power::StartupUndervoltageThreshold_V,-1,3,-1);
    }
    if (!Finish(BootStep::voltage,rc,error)) { return; }
    vehicle::diagnostics::Boot(BootStep::nvs,"BEGIN");
    rc=nvs_flash_init();
    if (rc != ESP_OK) { VEHICLE_ERROR(&error,rc,nvs,esp,rc); }
    if (!Finish(BootStep::nvs,rc,error)) { return; }
    vehicle::diagnostics::Boot(BootStep::core_dump,"BEGIN");
    size_t address{},size{};
    rc=esp_core_dump_image_get(&address,&size);
    if (rc == ESP_OK) { rc=esp_core_dump_image_check(); }
    if (rc == ESP_ERR_NOT_FOUND) {
        // 无历史dump时跳过检查，不阻塞启动。
        vehicle::diagnostics::Boot(BootStep::core_dump,"SKIP",rc);
    } else {
        if (rc != ESP_OK) { VEHICLE_ERROR(&error,rc,core_dump,esp,rc); }
        Finish(BootStep::core_dump,rc,error,false);
    }
    vehicle::diagnostics::Boot(BootStep::ble,"BEGIN");
    context.command_queue=xQueueCreateStatic(1,sizeof(vehicle::control::MotionCommand),command_buffer,&command_storage);
    context.telemetry_queue=xQueueCreateStatic(1,sizeof(vehicle::control::TelemetrySnapshot),queue_buffer,&queue_storage);
    context.ble_startup_queue=xQueueCreateStatic(1,sizeof(vehicle::freertos_tasks::BleStartup),ble_startup_buffer,&ble_startup_storage);
    if (!context.command_queue || !context.telemetry_queue || !context.ble_startup_queue ||
        !vehicle::freertos_tasks::CreateBleTask(context)) {
        rc=VEHICLE_ERROR(&error,ESP_ERR_NO_MEM,boot_resource,application,0);
        Finish(BootStep::ble,rc,error);
        return;
    }
    vehicle::freertos_tasks::BleStartup startup{};
    // BleTask完成首次广播后才放行控制初始化。
    // 初始化结果也通过静态队列传递。
    if (xQueueReceive(context.ble_startup_queue,&startup,pdMS_TO_TICKS(vehicle::ble::StartupWait_ms)) != pdTRUE) {
        startup.result=VEHICLE_ERROR(&startup.error,ESP_ERR_TIMEOUT,ble_ready_timeout,application,0);
    }
    if (!Finish(BootStep::ble,startup.result,startup.error)) { return; }

#if CONFIG_VEHICLE_WIFI_ENABLED
    vehicle::diagnostics::Boot(BootStep::wifi,"BEGIN");
    rc=vehicle::wifi_telemetry::Initialize();
    if (rc == ESP_OK) {
        const auto task=vehicle::freertos_tasks::CreateWifiTelemetryTask(context);
        if (!task) { rc=ESP_ERR_NO_MEM; }
    }
    if (rc != ESP_OK) { VEHICLE_ERROR(&error,rc,wifi_init,esp,rc); }
    Finish(BootStep::wifi,rc,error,false);
#else
    vehicle::diagnostics::Boot(BootStep::wifi,"SKIP");
#endif
    const auto task=vehicle::freertos_tasks::CreateControlTask(context);
    if (!task) {
        VEHICLE_ERROR(&error,ESP_ERR_NO_MEM,boot_resource,application,0);
        Finish(BootStep::timer,ESP_ERR_NO_MEM,error);
        return;
    }
    vehicle::diagnostics::ObserveControlStart();
}
