#include "application_tasks.hpp"
#include "motion_command.hpp"
#include "diagnostics.hpp"
#include "motor_foc_service.hpp"
#include "power_config.hpp"
#include "power_monitor.hpp"
#include "wifi_telemtry.hpp"
#include "esp_core_dump.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

namespace {
StaticTask_t ble_storage{};
StackType_t ble_stack[vehicle::freertos_tasks::kBleStackBytes]{};
StaticQueue_t command_storage{}, ble_startup_storage{};
std::uint8_t command_buffer[sizeof(vehicle::control::MotionCommand)]{};
std::uint8_t ble_startup_buffer[sizeof(vehicle::freertos_tasks::BleStartup)]{};
StaticTask_t control_storage{};
StackType_t control_stack[vehicle::freertos_tasks::kControlStackBytes]{};
vehicle::freertos_tasks::TaskContext context{};
StaticQueue_t queue_storage{};
std::uint8_t queue_buffer[sizeof(vehicle::wifi_telemtry::TelemetrySnapshot)]{};
#if CONFIG_VEHICLE_WIFI_ENABLED
StaticTask_t wifi_storage{};
StackType_t wifi_stack[vehicle::freertos_tasks::kWifiStackBytes]{};
#endif
bool finish(vehicle::diagnostics::BootStep step, esp_err_t rc,
    const vehicle::ErrorInfo &error, bool required=true)
{
    if (rc == ESP_OK) { vehicle::diagnostics::boot(step,"OK"); return true; }
    if (!required) { vehicle::diagnostics::boot(step,"DEGRADED",rc); vehicle::diagnostics::record(error); return false; }
    vehicle::ErrorInfo secondary{};
    const auto off=vehicle::motor::inhibitOutputs(&secondary);
    vehicle::diagnostics::record(error,true);
    if (off != ESP_OK) { vehicle::diagnostics::record(secondary); }
    vehicle::motor::disableOutputs();
    vehicle::diagnostics::boot(step,"FAIL",rc);
    ESP_LOGE("boot","BOOT_SUMMARY FAIL point=%u raw=%ld at %s:%lu",static_cast<unsigned>(error.point_id),static_cast<long>(error.raw_code),error.file,static_cast<unsigned long>(error.line));
    return false;
}
} // namespace
extern "C" void app_main(void)
{
    using vehicle::diagnostics::BootStep;
    vehicle::ErrorInfo error{};
    // First hardware action establishes safe output. Static diagnostic storage needs no allocation.
    auto rc=vehicle::motor::inhibitOutputs(&error);
    vehicle::diagnostics::initialize();
    vehicle::diagnostics::boot(BootStep::safe_output,"BEGIN");
    if (!finish(BootStep::safe_output,rc,error)) { return; }
    vehicle::diagnostics::boot(BootStep::storage,"BEGIN");
    vehicle::diagnostics::boot(BootStep::storage,"OK");
    vehicle::diagnostics::boot(BootStep::power,"BEGIN");
    if (!finish(BootStep::power,vehicle::power::initialize(&error),error)) { return; }
    vehicle::diagnostics::boot(BootStep::voltage,"BEGIN");
    float voltage{};
    rc=vehicle::power::readBusVoltage(&voltage,&error);
    if (rc == ESP_OK && voltage <= vehicle::power::config::kStartupUndervoltageThresholdV) {
        rc=VEHICLE_ERROR(&error,ESP_ERR_INVALID_STATE,undervoltage,application,0,
            voltage,vehicle::power::config::kStartupUndervoltageThresholdV,-1,3,-1);
    }
    if (!finish(BootStep::voltage,rc,error)) { return; }
    vehicle::diagnostics::boot(BootStep::nvs,"BEGIN");
    rc=nvs_flash_init();
    if (rc != ESP_OK) { VEHICLE_ERROR(&error,rc,nvs,esp,rc); }
    if (!finish(BootStep::nvs,rc,error)) { return; }
    vehicle::diagnostics::boot(BootStep::core_dump,"BEGIN");
    size_t address{},size{};
    rc=esp_core_dump_image_get(&address,&size);
    if (rc == ESP_OK) { rc=esp_core_dump_image_check(); }
    if (rc == ESP_ERR_NOT_FOUND) {
        vehicle::diagnostics::boot(BootStep::core_dump,"SKIP",rc);
    } else {
        if (rc != ESP_OK) { VEHICLE_ERROR(&error,rc,core_dump,esp,rc); }
        finish(BootStep::core_dump,rc,error,false);
    }
    vehicle::diagnostics::boot(BootStep::ble,"BEGIN");
    context.command_queue=xQueueCreateStatic(1,sizeof(vehicle::control::MotionCommand),command_buffer,&command_storage);
    context.telemetry_queue=xQueueCreateStatic(1,sizeof(vehicle::wifi_telemtry::TelemetrySnapshot),queue_buffer,&queue_storage);
    context.ble_startup_queue=xQueueCreateStatic(1,sizeof(vehicle::freertos_tasks::BleStartup),ble_startup_buffer,&ble_startup_storage);
    if (!context.command_queue || !context.telemetry_queue || !context.ble_startup_queue ||
        !xTaskCreateStaticPinnedToCore(vehicle::freertos_tasks::bleTask,"BleTask",vehicle::freertos_tasks::kBleStackBytes,
            &context,vehicle::freertos_tasks::kBlePriority,ble_stack,&ble_storage,vehicle::freertos_tasks::kServiceCore)) {
        rc=VEHICLE_ERROR(&error,ESP_ERR_NO_MEM,boot_resource,application,0);
        finish(BootStep::ble,rc,error);
        return;
    }
    vehicle::freertos_tasks::BleStartup startup{};
    // BleTask完成首次广播后才放行控制初始化；结果也通过静态队列传递。
    if (xQueueReceive(context.ble_startup_queue,&startup,pdMS_TO_TICKS(6000)) != pdTRUE) {
        startup.result=VEHICLE_ERROR(&startup.error,ESP_ERR_TIMEOUT,ble_ready_timeout,application,0);
    }
    if (!finish(BootStep::ble,startup.result,startup.error)) { return; }
#if CONFIG_VEHICLE_WIFI_ENABLED
    vehicle::diagnostics::boot(BootStep::wifi,"BEGIN");
    rc=vehicle::wifi_telemtry::initialize();
    if (rc == ESP_OK) {
        const auto task=xTaskCreateStaticPinnedToCore(vehicle::freertos_tasks::wifiTelemetryTask,"WifiTelemetryTask",
            vehicle::freertos_tasks::kWifiStackBytes,&context,vehicle::freertos_tasks::kWifiPriority,wifi_stack,&wifi_storage,vehicle::freertos_tasks::kServiceCore);
        if (!task) { rc=ESP_ERR_NO_MEM; }
    }
    if (rc != ESP_OK) { VEHICLE_ERROR(&error,rc,wifi_init,esp,rc); }
    finish(BootStep::wifi,rc,error,false);
#else
    vehicle::diagnostics::boot(BootStep::wifi,"SKIP");
#endif
    const auto task=xTaskCreateStaticPinnedToCore(vehicle::freertos_tasks::controlTask,"ControlTask",
        vehicle::freertos_tasks::kControlStackBytes,&context,vehicle::freertos_tasks::kControlPriority,control_stack,&control_storage,vehicle::freertos_tasks::kControlCore);
    if (!task) {
        VEHICLE_ERROR(&error,ESP_ERR_NO_MEM,boot_resource,application,0);
        finish(BootStep::timer,ESP_ERR_NO_MEM,error);
        return;
    }
    vehicle::diagnostics::observeControlStart();
}
