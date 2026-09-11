#include "application_tasks.hpp"
#include "ble_command_service.hpp"
#include "diagnostics.hpp"
#include "motor_foc_service.hpp"
#include "power_monitor.hpp"
#include "vehicle_config.hpp"
#include "wifi_telemtry.hpp"
#include "esp_core_dump.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

namespace {
using namespace vehicle;
StaticTask_t control_storage{};
StackType_t control_stack[freertos_tasks::kControlStackBytes]{};
freertos_tasks::TaskContext context{};
#if CONFIG_VEHICLE_WIFI_ENABLED
StaticQueue_t queue_storage{};
std::uint8_t queue_buffer[sizeof(wifi_telemtry::TelemetrySnapshot)]{};
StaticTask_t wifi_storage{};
StackType_t wifi_stack[freertos_tasks::kWifiStackBytes]{};
#endif
bool finish(diagnostics::BootStep step, esp_err_t rc, const ErrorInfo &error, bool required=true)
{
    if (rc == ESP_OK) { diagnostics::boot(step,"OK"); return true; }
    if (!required) { diagnostics::boot(step,"DEGRADED",rc); diagnostics::record(error); return false; }
    ErrorInfo secondary{};
    const auto off=motor::inhibitOutputs(&secondary);
    diagnostics::record(error,true);
    if (off != ESP_OK) { diagnostics::record(secondary); }
    ble::publishFault(1);
    motor::disableOutputs();
    diagnostics::boot(step,"FAIL",rc);
    ESP_LOGE("boot","BOOT_SUMMARY FAIL point=%u raw=%ld at %s:%lu",static_cast<unsigned>(error.point_id),static_cast<long>(error.raw_code),error.file,static_cast<unsigned long>(error.line));
    return false;
}
}
extern "C" void app_main(void)
{
    using diagnostics::BootStep;
    ErrorInfo error{};
    // First hardware action establishes safe output. Static diagnostic storage needs no allocation.
    auto rc=motor::inhibitOutputs(&error);
    diagnostics::initialize();
    diagnostics::boot(BootStep::safe_output,"BEGIN");
    if (!finish(BootStep::safe_output,rc,error)) { return; }
    diagnostics::boot(BootStep::storage,"BEGIN");
    diagnostics::boot(BootStep::storage,"OK");
    diagnostics::boot(BootStep::power,"BEGIN");
    if (!finish(BootStep::power,power::initialize(&error),error)) { return; }
    diagnostics::boot(BootStep::voltage,"BEGIN");
    float voltage{};
    rc=power::readBusVoltage(&voltage,&error);
    if (rc == ESP_OK && voltage <= config::kStartupUndervoltageThresholdV) {
        rc=VEHICLE_ERROR(&error,ESP_ERR_INVALID_STATE,undervoltage,application,0,
            voltage,config::kStartupUndervoltageThresholdV,-1,3,-1);
    }
    if (!finish(BootStep::voltage,rc,error)) { return; }
    diagnostics::boot(BootStep::nvs,"BEGIN");
    rc=nvs_flash_init();
    if (rc != ESP_OK) { VEHICLE_ERROR(&error,rc,nvs,esp,rc); }
    if (!finish(BootStep::nvs,rc,error)) { return; }
    diagnostics::boot(BootStep::core_dump,"BEGIN");
    size_t address{},size{};
    rc=esp_core_dump_image_get(&address,&size);
    if (rc == ESP_OK) { rc=esp_core_dump_image_check(); }
    if (rc == ESP_ERR_NOT_FOUND) {
        diagnostics::boot(BootStep::core_dump,"SKIP",rc);
    } else {
        if (rc != ESP_OK) { VEHICLE_ERROR(&error,rc,core_dump,esp,rc); }
        finish(BootStep::core_dump,rc,error,false);
    }
    diagnostics::boot(BootStep::ble,"BEGIN");
    if (!finish(BootStep::ble,ble::initialize(&error),error)) { return; }
#if CONFIG_VEHICLE_WIFI_ENABLED
    diagnostics::boot(BootStep::wifi,"BEGIN");
    rc=wifi_telemtry::initialize();
    if (rc == ESP_OK) {
        context.telemetry_queue=xQueueCreateStatic(1,sizeof(wifi_telemtry::TelemetrySnapshot),queue_buffer,&queue_storage);
        if (!context.telemetry_queue) { rc=ESP_ERR_NO_MEM; }
    }
    if (rc == ESP_OK) {
        const auto task=xTaskCreateStaticPinnedToCore(freertos_tasks::wifiTelemetryTask,"WifiTelemetryTask",
            freertos_tasks::kWifiStackBytes,&context,freertos_tasks::kWifiPriority,wifi_stack,&wifi_storage,freertos_tasks::kServiceCore);
        if (!task) { rc=ESP_ERR_NO_MEM; context.telemetry_queue=nullptr; }
    }
    if (rc != ESP_OK) { VEHICLE_ERROR(&error,rc,wifi_init,esp,rc); }
    finish(BootStep::wifi,rc,error,false);
#else
    diagnostics::boot(BootStep::wifi,"SKIP");
#endif
    const auto task=xTaskCreateStaticPinnedToCore(freertos_tasks::controlTask,"ControlTask",
        freertos_tasks::kControlStackBytes,&context,freertos_tasks::kControlPriority,control_stack,&control_storage,freertos_tasks::kControlCore);
    if (!task) {
        VEHICLE_ERROR(&error,ESP_ERR_NO_MEM,boot_resource,application,0);
        finish(BootStep::timer,ESP_ERR_NO_MEM,error);
        return;
    }
    diagnostics::observeControlStart();
}
