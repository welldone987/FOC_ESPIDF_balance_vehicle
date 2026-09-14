#include "freertos_tasks.hpp"
#include "diagnostics.hpp"
#include "motor_foc_service.hpp"
#include "power_monitor.hpp"
#include "esp_log.h"
#include "nvs_flash.h"

namespace {
/*
 * app_main按启动顺序调用各模块初始化入口，并保留必要的失败策略。
 * 必要步骤失败时禁用输出并锁存诊断；Wi-Fi失败只降级。
 * BleTask就绪后才创建ControlTask并进入低频观察。
 */
// context保存StartBle()创建的三组长度1静态队列句柄，供全部任务共享。
vehicle::freertos_tasks::TaskContext context{};
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
    if (!Finish(BootStep::voltage,vehicle::power::CheckStartupVoltage(&error),error)) { return; }
    vehicle::diagnostics::Boot(BootStep::nvs,"BEGIN");
    rc=nvs_flash_init();
    if (rc != ESP_OK) { VEHICLE_ERROR(&error,rc,nvs,rc); }
    if (!Finish(BootStep::nvs,rc,error)) { return; }
    vehicle::diagnostics::Boot(BootStep::ble,"BEGIN");
    if (!Finish(BootStep::ble,vehicle::freertos_tasks::StartBle(context,&error),error)) { return; }

    vehicle::diagnostics::Boot(BootStep::wifi,"BEGIN");
    Finish(BootStep::wifi,vehicle::freertos_tasks::StartWifiTelemetry(context,&error),error,false);
    const auto task=vehicle::freertos_tasks::CreateControlTask(context);
    if (!task) {
        VEHICLE_ERROR(&error,ESP_ERR_NO_MEM,boot_resource,0);
        Finish(BootStep::timer,ESP_ERR_NO_MEM,error);
        return;
    }
    vehicle::diagnostics::ObserveControlStart();
}
