#include "application_tasks.hpp"

#include "balance_controller.hpp"
#include "ble_command_service.hpp"
#include "bmi160_attitude.hpp"
#include "diagnostics.hpp"
#include "motor_foc_service.hpp"
#include "vehicle_config.hpp"
#include "wifi_telemtry.hpp"

#include "esp_timer.h"

namespace vehicle {
namespace freertos_tasks {
namespace {

/*
 * 任务实现把控制释放、网络遥测和低频诊断分开调度。
 * 控制任务使用定时器通知建立固定释放节拍，遥测和诊断任务使用vTaskDelayUntil保持周期。
 */

constexpr std::uint32_t kWifiPeriodMs = 50U;
constexpr std::uint32_t kDiagnosticsPeriodMs = 5000U;

// stopControl()关闭电机输出后挂起控制任务，避免故障状态继续驱动执行器。
[[noreturn]] void stopControl(std::uint8_t fault = 1U, bool emergency = false)
{
    motor::disableOutputs();
    ble::publishFault(fault, emergency);
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000U));
    }
}

// releaseControl()由ESP定时器回调通知控制任务开始下一周期。
void releaseControl(void *task_handle)
{
    xTaskNotifyGive(static_cast<TaskHandle_t>(task_handle));
}

} // namespace

void controlTask(void *argument)
{
    // context由app_main提供，在整个静态任务生命周期内保持有效。
    auto &context = *static_cast<TaskContext *>(argument);
    // controller保存PID和低通滤波器的跨周期状态。
    control::ControllerState controller{};
    control::initialize(controller);

    esp_err_t result = imu::initialize();
    diagnostics::logInitialization("BMI160", result);
    if (result != ESP_OK) {
        stopControl();
    }

    result = motor::initialize();
    diagnostics::logInitialization("Motor", result);
    if (result != ESP_OK) {
        stopControl();
    }
    imu::resetEstimator();

    // 控制定时器使用ESP_TIMER_TASK回调，回调只发送通知，不执行控制计算。
    esp_timer_create_args_t timer_config{};
    timer_config.callback = releaseControl;
    timer_config.arg = xTaskGetCurrentTaskHandle();
    timer_config.dispatch_method = ESP_TIMER_TASK;
    timer_config.name = "control_release";

    esp_timer_handle_t timer = nullptr;
    result = esp_timer_create(&timer_config, &timer);
    if (result == ESP_OK) {
        result = esp_timer_start_periodic(timer, config::kControlPeriodUs);
    }
    diagnostics::logInitialization("Control timer", result);
    if (result != ESP_OK) {
        stopControl(3U);
    }

    std::uint32_t sequence = 0U;
    for (;;) {
        // 每次通知对应一个控制释放点；控制计算在本任务上下文中顺序执行。
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // cycle_time_us成为遥测帧的设备时间戳，单位us。
        const std::int64_t cycle_time_us = esp_timer_get_time();
        // 急停在消费上周期电机目标之前检查；无线与调度延迟仍需实测。
        const ble::CommandSnapshot command = ble::latestCommand();
        if (command.mode == ble::RemoteMode::emergency) { stopControl(0U, true); }
        if (command.mode != ble::RemoteMode::active) {
            controller.steering_filter.previous_output = 0.0f;
        }
        // runFocAndReadWheelState()先消费上一周期stageTarget()写入的目标。
        const motor::WheelState wheels = motor::runFocAndReadWheelState();
        // 姿态读取紧跟轮速读取，结果随后进入同一轮控制计算。
        const imu::AttitudeSample attitude = imu::readAttitude();
        if (!wheels.valid || !attitude.valid) {
            stopControl(2U);
        }

        // BLE只提供最近命令快照，不在高频控制任务中执行通信等待。
        const control::ControlOutput output = control::update(
            controller,
            control::ControlInput{
                wheels.left_velocity_rad_s,
                wheels.right_velocity_rad_s,
                attitude.pitch_deg,
                command.steering_voltage_v,
                command.throttle_velocity_rad_s,
            });
        // 本轮控制输出暂存为下一轮FOC move()使用的左右目标电压。
        motor::stageTarget(
            motor::VoltageCommand{output.left_target_v, output.right_target_v});

        // sequence递增后，遥测任务可识别新的控制快照。
        ++sequence;
        ble::publishStatus(ble::StatusSnapshot{
            command, cycle_time_us, sequence, attitude.pitch_deg,
            config::kMotor0Direction * wheels.left_velocity_rad_s,
            config::kMotor1Direction * wheels.right_velocity_rad_s, 0U, true,
        });
        const wifi_telemtry::TelemetrySnapshot snapshot{
            cycle_time_us,
            sequence,
            attitude.pitch_deg,
            wheels.left_velocity_rad_s,
            wheels.right_velocity_rad_s,
            output.left_target_v,
            output.right_target_v,
        };
        // 只保留最近一帧遥测，避免网络服务拖慢控制任务。
        xQueueOverwrite(context.telemetry_queue, &snapshot);
    }
}

void wifiTelemetryTask(void *argument)
{
    // Wi-Fi服务运行在低频服务任务中，与控制任务共享最新值队列。
    auto &context = *static_cast<TaskContext *>(argument);
    const esp_err_t result = wifi_telemtry::initialize();
    diagnostics::logInitialization("Wi-Fi telemetry", result);
    if (result != ESP_OK) {
        vTaskSuspend(nullptr);
    }

    // release使用绝对唤醒时刻，减少服务周期随执行耗时漂移。
    TickType_t release = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&release, pdMS_TO_TICKS(kWifiPeriodMs));
        // xQueuePeek()复制最新快照但不移除队列中的值。
        wifi_telemtry::TelemetrySnapshot snapshot{};
        const wifi_telemtry::TelemetrySnapshot *latest = nullptr;
        if (xQueuePeek(context.telemetry_queue, &snapshot, 0U) == pdPASS) {
            latest = &snapshot;
        }
        wifi_telemtry::service(latest);
    }
}

void diagnosticsTask(void *argument)
{
    // 诊断任务只读取任务句柄和栈水位，不进入控制数据路径。
    auto &context = *static_cast<TaskContext *>(argument);
    TickType_t release = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&release, pdMS_TO_TICKS(kDiagnosticsPeriodMs));
        diagnostics::logTaskStack(
            "ControlTask", context.control_handle.load(std::memory_order_acquire));
        diagnostics::logTaskStack(
            "WifiTelemetryTask", context.wifi_handle.load(std::memory_order_acquire));
        diagnostics::logTaskStack("DiagnosticsTask", nullptr);
    }
}

} // namespace freertos_tasks
} // namespace vehicle
