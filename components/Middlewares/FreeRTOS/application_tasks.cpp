#include "application_tasks.hpp"

#include "balance_controller.hpp"
#include "ble_command_service.hpp"
#include "bmi160_attitude.hpp"
#include "diagnostics.hpp"
#include "motor_foc_service.hpp"
#include "vehicle_config.hpp"
#include "wifi_telemtry.hpp"

#include "esp_timer.h"
#include <algorithm>
#include <cmath>

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
    // controller保存轨迹、外环积分和电流分配的跨周期状态。
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
    unsigned attitude_divider = 0;
    bool was_driving = false;
    std::int64_t previous_cycle_us = esp_timer_get_time();
    std::int64_t previous_attitude_us = previous_cycle_us;
    imu::AttitudeSample attitude{};
    control::ControlOutput output{};
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        const std::int64_t cycle_time_us = esp_timer_get_time();
        const float cycle_dt = (cycle_time_us - previous_cycle_us) * 1.0e-6f;
        previous_cycle_us = cycle_time_us;
        if (!(cycle_dt > 0.0f && cycle_dt <= config::kMaximumControlGapS)) { stopControl(3U); }
        const ble::CommandSnapshot command = ble::latestCommand();
        if (command.mode == ble::RemoteMode::emergency) { stopControl(0U, true); }
        if (was_driving && command.mode != ble::RemoteMode::active) {
            // 停止事件在电流环之前撤销旧驾驶目标，不等待下一次外环更新。
            control::initialize(controller);
            motor::stageTarget({0.0f, 0.0f});
            output = {};
        }
        was_driving = command.mode == ble::RemoteMode::active;
        const motor::WheelState wheels = motor::runFocAndReadWheelState();
        if (!wheels.valid) { stopControl(2U); }
        if (++attitude_divider < config::kAttitudeDivider) { continue; }
        attitude_divider = 0;
        attitude = imu::readAttitude();
        const std::int64_t attitude_time_us = esp_timer_get_time();
        const float dt = (attitude_time_us - previous_attitude_us) * 1.0e-6f;
        previous_attitude_us = attitude_time_us;
        if (!attitude.valid || !std::isfinite(attitude.pitch_deg) ||
            !std::isfinite(attitude.pitch_rate_deg_s) ||
            std::abs(attitude.pitch_deg - config::kPitchOffsetDeg) > config::kFallAngleDeg) {
            stopControl(2U);
        }
        if (!(dt > 0.0f && dt <= config::kMaximumControlGapS)) { stopControl(3U); }
        // 保留BLE线上电压刻度，仅此处将其解释为归一化转向命令。
        // 旧命令正转向为左轮更快，转换到右轮更快为正的偏航坐标需负号。
        const float yaw_command = -std::clamp(
            command.steering_voltage_v / config::kRemoteSteeringVoltageV, -1.0f, 1.0f) *
            config::kYawRateLimitRadS;
        output = control::update(controller, {
            config::kMotor0Direction * wheels.left_velocity_rad_s,
            config::kMotor1Direction * wheels.right_velocity_rad_s,
            attitude.pitch_deg, attitude.pitch_rate_deg_s,
            command.throttle_velocity_rad_s, yaw_command,
            command.mode == ble::RemoteMode::active}, dt);
        motor::stageTarget({output.left_target_a, output.right_target_a});
        ++sequence;
        ble::publishStatus(ble::StatusSnapshot{
            command, cycle_time_us, sequence, attitude.pitch_deg,
            config::kMotor0Direction * wheels.left_velocity_rad_s,
            config::kMotor1Direction * wheels.right_velocity_rad_s, 0U, true,
        });
        const wifi_telemtry::TelemetrySnapshot snapshot{
            cycle_time_us, sequence, attitude.pitch_deg,
            wheels.left_velocity_rad_s, wheels.right_velocity_rad_s,
            output.left_target_a, output.right_target_a,
        };
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
