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
    bool was_driving = false;
    bool current_saturated = false;
    std::int64_t previous_cycle_us = esp_timer_get_time();
    std::int64_t previous_attitude_us = previous_cycle_us;
    std::int64_t next_attitude_us = previous_cycle_us;
    imu::AttitudeSample attitude{};
    control::ControlOutput output{};
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        const std::int64_t cycle_time_us = esp_timer_get_time();
        const float cycle_dt_s = (cycle_time_us - previous_cycle_us) * 1.0e-6f;
        previous_cycle_us = cycle_time_us;
        if (!(cycle_dt_s > 0.0f && cycle_dt_s <= config::kMaximumControlGapS)) { stopControl(3U); }
        const ble::CommandSnapshot command = ble::latestCommand();
        if (command.mode == ble::RemoteMode::emergency) { stopControl(0U, true); }
        const bool driving = command.mode == ble::RemoteMode::active;
        const bool starting = driving && !was_driving;
        if (!driving || starting) {
            control::initialize(controller);
            output = {};
            current_saturated = false;
        }
        if (!driving) { motor::pauseOutputs(); }
        was_driving = driving;
        // 无PWM写入的编码器采样；电流采样放到IMU/外环之后以缩短电流年龄。
        const motor::WheelState wheels = motor::readWheelState();
        if (!wheels.valid) { stopControl(2U); }
        control::observeCurrentSaturation(controller, current_saturated);
        const bool attitude_due = !attitude.valid || starting ||
            cycle_time_us >= next_attitude_us;
        if (attitude_due) {
            attitude = imu::readAttitude();
            const float attitude_dt_s = (cycle_time_us - previous_attitude_us) * 1.0e-6f;
            previous_attitude_us = cycle_time_us;
            constexpr auto attitude_period_us = static_cast<std::int64_t>(
                config::kControlPeriodUs * config::kAttitudeDivider);
            // 绝对截止点避免接近2ms的抖动使姿态更新退化为每3周期；跳过旧释放点。
            if (cycle_time_us >= next_attitude_us) {
                next_attitude_us += ((cycle_time_us - next_attitude_us) / attitude_period_us + 1) * attitude_period_us;
            }
            const float pitch_rad = attitude.pitch_deg * config::kDegToRad;
            const float pitch_rate_rad_s = attitude.pitch_rate_deg_s * config::kDegToRad;
            if (!attitude.valid || !std::isfinite(pitch_rad) || !std::isfinite(pitch_rate_rad_s)) { stopControl(2U); }
            if (driving && std::abs(pitch_rad - config::kPitchOffsetRad) > config::kFallAngleRad) { stopControl(2U); }
            if (!(attitude_dt_s > 0.0f && attitude_dt_s <= config::kMaximumControlGapS)) { stopControl(3U); }
            // BLE旧转向刻度仅在边界解释；正旧转向=左轮更快，故映射到负偏航。
            const float yaw_command_rad_s = -std::clamp(
                command.steering_voltage_v / config::kRemoteSteeringVoltageV, -1.0f, 1.0f) *
                config::kYawRateLimitRadS;
            output = control::update(controller, {
                wheels.left_velocity_rad_s, wheels.right_velocity_rad_s,
                pitch_rad, pitch_rate_rad_s, command.throttle_velocity_rad_s,
                yaw_command_rad_s, driving}, attitude_dt_s);
            if (!output.valid) { stopControl(2U); }
        }
        motor::CurrentFeedback current{};
        if (driving) {
            current = motor::runCurrentControl({output.left_target_a, output.right_target_a});
            if (!current.valid) { stopControl(2U); }
        }
        current_saturated = current.left.voltage_saturated || current.right.voltage_saturated ||
            current.left.reference_limited || current.right.reference_limited;
        if (esp_timer_get_time() - cycle_time_us > static_cast<std::int64_t>(config::kMaximumControlGapS * 1.0e6f)) {
            stopControl(3U);
        }
        ++sequence;
        ble::publishStatus(ble::StatusSnapshot{
            command, cycle_time_us, sequence, attitude.pitch_deg,
            wheels.left_velocity_rad_s, wheels.right_velocity_rad_s, 0U, true,
        });
        const wifi_telemtry::TelemetrySnapshot snapshot{
            cycle_time_us, sequence, attitude.pitch_deg,
            wheels.left_velocity_rad_s, wheels.right_velocity_rad_s,
            current.left.iq_reference_a, current.right.iq_reference_a,
            current.left.iq_measured_a, current.right.iq_measured_a,
            current.left.uq_applied_v, current.right.uq_applied_v,
            current.left.phase_a_a, current.left.phase_b_a, current.left.phase_c_a,
            current.right.phase_a_a, current.right.phase_b_a, current.right.phase_c_a,
            current.dt_s, current.sample_age_us, current_saturated, current.valid,
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
