#include "application_tasks.hpp"

#include "balance_controller.hpp"
#include "ble_command_service.hpp"
#include "bmi160_attitude.hpp"
#include "diagnostics.hpp"
#include "motor_foc_service.hpp"
#include "vehicle_config.hpp"
#include "wifi_telemtry.hpp"

#include "esp_timer.h"
#include "esp_log.h"
#include <algorithm>
#include <cmath>

namespace vehicle {
namespace freertos_tasks {
namespace {

/*
 * 控制任务独占传感器与执行器；可选Wi-Fi任务消费最新遥测。
 * 初始化完成并输出BOOT_SUMMARY后，才允许v2 ARM进入控制运行。
 */

constexpr std::uint32_t kWifiPeriodMs = 50U;
esp_timer_handle_t control_timer{};


// stopControl()关闭电机输出后挂起控制任务，避免故障状态继续驱动执行器。
[[noreturn]] void stopControl(const ErrorInfo &error, std::uint8_t fault=2U, bool emergency=false, bool boot_failure=false)
{
    ErrorInfo secondary{};
    const auto rc=motor::inhibitOutputs(&secondary);
    diagnostics::record(error,true);
    if (rc != ESP_OK) { diagnostics::record(secondary); }
    ble::publishFault(fault,emergency);
    motor::disableOutputs();
    if (control_timer) { esp_timer_stop(control_timer); }
    if (boot_failure) {
        diagnostics::boot(static_cast<diagnostics::BootStep>(error.point_id),"FAIL",error.code);
        ESP_LOGE("boot","BOOT_SUMMARY FAIL point=%u raw=%ld at %s:%lu",static_cast<unsigned>(error.point_id),static_cast<long>(error.raw_code),error.file,static_cast<unsigned long>(error.line));
    }
    for (;;) { vTaskDelay(pdMS_TO_TICKS(1000U)); }
}
void bootResult(diagnostics::BootStep step, esp_err_t rc, const ErrorInfo &error)
{
    if (rc != ESP_OK) { stopControl(error,1,false,true); }
    diagnostics::boot(step,"OK",rc);
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
    [[maybe_unused]] auto &context = *static_cast<TaskContext *>(argument);
    // controller保存轨迹、外环积分和电流分配的跨周期状态。
    control::ControllerState controller{};
    control::initialize(controller);

    ErrorInfo error{};
    diagnostics::boot(diagnostics::BootStep::imu,"BEGIN");
    esp_err_t result=imu::initialize(&error);
    bootResult(diagnostics::BootStep::imu,result,error);
    diagnostics::boot(diagnostics::BootStep::motor,"BEGIN");
    if (ble::latestCommand().mode == ble::RemoteMode::emergency) {
        VEHICLE_ERROR(&error,ESP_ERR_INVALID_STATE,emergency,application,0); stopControl(error,0,true,true);
    }
    result=motor::initialize(&error,[](std::uint16_t step,const char *state) {
        diagnostics::boot(static_cast<diagnostics::BootStep>(step),state);
    });
    bootResult(diagnostics::BootStep::motor,result,error);
    diagnostics::boot(diagnostics::BootStep::outputs_off,"BEGIN");
    result=motor::pauseOutputs(&error);
    bootResult(diagnostics::BootStep::outputs_off,result,error);
    imu::resetEstimator();

    // 控制定时器使用ESP_TIMER_TASK回调，回调只发送通知，不执行控制计算。
    esp_timer_create_args_t timer_config{};
    timer_config.callback = releaseControl;
    timer_config.arg = xTaskGetCurrentTaskHandle();
    timer_config.dispatch_method = ESP_TIMER_TASK;
    timer_config.name = "control_release";

    diagnostics::boot(diagnostics::BootStep::timer,"BEGIN");
    auto &timer = control_timer;
    result = esp_timer_create(&timer_config, &timer);
    if (result == ESP_OK) {
        result = esp_timer_start_periodic(timer, config::kControlPeriodUs);
    }
    if (result != ESP_OK) { VEHICLE_ERROR(&error,result,control_timer,esp,result); }
    bootResult(diagnostics::BootStep::timer,result,error);
    diagnostics::boot(diagnostics::BootStep::complete,"OK");
    ESP_LOGI("boot","BOOT_SUMMARY OK");
    diagnostics::completeBoot();
    ble::allowControl();

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
        if (!(cycle_dt_s > 0.0f && cycle_dt_s <= config::kMaximumControlGapS)) { VEHICLE_ERROR(&error, ESP_ERR_TIMEOUT, control_gap, application, 0, cycle_dt_s, config::kMaximumControlGapS, -1, 3, 1); stopControl(error,3U); }
        const ble::CommandSnapshot command = ble::latestCommand();
        if (command.mode == ble::RemoteMode::emergency) { VEHICLE_ERROR(&error, ESP_ERR_INVALID_STATE, emergency, application, 0); stopControl(error,0U,true); }
        const bool driving = command.mode == ble::RemoteMode::active;
        const bool starting = driving && !was_driving;
        if (!driving || starting) {
            control::initialize(controller);
            output = {};
            current_saturated = false;
        }
        if (!driving && motor::pauseOutputs(&error) != ESP_OK) { stopControl(error); }
        was_driving = driving;
        // 无PWM写入的编码器采样；电流采样放到IMU/外环之后以缩短电流年龄。
        motor::WheelState wheels{};
        if (motor::readWheelState(&wheels,&error) != ESP_OK) { stopControl(error); }
        control::observeCurrentSaturation(controller, current_saturated);
        const bool attitude_due = !attitude.valid || starting ||
            cycle_time_us >= next_attitude_us;
        if (attitude_due) {
            if (imu::readAttitude(&attitude,&error) != ESP_OK) { stopControl(error); }
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
            if (!attitude.valid || !std::isfinite(pitch_rad) || !std::isfinite(pitch_rate_rad_s)) { VEHICLE_ERROR(&error, ESP_ERR_INVALID_RESPONSE, imu_filter, application, 0); stopControl(error); }
            if (driving && std::abs(pitch_rad - config::kPitchOffsetRad) > config::kFallAngleRad) { VEHICLE_ERROR(&error, ESP_ERR_INVALID_STATE, fall, application, 0, pitch_rad-config::kPitchOffsetRad, config::kFallAngleRad, -1, 3, 1); stopControl(error); }
            if (!(attitude_dt_s > 0.0f && attitude_dt_s <= config::kMaximumControlGapS)) { VEHICLE_ERROR(&error, ESP_ERR_TIMEOUT, attitude_gap, application, 0, attitude_dt_s, config::kMaximumControlGapS, -1, 3, 1); stopControl(error,3U); }
            // BLE旧转向刻度仅在边界解释；正旧转向=左轮更快，故映射到负偏航。
            const float yaw_command_rad_s = -std::clamp(
                command.steering_voltage_v / config::kRemoteSteeringVoltageV, -1.0f, 1.0f) *
                config::kYawRateLimitRadS;
            output = control::update(controller, {
                wheels.left_velocity_rad_s, wheels.right_velocity_rad_s,
                pitch_rad, pitch_rate_rad_s, command.throttle_velocity_rad_s,
                yaw_command_rad_s, driving}, attitude_dt_s);
            if (!output.valid) { VEHICLE_ERROR(&error, ESP_ERR_INVALID_RESPONSE, control_output, application, 0); stopControl(error); }
        }
        motor::CurrentFeedback current{};
        if (driving) {
            if (motor::runCurrentControl({output.left_target_a, output.right_target_a},&current,&error) != ESP_OK) { stopControl(error); }
        }
        current_saturated = current.left.voltage_saturated || current.right.voltage_saturated ||
            current.left.reference_limited || current.right.reference_limited;
        if (esp_timer_get_time() - cycle_time_us > static_cast<std::int64_t>(config::kMaximumControlGapS * 1.0e6f)) {
            VEHICLE_ERROR(&error, ESP_ERR_TIMEOUT, control_gap, application, 0, esp_timer_get_time()-cycle_time_us, config::kMaximumControlGapS*1.0e6f, -1, 3, 1); stopControl(error,3U);
        }
        ++sequence;
        ble::publishStatus(ble::StatusSnapshot{
            command, cycle_time_us, sequence, attitude.pitch_deg,
            wheels.left_velocity_rad_s, wheels.right_velocity_rad_s, 0U, true,
        });
        diagnostics::controlSnapshot({cycle_time_us,sequence,attitude.pitch_deg,
            wheels.left_velocity_rad_s,wheels.right_velocity_rad_s,output.left_target_a,output.right_target_a,
            current.left.iq_measured_a,current.right.iq_measured_a,cycle_dt_s,true});
#if CONFIG_VEHICLE_WIFI_ENABLED
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
        if (context.telemetry_queue) { xQueueOverwrite(context.telemetry_queue, &snapshot); }
#endif
    }
}
#if CONFIG_VEHICLE_WIFI_ENABLED
void wifiTelemetryTask(void *argument)
{
    // Wi-Fi服务运行在低频服务任务中，与控制任务共享最新值队列。
    [[maybe_unused]] auto &context = *static_cast<TaskContext *>(argument);
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

#endif

} // namespace freertos_tasks
} // namespace vehicle
