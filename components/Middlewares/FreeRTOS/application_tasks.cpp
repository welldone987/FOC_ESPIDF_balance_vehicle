#include "application_tasks.hpp"

#include "balance_controller.hpp"
#include "ble_command_service.hpp"
#include "control_config.hpp"
#include "control_timing.hpp"
#include "motion_command.hpp"
#include "bmi160_attitude.hpp"
#include "diagnostics.hpp"
#include "encoder_config.hpp"
#include "motor_config.hpp"
#include "motor_foc_service.hpp"
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
 * 初始化完成后开始本地零速平衡；BleTask只提供速度和转向目标。
 */

constexpr std::uint32_t kWifiPeriodMs = 50U;
esp_timer_handle_t control_timer{};
// 仅ControlTask访问；运行中不格式化日志，完成/故障时复制定长现场。
control::ControlTiming cycle_timing{};
std::int64_t cycle_started_us{};


// stopControl()关闭电机输出后挂起控制任务，避免故障状态继续驱动执行器。
[[noreturn]] void stopControl(const ErrorInfo &error, bool boot_failure=false)
{
    if (cycle_started_us != 0) { cycle_timing.elapsed_us=esp_timer_get_time()-cycle_started_us; }
    ErrorInfo secondary{};
    const auto rc=motor::inhibitOutputs(&secondary);
    diagnostics::controlTiming(cycle_timing);
    diagnostics::record(error,true);
    if (rc != ESP_OK) { diagnostics::record(secondary); }
    motor::disableOutputs();
    if (control_timer) { esp_timer_stop(control_timer); }
    // 输出已禁能，定时器已停止；一次性串口报告不占用运行周期预算。
    diagnostics::printControlFault(error);
    if (rc != ESP_OK) {
        ESP_LOGE("control_diag","CONTROL_SECONDARY disable request failed: code=%s raw=%ld file=%s:%lu",
            esp_err_to_name(rc),static_cast<long>(secondary.raw_code),secondary.file ? secondary.file : "?",static_cast<unsigned long>(secondary.line));
    }
    if (boot_failure) {
        diagnostics::boot(static_cast<diagnostics::BootStep>(error.point_id),"FAIL",error.code);
        ESP_LOGE("boot","BOOT_SUMMARY FAIL point=%u raw=%ld at %s:%lu",static_cast<unsigned>(error.point_id),static_cast<long>(error.raw_code),error.file,static_cast<unsigned long>(error.line));
    }
    for (;;) { vTaskDelay(pdMS_TO_TICKS(1000U)); }
}
void bootResult(diagnostics::BootStep step, esp_err_t rc, const ErrorInfo &error)
{
    if (rc != ESP_OK) { stopControl(error,true); }
    diagnostics::boot(step,"OK",rc);
}

// releaseControl()由ESP定时器回调通知控制任务开始下一周期。
void releaseControl(void *task_handle)
{
    xTaskNotifyGive(static_cast<TaskHandle_t>(task_handle));
}

} // namespace

void bleTask(void *argument)
{
    auto &context=*static_cast<TaskContext *>(argument);
    BleStartup startup{};
    startup.result=ble::initialize(&startup.error);
    xQueueOverwrite(context.ble_startup_queue,&startup);
    if (startup.result == ESP_OK) { ble::run(context.command_queue); }
    // 初始化失败不重试，也不放行控制；静态任务资源保留供诊断。
    for (;;) { vTaskDelay(pdMS_TO_TICKS(1000)); }
}

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
    result=motor::initialize(&error,[](std::uint16_t step,const char *state) {
        diagnostics::boot(static_cast<diagnostics::BootStep>(step),state);
    });
    bootResult(diagnostics::BootStep::motor,result,error);
    diagnostics::boot(diagnostics::BootStep::outputs_off,"BEGIN");
    result=motor::pauseOutputs(&error);
    bootResult(diagnostics::BootStep::outputs_off,result,error);
    imu::resetEstimator();
    ESP_LOGI("control_diag","CONTROL_START mode=INDEPENDENT_BALANCE; BLE supplies velocity and yaw targets");
    ESP_LOGI("control_diag","CONTROL_CONFIG current_period_us=%llu attitude_period_us=%llu outer_period_us=%ld",
        static_cast<unsigned long long>(motor::config::kControlPeriodUs),
        static_cast<unsigned long long>(motor::config::kControlPeriodUs * control::config::kAttitudeDivider),
        static_cast<long>(control::config::kOuterPeriodS * 1.0e6f));
    ESP_LOGI("control_diag","CONTROL_LIMITS encoder_output_max_age_us=%lld current_output_max_age_us=%lld",
        static_cast<long long>(encoder::config::kOutputMaxAgeUs),
        static_cast<long long>(motor::config::kCurrentOutputMaxAgeUs));

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
        result = esp_timer_start_periodic(timer, motor::config::kControlPeriodUs);
    }
    if (result != ESP_OK) { VEHICLE_ERROR(&error,result,control_timer,esp,result); }
    bootResult(diagnostics::BootStep::timer,result,error);
    diagnostics::boot(diagnostics::BootStep::complete,"OK");
    ESP_LOGI("boot","BOOT_SUMMARY OK");
    diagnostics::completeBoot();
    const auto command_ready_us=esp_timer_get_time();
    control::MotionCommand latest_command{};

    std::uint32_t sequence = 0U;
    std::uint32_t balance_cycles=0U, skipped_releases=0U;
    bool was_balancing = false;
    bool current_saturated = false;
    std::int64_t previous_cycle_us = 0;
    std::int64_t previous_attitude_us = previous_cycle_us;
    std::int64_t next_attitude_us = previous_cycle_us;
    imu::AttitudeSample attitude{};
    control::ControlOutput output{};
    for (;;) {
        const auto notifications=ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        const std::int64_t cycle_time_us = esp_timer_get_time();
        cycle_started_us=cycle_time_us;
        if (notifications>1) { skipped_releases+=notifications-1; }
        cycle_timing={};
        cycle_timing.cycle=sequence+1; cycle_timing.balance_cycle=balance_cycles;
        cycle_timing.notifications=notifications; cycle_timing.skipped_releases=skipped_releases;
        cycle_timing.first_release=previous_cycle_us == 0;
        cycle_timing.dt_us=control::sampleInterval(cycle_time_us,previous_cycle_us,motor::config::kControlPeriodUs);
        const float cycle_dt_s = cycle_timing.dt_us * 1.0e-6f;
        previous_cycle_us = cycle_time_us;
        cycle_timing.stage=control::ControlStage::command;
        if (!(cycle_dt_s > 0.0f && cycle_dt_s <= control::config::kMaximumControlGapS)) { VEHICLE_ERROR(&error, ESP_ERR_TIMEOUT, control_gap, application, 0, cycle_dt_s, control::config::kMaximumControlGapS, -1, 3, 1); stopControl(error); }
        // 非阻塞读取最新目标；ControlTask独立检查时效，包括BleTask饥饿的情况。
        (void)xQueueReceive(context.command_queue,&latest_command,0);
        const bool driving=control::freshCommand(latest_command,cycle_time_us,command_ready_us);
        const auto command=driving ? latest_command : control::MotionCommand{};
        const bool balancing=true;
        const bool starting = balancing && !was_balancing;
        cycle_timing.balancing=balancing; cycle_timing.driving=driving; cycle_timing.starting=starting;
        if (balancing) { cycle_timing.balance_cycle=++balance_cycles; }
        if (starting) {
            control::initialize(controller);
            output = {};
            current_saturated = false;
        }
        was_balancing = balancing;
        control::observeCurrentSaturation(controller, current_saturated);
        const bool attitude_due = !attitude.valid || starting ||
            cycle_time_us >= next_attitude_us;
        float attitude_dt_s=0.0f;
        // 先完成到期IMU读取与倾倒检查，避免其阻塞时间计入随后采集的编码器年龄。
        if (attitude_due) {
            cycle_timing.imu_updated=true; cycle_timing.stage=control::ControlStage::imu;
            const auto imu_start=esp_timer_get_time();
            result=imu::readAttitude(&attitude,&error);
            cycle_timing.imu_us=esp_timer_get_time()-imu_start;
            if (result != ESP_OK) { stopControl(error); }
            attitude_dt_s = control::sampleInterval(cycle_time_us,previous_attitude_us,
                motor::config::kControlPeriodUs * control::config::kAttitudeDivider) * 1.0e-6f;
            previous_attitude_us = cycle_time_us;
            constexpr auto attitude_period_us = static_cast<std::int64_t>(
                motor::config::kControlPeriodUs * control::config::kAttitudeDivider);
            // 按5ms绝对截止点更新姿态，跳过旧释放点，不补算积压样本。
            if (cycle_time_us >= next_attitude_us) {
                next_attitude_us += ((cycle_time_us - next_attitude_us) / attitude_period_us + 1) * attitude_period_us;
            }
            const float pitch_rad = attitude.pitch_deg * control::config::kDegToRad;
            const float pitch_rate_rad_s = attitude.pitch_rate_deg_s * control::config::kDegToRad;
            if (!attitude.valid || !std::isfinite(pitch_rad) || !std::isfinite(pitch_rate_rad_s)) { VEHICLE_ERROR(&error, ESP_ERR_INVALID_RESPONSE, imu_filter, application, 0); stopControl(error); }
            if (balancing && std::abs(pitch_rad - control::config::kPitchOffsetRad) > control::config::kFallAngleRad) { VEHICLE_ERROR(&error, ESP_ERR_INVALID_STATE, fall, application, 0, pitch_rad-control::config::kPitchOffsetRad, control::config::kFallAngleRad, -1, 3, 1); stopControl(error); }
            if (!(attitude_dt_s > 0.0f && attitude_dt_s <= control::config::kMaximumControlGapS)) { VEHICLE_ERROR(&error, ESP_ERR_TIMEOUT, attitude_gap, application, 0, attitude_dt_s, control::config::kMaximumControlGapS, -1, 3, 1); stopControl(error); }
        }
        // 编码器仍每个电流周期采集；本轮外环同时使用最新IMU和最新轮速。
        motor::WheelState wheels{};
        cycle_timing.stage=control::ControlStage::encoder;
        const auto encoder_start=esp_timer_get_time();
        result=motor::readWheelState(&wheels,&error);
        cycle_timing.encoder_us=esp_timer_get_time()-encoder_start;
        if (result != ESP_OK) { stopControl(error); }
        if (attitude_due) {
            const float pitch_rad = attitude.pitch_deg * control::config::kDegToRad;
            const float pitch_rate_rad_s = attitude.pitch_rate_deg_s * control::config::kDegToRad;
            cycle_timing.stage=control::ControlStage::outer;
            const auto outer_start=esp_timer_get_time();
            output = control::update(controller, {
                wheels.left_velocity_rad_s, wheels.right_velocity_rad_s,
                pitch_rad, pitch_rate_rad_s, command.velocity_rad_s,
                command.yaw_rate_rad_s, balancing, driving}, attitude_dt_s);
            cycle_timing.outer_us=esp_timer_get_time()-outer_start;
            if (!output.valid) { VEHICLE_ERROR(&error, ESP_ERR_INVALID_RESPONSE, control_output, application, 0); stopControl(error); }
        }
        motor::CurrentFeedback current{};
        if (balancing) {
            cycle_timing.stage=control::ControlStage::current;
            if (motor::runCurrentControl({output.left_target_a, output.right_target_a},&current,&error,&cycle_timing.current) != ESP_OK) { stopControl(error); }
        }
        current_saturated = current.left.voltage_saturated || current.right.voltage_saturated ||
            current.left.reference_limited || current.right.reference_limited;
        if (esp_timer_get_time() - cycle_time_us > static_cast<std::int64_t>(control::config::kMaximumControlGapS * 1.0e6f)) {
            VEHICLE_ERROR(&error, ESP_ERR_TIMEOUT, control_gap, application, 0, esp_timer_get_time()-cycle_time_us, control::config::kMaximumControlGapS*1.0e6f, -1, 3, 1); stopControl(error);
        }
        ++sequence;
        cycle_timing.stage=control::ControlStage::publish;
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
        cycle_timing.stage=control::ControlStage::complete;
        cycle_timing.elapsed_us=esp_timer_get_time()-cycle_time_us;
        diagnostics::controlTiming(cycle_timing);
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
