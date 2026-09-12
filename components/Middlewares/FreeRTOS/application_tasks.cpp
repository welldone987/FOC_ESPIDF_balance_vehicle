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
#include "wifi_telemetry.hpp"
#include "wifi_telemetry_config.hpp"

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

// control_timer是周期通知控制任务的ESP定时器句柄。
esp_timer_handle_t control_timer{};
// 仅ControlTask访问。
// 运行中不格式化日志，完成/故障时复制定长现场。
control::ControlTiming cycle_timing{};
// cycle_started_us记录本轮开始的时刻，单位us。
std::int64_t cycle_started_us{};


// StopControl()关闭电机输出后挂起控制任务，避免故障状态继续驱动执行器。
[[noreturn]] void StopControl(const ErrorInfo &error, bool boot_failure=false)
{
    if (cycle_started_us != 0) { cycle_timing.elapsed_us=esp_timer_get_time()-cycle_started_us; }
    ErrorInfo secondary{};
    const auto rc=motor::InhibitOutputs(&secondary);
    diagnostics::CommitControlTiming(cycle_timing);
    diagnostics::Record(error,true);
    if (rc != ESP_OK) { diagnostics::Record(secondary); }
    motor::DisableOutputs();
    if (control_timer) { esp_timer_stop(control_timer); }
    // 此时输出已禁能、定时器已停止。
    // 一次性串口报告不占用运行周期预算。
    diagnostics::PrintControlFault(error);
    if (rc != ESP_OK) {
        ESP_LOGE("control_diag","CONTROL_SECONDARY disable request failed: code=%s raw=%ld file=%s:%lu",
            esp_err_to_name(rc),static_cast<long>(secondary.raw_code),secondary.file ? secondary.file : "?",static_cast<unsigned long>(secondary.line));
    }
    if (boot_failure) {
        diagnostics::Boot(static_cast<diagnostics::BootStep>(error.point_id),"FAIL",error.code);
        ESP_LOGE("boot","BOOT_SUMMARY FAIL point=%u raw=%ld at %s:%lu",static_cast<unsigned>(error.point_id),static_cast<long>(error.raw_code),error.file,static_cast<unsigned long>(error.line));
    }
    for (;;) { vTaskDelay(pdMS_TO_TICKS(1000U)); }
}
// BootResult()在启动步骤失败时转入锁存停机，否则输出启动日志。
void BootResult(diagnostics::BootStep step, esp_err_t rc, const ErrorInfo &error)
{
    if (rc != ESP_OK) { StopControl(error,true); }
    diagnostics::Boot(step,"OK",rc);
}

// ReleaseControl()由ESP定时器回调通知控制任务开始下一周期。
void ReleaseControl(void *task_handle)
{
    xTaskNotifyGive(static_cast<TaskHandle_t>(task_handle));
}

} // namespace

// BleTask初始化BLE并把结果写入启动队列，随后进入报文消费循环。
void BleTask(void *argument)
{
    auto &context=*static_cast<TaskContext *>(argument);
    BleStartup startup{};
    startup.result=ble::Initialize(context.telemetry_queue,&startup.error);
    xQueueOverwrite(context.ble_startup_queue,&startup);
    if (startup.result == ESP_OK) { ble::Run(context.command_queue); }
    // 初始化失败不重试，也不放行控制。
    // 静态任务资源保留供诊断。
    for (;;) { vTaskDelay(pdMS_TO_TICKS(1000)); }
}

// ControlTask完成硬件初始化、启动控制定时器并进入500Hz控制循环。
void ControlTask(void *argument)
{
    // context由app_main提供，在整个静态任务生命周期内保持有效。
    [[maybe_unused]] auto &context = *static_cast<TaskContext *>(argument);
    // controller保存轨迹、外环积分和电流分配的跨周期状态。
    control::ControllerState controller{};
    control::Initialize(controller);

    // error在各初始化步骤和周期故障中复用。
    ErrorInfo error{};
    diagnostics::Boot(diagnostics::BootStep::imu,"BEGIN");
    esp_err_t result=imu::Initialize(&error);
    BootResult(diagnostics::BootStep::imu,result,error);
    diagnostics::Boot(diagnostics::BootStep::motor,"BEGIN");
    result=motor::Initialize(&error,[](std::uint16_t step,const char *state) {
        diagnostics::Boot(static_cast<diagnostics::BootStep>(step),state);
    });
    BootResult(diagnostics::BootStep::motor,result,error);
    diagnostics::Boot(diagnostics::BootStep::outputs_off,"BEGIN");
    result=motor::PauseOutputs(&error);
    BootResult(diagnostics::BootStep::outputs_off,result,error);
    imu::ResetEstimator();
    ESP_LOGI("control_diag","CONTROL_START mode=INDEPENDENT_BALANCE; BLE supplies velocity and yaw targets");
    ESP_LOGI("control_diag","CONTROL_CONFIG current_period_us=%llu attitude_period_us=%llu outer_period_us=%ld",
        static_cast<unsigned long long>(motor::ControlPeriod_us),
        static_cast<unsigned long long>(control::AttitudePeriod_us),
        static_cast<long>(control::OuterPeriod_s * 1.0e6f));
    ESP_LOGI("control_diag","CONTROL_LIMITS encoder_output_max_age_us=%lld current_output_max_age_us=%lld",
        static_cast<long long>(encoder::OutputMaxAge_us),
        static_cast<long long>(motor::CurrentOutputMaxAge_us));

    // 控制定时器使用ESP_TIMER_TASK回调，回调只发送通知，不执行控制计算。
    esp_timer_create_args_t timer_config{};
    timer_config.callback = ReleaseControl;
    timer_config.arg = xTaskGetCurrentTaskHandle();
    timer_config.dispatch_method = ESP_TIMER_TASK;
    timer_config.name = "control_release";

    diagnostics::Boot(diagnostics::BootStep::timer,"BEGIN");
    auto &timer = control_timer;
    result = esp_timer_create(&timer_config, &timer);
    if (result == ESP_OK) {
        result = esp_timer_start_periodic(timer, motor::ControlPeriod_us);
    }
    if (result != ESP_OK) { VEHICLE_ERROR(&error,result,control_timer,esp,result); }
    BootResult(diagnostics::BootStep::timer,result,error);
    diagnostics::Boot(diagnostics::BootStep::complete,"OK");
    ESP_LOGI("boot","BOOT_SUMMARY OK");
    diagnostics::CompleteBoot();
    // command_ready_us之后的命令才被采纳，用于拒绝初始化期间的目标。
    const auto command_ready_us=esp_timer_get_time();
    // latest_command保存最近接收的运动目标。
    control::MotionCommand latest_command{};

    // sequence、balance_cycles和skipped_releases记录周期与通知统计。
    std::uint32_t sequence = 0U;
    std::uint32_t balance_cycles=0U, skipped_releases=0U;
    // was_balancing和was_driving保存上一轮的使能与目标状态。
    bool was_balancing = false;
    bool was_driving = false;
    // current_saturated汇总本周期电流环饱和，供外环冻结积分。
    bool current_saturated = false;
    // previous_cycle_us、previous_attitude_us和next_attitude_us维护控制与姿态的时间基准。
    std::int64_t previous_cycle_us = 0;
    std::int64_t previous_attitude_us = previous_cycle_us;
    std::int64_t next_attitude_us = previous_cycle_us;
    // attitude保存最近一帧姿态估计。
    imu::AttitudeSample attitude{};
    // output保存外环最近一次输出。
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
        cycle_timing.dt_us=control::SampleInterval(cycle_time_us,previous_cycle_us,motor::ControlPeriod_us);
        const float cycle_dt_s = cycle_timing.dt_us * 1.0e-6f;
        previous_cycle_us = cycle_time_us;
        cycle_timing.stage=control::ControlStage::command;
        if (!(cycle_dt_s > 0.0f && cycle_dt_s <= control::MaximumControlGap_s)) { VEHICLE_ERROR(&error, ESP_ERR_TIMEOUT, control_gap, application, 0, cycle_dt_s, control::MaximumControlGap_s, -1, 3, 1); StopControl(error); }
        // 非阻塞读取最新目标。
        // ControlTask独立检查时效，包括BleTask饥饿的情况。
        (void)xQueueReceive(context.command_queue,&latest_command,0);
        const bool driving=control::IsCommandFresh(latest_command,cycle_time_us,command_ready_us);
        if (was_driving && !driving && latest_command.valid) {
            ErrorInfo timeout{};
            VEHICLE_ERROR(&timeout,ESP_ERR_TIMEOUT,ble_command_timeout,application,0,
                static_cast<float>(cycle_time_us-latest_command.received_us),
                static_cast<float>(control::CommandTimeout_us),-1,3,1);
            diagnostics::Record(timeout); // 只复制；串口格式化在BleTask低频观察。
        }
        was_driving=driving;
        // command在本轮无效时退化为零目标。
        const auto command=driving ? latest_command : control::MotionCommand{};
        const bool balancing=true;
        const bool starting = balancing && !was_balancing;
        cycle_timing.balancing=balancing; cycle_timing.driving=driving; cycle_timing.starting=starting;
        if (balancing) { cycle_timing.balance_cycle=++balance_cycles; }
        if (starting) {
            control::Initialize(controller);
            output = {};
            current_saturated = false;
        }
        was_balancing = balancing;
        control::ObserveCurrentSaturation(controller, current_saturated);
        // attitude_due标记本轮到姿态截止点。
        // attitude_dt_s是实际姿态间隔，单位s。
        const bool attitude_due = !attitude.valid || starting ||
            cycle_time_us >= next_attitude_us;
        float attitude_dt_s=0.0f;
        // 先完成到期IMU读取与倾倒检查，避免其阻塞时间计入随后采集的编码器年龄。
        if (attitude_due) {
            cycle_timing.imu_updated=true; cycle_timing.stage=control::ControlStage::imu;
            const auto imu_start=esp_timer_get_time();
            result=imu::ReadAttitude(&attitude,&error);
            cycle_timing.imu_us=esp_timer_get_time()-imu_start;
            if (result != ESP_OK) { StopControl(error); }
            attitude_dt_s = control::SampleInterval(cycle_time_us,previous_attitude_us,
                control::AttitudePeriod_us) * 1.0e-6f;
            previous_attitude_us = cycle_time_us;
            constexpr auto attitude_period_us = static_cast<std::int64_t>(control::AttitudePeriod_us);
            // 按5ms绝对截止点更新姿态，跳过旧释放点，不补算积压样本。
            if (cycle_time_us >= next_attitude_us) {
                next_attitude_us += ((cycle_time_us - next_attitude_us) / attitude_period_us + 1) * attitude_period_us;
            }
            const float pitch_rad = attitude.pitch_deg * control::DegToRad;
            const float pitch_rate_rad_s = attitude.pitch_rate_deg_s * control::DegToRad;
            if (!attitude.valid || !std::isfinite(pitch_rad) || !std::isfinite(pitch_rate_rad_s)) { VEHICLE_ERROR(&error, ESP_ERR_INVALID_RESPONSE, imu_filter, application, 0); StopControl(error); }
            if (balancing && std::abs(pitch_rad - control::PitchOffset_rad) > control::FallAngle_rad) { VEHICLE_ERROR(&error, ESP_ERR_INVALID_STATE, fall, application, 0, pitch_rad-control::PitchOffset_rad, control::FallAngle_rad, -1, 3, 1); StopControl(error); }
            if (!(attitude_dt_s > 0.0f && attitude_dt_s <= control::MaximumControlGap_s)) { VEHICLE_ERROR(&error, ESP_ERR_TIMEOUT, attitude_gap, application, 0, attitude_dt_s, control::MaximumControlGap_s, -1, 3, 1); StopControl(error); }
        }
        // 编码器仍每个电流周期采集。
        // 本轮外环同时使用最新IMU和最新轮速。
        // wheels保存本周期轮速。
        // encoder_start用于统计编码器耗时。
        motor::WheelState wheels{};
        cycle_timing.stage=control::ControlStage::encoder;
        const auto encoder_start=esp_timer_get_time();
        result=motor::ReadWheelState(&wheels,&error);
        cycle_timing.encoder_us=esp_timer_get_time()-encoder_start;
        if (result != ESP_OK) { StopControl(error); }
        if (attitude_due) {
            const float pitch_rad = attitude.pitch_deg * control::DegToRad;
            const float pitch_rate_rad_s = attitude.pitch_rate_deg_s * control::DegToRad;
            cycle_timing.stage=control::ControlStage::outer;
            const auto outer_start=esp_timer_get_time();
            output = control::Update(controller, {
                wheels.velocity_M0_rad_s, wheels.velocity_M1_rad_s,
                pitch_rad, pitch_rate_rad_s, command.velocity_rad_s,
                command.yaw_rate_rad_s, balancing, driving}, attitude_dt_s);
            cycle_timing.outer_us=esp_timer_get_time()-outer_start;
            if (!output.valid) { VEHICLE_ERROR(&error, ESP_ERR_INVALID_RESPONSE, control_output, application, 0); StopControl(error); }
        }
        // current保存本周期电流反馈。
        motor::CurrentFeedback current{};
        if (balancing) {
            cycle_timing.stage=control::ControlStage::current;
            if (motor::RunCurrentControl({output.target_M0_A, output.target_M1_A},&current,&error,&cycle_timing.current) != ESP_OK) { StopControl(error); }
        }
        current_saturated = current.sample_M0.voltage_saturated || current.sample_M1.voltage_saturated ||
            current.sample_M0.reference_limited || current.sample_M1.reference_limited;
        if (esp_timer_get_time() - cycle_time_us > static_cast<std::int64_t>(control::MaximumControlGap_s * 1.0e6f)) {
            VEHICLE_ERROR(&error, ESP_ERR_TIMEOUT, control_gap, application, 0, esp_timer_get_time()-cycle_time_us, control::MaximumControlGap_s*1.0e6f, -1, 3, 1); StopControl(error);
        }
        ++sequence;
        cycle_timing.stage=control::ControlStage::publish;
        diagnostics::CommitControlSnapshot({cycle_time_us,sequence,attitude.pitch_deg,
            wheels.velocity_M0_rad_s,wheels.velocity_M1_rad_s,output.target_M0_A,output.target_M1_A,
            current.sample_M0.iq_measured_A,current.sample_M1.iq_measured_A,cycle_dt_s,true});
        // snapshot发布到遥测队列，供BLE .007和可选Wi-Fi消费。
        const wifi_telemetry::TelemetrySnapshot snapshot{
            cycle_time_us, sequence, attitude.pitch_deg,
            wheels.velocity_M0_rad_s, wheels.velocity_M1_rad_s,
            current.sample_M0.iq_reference_A, current.sample_M1.iq_reference_A,
            current.sample_M0.iq_measured_A, current.sample_M1.iq_measured_A,
            current.sample_M0.uq_applied_V, current.sample_M1.uq_applied_V,
            current.sample_M0.phase_a_A, current.sample_M0.phase_b_A, current.sample_M0.phase_c_A,
            current.sample_M1.phase_a_A, current.sample_M1.phase_b_A, current.sample_M1.phase_c_A,
            current.dt_s, current.sample_age_us, current_saturated, current.valid,
        };
        if (context.telemetry_queue) { xQueueOverwrite(context.telemetry_queue, &snapshot); }
        cycle_timing.stage=control::ControlStage::complete;
        cycle_timing.elapsed_us=esp_timer_get_time()-cycle_time_us;
        diagnostics::CommitControlTiming(cycle_timing);
    }
}
#if CONFIG_VEHICLE_WIFI_ENABLED
// WifiTelemetryTask按ServicePeriod_ms消费最新遥测快照。
void WifiTelemetryTask(void *argument)
{
    // Wi-Fi服务运行在低频服务任务中，与控制任务共享最新值队列。
    [[maybe_unused]] auto &context = *static_cast<TaskContext *>(argument);
    // release使用绝对唤醒时刻，减少服务周期随执行耗时漂移。
    TickType_t release = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&release, pdMS_TO_TICKS(wifi_telemetry::ServicePeriod_ms));
        // xQueuePeek()复制最新快照但不移除队列中的值。
        wifi_telemetry::TelemetrySnapshot snapshot{};
        const wifi_telemetry::TelemetrySnapshot *latest = nullptr;
        if (xQueuePeek(context.telemetry_queue, &snapshot, 0U) == pdPASS) {
            latest = &snapshot;
        }
        wifi_telemetry::Service(latest);
    }
}

#endif

} // namespace freertos_tasks
} // namespace vehicle
