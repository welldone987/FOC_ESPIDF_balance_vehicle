#include "application_tasks.hpp"

#include "balance_controller.hpp"
#include "ble_command_service.hpp"
#include "bmi160_attitude.hpp"
#include "motor_foc_service.hpp"
#include "vehicle_config.hpp"

#include <cstdint>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

namespace vehicle {
namespace freertos_tasks {
namespace {

constexpr char kTag[] = "balance_tasks";
// kDiagnosticsPeriodMs定义健康指标日志的输出周期，单位ms。
constexpr std::uint32_t kDiagnosticsPeriodMs = 5000U;

// delayMs把毫秒延时转换为当前FreeRTOS tick延时。
void delayMs(std::uint32_t milliseconds)
{
    vTaskDelay(pdMS_TO_TICKS(milliseconds));
}

// recordControlExecutionTime更新窗口最大值，并记录超过控制周期的执行时间。
void recordControlExecutionTime(TaskRuntime &runtime,
                                std::uint32_t execution_us)
{
    std::uint32_t current_maximum =
        runtime.maximum_control_execution_us.load(std::memory_order_relaxed);
    while (execution_us > current_maximum &&
           !runtime.maximum_control_execution_us.compare_exchange_weak(
               current_maximum, execution_us, std::memory_order_relaxed)) {
    }

    if (execution_us > config::kControlPeriodUs) {
        runtime.control_deadline_overruns.fetch_add(
            1U, std::memory_order_relaxed);
    }
}

// logDiagnosticEvent把队列中的诊断事件转换为串口日志。
void logDiagnosticEvent(const DiagnosticEvent &event)
{
    switch (event.code) {
    case DiagnosticCode::ApplicationStarting:
        ESP_LOGI(kTag, "DengFOC V4 balance controller starting");
        break;
    case DiagnosticCode::PowerReady:
        ESP_LOGI(kTag, "Power ready: %.2f V", static_cast<double>(event.value));
        break;
    case DiagnosticCode::StartupUndervoltage:
        ESP_LOGW(kTag,
                 "Startup voltage too low: %.2f V (threshold %.2f V)",
                 static_cast<double>(event.value),
                 static_cast<double>(config::kStartupUndervoltageThresholdV));
        break;
    case DiagnosticCode::PowerInitializationFailed:
        ESP_LOGE(kTag, "Power initialization failed: %s", esp_err_to_name(event.error));
        break;
    case DiagnosticCode::PowerReadFailed:
        ESP_LOGE(kTag, "Power read failed: %s", esp_err_to_name(event.error));
        break;
    case DiagnosticCode::BleInitializationFailed:
        ESP_LOGE(kTag, "BLE initialization failed: %s", esp_err_to_name(event.error));
        break;
    case DiagnosticCode::ControlTaskCreationFailed:
        ESP_LOGE(kTag, "ControlTask creation failed");
        break;
    case DiagnosticCode::ControlTaskStarting:
        ESP_LOGI(kTag,
                 "ControlTask started on Core 1 at %lu Hz; BMI160 ODR is 1600 Hz",
                 static_cast<unsigned long>(config::kControlRateHz));
        break;
    case DiagnosticCode::ControlTimerFailed:
        ESP_LOGE(kTag, "Control timer failed: %s", esp_err_to_name(event.error));
        break;
    case DiagnosticCode::ImuInitializationFailed:
        ESP_LOGE(kTag, "BMI160 initialization failed: %s", esp_err_to_name(event.error));
        break;
    case DiagnosticCode::MotorInitializationFailed:
        ESP_LOGE(kTag, "Motor initialization failed: %s", esp_err_to_name(event.error));
        break;
    case DiagnosticCode::MotorStateInvalid:
        ESP_LOGE(kTag, "Control stopped: motor/encoder state invalid");
        break;
    case DiagnosticCode::AttitudeInvalid:
        ESP_LOGE(kTag, "Control stopped: BMI160 read or attitude invalid");
        break;
    }
}

// stopControl关闭电机输出、发布故障事件并让控制任务停留在安全状态。
[[noreturn]] void stopControl(TaskRuntime &runtime, DiagnosticCode code)
{
    motor::disableOutputs();
    postDiagnosticEvent(runtime, code);
    for (;;) {
        delayMs(1000U);
    }
}

// controlTimerCallback只向ControlTask累加一次通知，不在ESP timer任务中执行控制计算。
void controlTimerCallback(void *argument)
{
    const TaskHandle_t control = static_cast<TaskHandle_t>(argument);
    if (control != nullptr) {
        xTaskNotifyGive(control);
    }
}

} // namespace

void postDiagnosticEvent(TaskRuntime &runtime,
                         DiagnosticCode code,
                         esp_err_t error,
                         float value)
{
    if (runtime.diagnostics_queue == nullptr) {
        return;
    }

    const DiagnosticEvent event{code, error, value};
    // xQueueSend()使用零等待避免诊断发布阻塞控制路径；满队列事件由计数器记录。
    if (xQueueSend(runtime.diagnostics_queue, &event, 0U) != pdPASS) {
        runtime.dropped_diagnostic_events.fetch_add(
            1U, std::memory_order_relaxed);
    }
}

// DiagnosticsTask在Core 0消费事件，并每5秒输出一次控制与资源健康指标。
void diagnosticsTask(void *argument)
{
    auto &runtime = *static_cast<TaskRuntime *>(argument);
    // last_report_tick标记上一次健康报告的RTOS tick。
    TickType_t last_report_tick = xTaskGetTickCount();
    // previous_cycle_count用于把累计控制周期换算为本报告窗口内的数量。
    std::uint32_t previous_cycle_count = 0U;

    for (;;) {
        DiagnosticEvent event{};
        // 队列等待最多1秒，超时后仍继续检查5秒健康报告周期。
        if (xQueueReceive(
                runtime.diagnostics_queue,
                &event,
                pdMS_TO_TICKS(1000U)) == pdPASS) {
            logDiagnosticEvent(event);
        }

        const TickType_t now = xTaskGetTickCount();
        if ((now - last_report_tick) < pdMS_TO_TICKS(kDiagnosticsPeriodMs)) {
            continue;
        }
        last_report_tick = now;

        const std::uint32_t cycles =
            runtime.control_cycle_count.load(std::memory_order_relaxed);
        const std::uint32_t cycle_delta = cycles - previous_cycle_count;
        previous_cycle_count = cycles;

        const TaskHandle_t control =
            runtime.control_task_handle.load(std::memory_order_acquire);
        const UBaseType_t control_stack_free =
            control == nullptr ? 0U : uxTaskGetStackHighWaterMark(control);
        const UBaseType_t diagnostics_stack_free =
            uxTaskGetStackHighWaterMark(nullptr);
        // exchange()读取本窗口最大执行时间并为下一个窗口清零。
        const std::uint32_t maximum_execution_us =
            runtime.maximum_control_execution_us.exchange(
                0U, std::memory_order_relaxed);

        ESP_LOGI(kTag,
                 "health: cycles/5s=%lu missed=%lu overruns=%lu max_exec_us=%lu "
                 "heap_free=%u heap_min=%u "
                 "control_stack_free=%u diagnostics_stack_free=%u dropped_logs=%lu",
                 static_cast<unsigned long>(cycle_delta),
                 static_cast<unsigned long>(
                     runtime.missed_control_releases.load(std::memory_order_relaxed)),
                 static_cast<unsigned long>(
                     runtime.control_deadline_overruns.load(std::memory_order_relaxed)),
                 static_cast<unsigned long>(maximum_execution_us),
                 static_cast<unsigned int>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)),
                 static_cast<unsigned int>(heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL)),
                 static_cast<unsigned int>(control_stack_free),
                 static_cast<unsigned int>(diagnostics_stack_free),
                 static_cast<unsigned long>(
                     runtime.dropped_diagnostic_events.load(std::memory_order_relaxed)));
    }
}

// ControlTask在初始化外设后按定时器通知串行执行FOC、传感器读取和控制计算。
void controlTask(void *argument)
{
    auto &runtime = *static_cast<TaskRuntime *>(argument);
    postDiagnosticEvent(runtime, DiagnosticCode::ControlTaskStarting);

    // controller由ControlTask栈构造，并在各控制周期之间保留控制器内部状态。
    control::BalanceController controller{};

    esp_err_t result = imu::initialize();
    if (result != ESP_OK) {
        postDiagnosticEvent(runtime, DiagnosticCode::ImuInitializationFailed, result);
        vTaskSuspend(nullptr);
    }

    result = motor::initialize();
    if (result != ESP_OK) {
        postDiagnosticEvent(runtime, DiagnosticCode::MotorInitializationFailed, result);
        vTaskSuspend(nullptr);
    }

    imu::resetEstimator();

    const TaskHandle_t current_task = xTaskGetCurrentTaskHandle();
    esp_timer_create_args_t timer_arguments{};
    timer_arguments.callback = controlTimerCallback;
    timer_arguments.arg = current_task;
    timer_arguments.dispatch_method = ESP_TIMER_TASK;
    timer_arguments.name = "control_release";
    timer_arguments.skip_unhandled_events = false;

    esp_timer_handle_t control_timer = nullptr;
    result = esp_timer_create(&timer_arguments, &control_timer);
    if (result == ESP_OK) {
        // control_timer每1000us释放一次通知；这是软件控制周期，不是BMI160的ODR。
        result = esp_timer_start_periodic(control_timer, config::kControlPeriodUs);
    }
    if (result != ESP_OK) {
        postDiagnosticEvent(runtime, DiagnosticCode::ControlTimerFailed, result);
        motor::disableOutputs();
        vTaskSuspend(nullptr);
    }

    for (;;) {
        // 每次定时器回调累加一次通知；pdTRUE取出并清零，额外通知表示错过了释放。
        const std::uint32_t pending_releases =
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (pending_releases > 1U) {
            runtime.missed_control_releases.fetch_add(
                pending_releases - 1U, std::memory_order_relaxed);
        }

        // cycle_start_us用于测量本次控制链的执行时间，单位us。
        const std::int64_t cycle_start_us = esp_timer_get_time();
        // 该调用先让FOC和move()消耗上一轮stageTarget()写入的左右电机目标，再返回当前轮速。
        const motor::WheelState wheels = motor::runFocAndReadWheelState();
        if (!wheels.valid) {
            stopControl(runtime, DiagnosticCode::MotorStateInvalid);
        }

        // BMI160的1600Hz ODR只刷新器件数据寄存器；每个1ms软件周期读取一次不能证明消费了每个ODR样本。
        const imu::AttitudeSample attitude = imu::readAttitude();
        if (!attitude.valid) {
            stopControl(runtime, DiagnosticCode::AttitudeInvalid);
        }

        const ble::CommandSnapshot command = ble::latestCommand();
        const control::ControlOutput output = controller.update(
            control::ControlInput{
                wheels.left_velocity_rad_s,
                wheels.right_velocity_rad_s,
                attitude.pitch_deg,
                command.steering_voltage_v,
                command.throttle_velocity_rad_s,
            });

        // stageTarget()把本轮控制输出保存为下一轮runFocAndReadWheelState()使用的目标。
        motor::stageTarget(motor::VoltageCommand{
            output.left_target_v,
            output.right_target_v,
        });

        // execution_us记录从FOC读取到目标暂存完成的控制链耗时，单位us。
        const std::int64_t execution_us = esp_timer_get_time() - cycle_start_us;
        recordControlExecutionTime(
            runtime, static_cast<std::uint32_t>(execution_us));
        // control_cycle_count只统计已完成并记录执行时间的控制周期。
        runtime.control_cycle_count.fetch_add(1U, std::memory_order_relaxed);
    }
}

} // namespace freertos_tasks
} // namespace vehicle
