#pragma once

#include <atomic>
#include <cstdint>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

/*
 * FreeRTOS应用任务模块把BLE命令、BMI160姿态、轮速和电机目标串成控制任务数据流。
 * ControlTask在Core 1按1 ms周期读取状态并计算下一控制周期的电机目标。
 * DiagnosticsTask在Core 0消费有界诊断队列并汇总控制运行时指标。
 */
namespace vehicle {
namespace freertos_tasks {

// ControlTask固定在Core 1，拥有BMI160、轮速和电机控制链的调用上下文。
inline constexpr BaseType_t kControlCore = 1;
// DiagnosticsTask固定在Core 0，负责诊断事件消费和健康日志。
inline constexpr BaseType_t kDiagnosticsCore = 0;
// kControlPriority使控制任务优先于低频DiagnosticsTask运行。
inline constexpr UBaseType_t kControlPriority = 20U;
// kDiagnosticsPriority为诊断任务提供低于控制任务的调度优先级。
inline constexpr UBaseType_t kDiagnosticsPriority = 3U;
// kControlStackSizeBytes为静态ControlTask栈保留8192字节容量。
inline constexpr std::uint32_t kControlStackSizeBytes = 8192U;
// kDiagnosticsStackSizeBytes为静态DiagnosticsTask栈保留4096字节容量。
inline constexpr std::uint32_t kDiagnosticsStackSizeBytes = 4096U;
// kDiagnosticsQueueLength限定待处理诊断事件数量；队列满时发布方累计丢弃计数。
inline constexpr UBaseType_t kDiagnosticsQueueLength = 16U;

// DiagnosticCode标识启动、控制初始化和运行时故障的诊断事件类型。
enum class DiagnosticCode : std::uint8_t {
    ApplicationStarting,
    PowerReady,
    StartupUndervoltage,
    PowerInitializationFailed,
    PowerReadFailed,
    BleInitializationFailed,
    ControlTaskCreationFailed,
    ControlTaskStarting,
    ControlTimerFailed,
    ImuInitializationFailed,
    MotorInitializationFailed,
    MotorStateInvalid,
    AttitudeInvalid,
};

// DiagnosticEvent把事件类型、ESP-IDF错误码和可选测量值送入诊断队列。
struct DiagnosticEvent {
    // code描述事件所属的诊断状态。
    DiagnosticCode code;
    // error保存产生事件的ESP-IDF返回码。
    esp_err_t error;
    // value保存事件关联的数值，例如启动母线电压，单位V。
    float value;
};

// TaskRuntime保存两个任务共享的队列句柄、任务句柄和运行时计数器。
struct TaskRuntime {
    // diagnostics_queue把启动/控制任务事件交给DiagnosticsTask消费。
    QueueHandle_t diagnostics_queue{nullptr};
    // control_task_handle保存静态创建后的ControlTask句柄，供健康报告读取栈余量。
    std::atomic<TaskHandle_t> control_task_handle{nullptr};
    // control_cycle_count累计已经完成的控制周期数量。
    std::atomic<std::uint32_t> control_cycle_count{0U};
    // missed_control_releases累计一次唤醒中合并掉的额外定时器释放次数。
    std::atomic<std::uint32_t> missed_control_releases{0U};
    // control_deadline_overruns累计执行时间超过kControlPeriodUs的控制周期数。
    std::atomic<std::uint32_t> control_deadline_overruns{0U};
    // maximum_control_execution_us保存当前健康报告窗口内的最大执行时间，单位us。
    std::atomic<std::uint32_t> maximum_control_execution_us{0U};
    // dropped_diagnostic_events累计诊断队列满导致的非阻塞发布失败次数。
    std::atomic<std::uint32_t> dropped_diagnostic_events{0U};
};

// 以非阻塞方式发布诊断事件；队列满时只累计丢弃计数。
void postDiagnosticEvent(TaskRuntime &runtime,
                         DiagnosticCode code,
                         esp_err_t error = ESP_OK,
                         float value = 0.0f);
// ControlTask按定时器通知驱动BMI160、轮速和控制器，并暂存下一周期电机目标。
void controlTask(void *argument);
// DiagnosticsTask消费事件队列并周期性输出控制健康指标。
void diagnosticsTask(void *argument);

} // namespace freertos_tasks
} // namespace vehicle
