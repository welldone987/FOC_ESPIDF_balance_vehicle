#pragma once

#include <cstddef>
#include <cstdint>

#include "control_snapshot.hpp"
#include "control_timing.hpp"
#include "diagnostics_config.hpp"
#include "error_config.hpp"

namespace vehicle {
namespace diagnostics {
/*
 * diagnostics保存RAM诊断区与生产者/读取原语，是诊断状态的唯一所有者。
 * 功能传递走函数返回的esp_err_t；ErrorInfo为旁路详情，只服务定位。
 * 消费实现（diagnostic_reader/diagnostic_sink）只通过本文件的API读写。
 */

// BootStep按启动顺序标记当前初始化阶段。
// 保持既有编号不重排：原storage=2与随coredump删除的core_dump=6保留空缺，power=3、ble=7起的编号维持原值。
enum class BootStep : std::uint16_t {
    safe_output=1, power=3, voltage, nvs, ble=7,
    wifi, imu, motor, outputs_off, timer, complete
};

// Event保存一条ErrorInfo及其标志位（bit0 fatal、bit1 首故障标记）。
struct Event { std::uint32_t event_seq{}; ErrorInfo error{}; std::uint8_t flags{}; };

// Initialize()清空RAM诊断区，供启动早期调用。
void Initialize();
// Boot()输出一行启动步骤日志。
void Boot(BootStep step, const char *state, esp_err_t rc=ESP_OK);
// BootSubstep()输出电机子步骤的ErrorPoint编号与状态（0x3xx）。
void BootSubstep(std::uint16_t point, const char *state);
// Record()在短临界区内提交一条诊断事件。
void Record(const ErrorInfo &error, bool fatal=false);
// CommitControlFrame()保存一次完成周期的快照与计时（单次临界区）。
void CommitControlFrame(const control::ControlSnapshot &snapshot, const control::ControlTiming &timing);
// CommitFaultTiming()仅更新计时，供故障路径定格失败阶段。
void CommitFaultTiming(const control::ControlTiming &timing);
// ReadEvent()按序号读取下一个环形事件。
bool ReadEvent(std::uint32_t after, Event &event);
// ReadFirstFault()读取独立首故障；不存在时返回false。
bool ReadFirstFault(Event &event);
// FaultFrame()复制故障前控制快照与失败周期计时。
void FaultFrame(control::ControlSnapshot &snapshot, control::ControlTiming &timing);
// FirstLoopTiming()复制首个完整周期的计时；未完成时cycle为0。
control::ControlTiming FirstLoopTiming();

} // namespace diagnostics
} // namespace vehicle
