#include "diagnostics.hpp"

#include <cstdint>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"

namespace vehicle {
namespace diagnostics {
namespace {

/*
 * 诊断区是纯RAM单例：事件环、首故障与控制现场，全部更新走同一把锁。
 * 状态不对外暴露，生产者与消费者只能使用本文件的API。
 */
struct DiagnosticState {
    // event_seq记录事件序号。
    std::uint32_t event_seq{};
    // first_fault保留独立首故障槽。
    Event first_fault{};
    // events以环形保存最近的EventCapacity条事件。
    Event events[EventCapacity]{};
    // count和next记录环内事件数与下一个写入槽。
    std::uint8_t count{}, next{};
    // last_control和fault_control保存最近与故障前控制快照。
    control::ControlSnapshot last_control{}, fault_control{};
    // last_timing、fault_timing与首轮计时用于时序诊断。
    control::ControlTiming last_timing{}, fault_timing{}, first_loop{};
};

// state是诊断区单例；lock保护其多字段更新。
DiagnosticState state{};
portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;

// Commit()提交一条事件；致命错误且无首故障时同时保存首故障和现场快照。
void Commit(const ErrorInfo &error, bool fatal)
{
    Event event{++state.event_seq, error, static_cast<std::uint8_t>(fatal ? 1 : 0)};
    if (fatal && state.first_fault.event_seq == 0) {
        event.flags |= 2;
        state.first_fault = event;
        state.fault_control = state.last_control;
        state.fault_timing = state.last_timing;
    }
    state.events[state.next] = event;
    state.next = (state.next + 1) % EventCapacity;
    if (state.count < EventCapacity) { ++state.count; }
}

// CommitTiming()保存最近计时，并记录首个完整周期。
void CommitTiming(const control::ControlTiming &timing)
{
    state.last_timing = timing;
    if (timing.stage != control::ControlStage::complete) { return; }
    if (state.first_loop.cycle == 0) { state.first_loop = timing; }
}

// NextEvent()按提交顺序查找序号大于after的最早事件。
bool NextEvent(std::uint32_t after, Event &out)
{
    for (unsigned index=0; index<state.count; ++index) {
        const auto &event = state.events[(state.next + EventCapacity - state.count + index) % EventCapacity];
        if (event.event_seq > after) { out = event; return true; }
    }
    return false;
}

} // namespace

void Initialize() { portENTER_CRITICAL(&lock); state = {}; portEXIT_CRITICAL(&lock); }

void Boot(BootStep step, const char *state, esp_err_t rc)
{
    ESP_LOGI("boot", "boot_step=%u %s rc=%s (0x%x)", static_cast<unsigned>(step), state, esp_err_to_name(rc), rc);
}

void BootSubstep(std::uint16_t point, const char *state)
{
    ESP_LOGI("boot", "boot_substep=0x%03x state=%s", static_cast<unsigned>(point), state);
}

void Record(const ErrorInfo &error, bool fatal)
{ portENTER_CRITICAL(&lock); Commit(error,fatal); portEXIT_CRITICAL(&lock); }

void CommitControlFrame(const control::ControlSnapshot &snapshot, const control::ControlTiming &timing)
{
    portENTER_CRITICAL(&lock);
    state.last_control=snapshot;
    CommitTiming(timing);
    portEXIT_CRITICAL(&lock);
}

void CommitFaultTiming(const control::ControlTiming &timing)
{ portENTER_CRITICAL(&lock); CommitTiming(timing); portEXIT_CRITICAL(&lock); }

bool ReadEvent(std::uint32_t after, Event &event)
{
    portENTER_CRITICAL(&lock);
    const bool found=NextEvent(after,event);
    portEXIT_CRITICAL(&lock);
    return found;
}

bool ReadFirstFault(Event &event)
{
    portENTER_CRITICAL(&lock);
    const bool found=state.first_fault.event_seq != 0;
    if (found) { event=state.first_fault; }
    portEXIT_CRITICAL(&lock);
    return found;
}

void FaultFrame(control::ControlSnapshot &snapshot, control::ControlTiming &timing)
{
    portENTER_CRITICAL(&lock);
    snapshot=state.fault_control; timing=state.fault_timing;
    portEXIT_CRITICAL(&lock);
}

control::ControlTiming FirstLoopTiming()
{
    portENTER_CRITICAL(&lock);
    const auto timing=state.first_loop;
    portEXIT_CRITICAL(&lock);
    return timing;
}

} // namespace diagnostics
} // namespace vehicle
