#include "diagnostics.hpp"
#include <cstdio>
#include <cstring>

#include "esp_attr.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

COREDUMP_DRAM_ATTR vehicle::diagnostics::CrashState g_diag_crash{};

namespace vehicle {
namespace diagnostics {
namespace {

portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;

const char *StageName(control::ControlStage stage)
{
    constexpr const char *names[]={"WAIT","COMMAND","OUTPUTS_OFF","ENCODER","IMU","OUTER","CURRENT","PUBLISH","COMPLETE"};
    const auto index=static_cast<unsigned>(stage);
    return index < sizeof(names)/sizeof(names[0]) ? names[index] : "UNKNOWN";
}

const char *CurrentStageName(motor::CurrentStage stage)
{
    constexpr const char *names[]={"NOT_RUN","ADC","MATH","BEFORE_PWM","PWM","AFTER_PWM","ENABLE","COMPLETE"};
    const auto index=static_cast<unsigned>(stage);
    return index < sizeof(names)/sizeof(names[0]) ? names[index] : "UNKNOWN";
}

// PrintTiming()仅在低频观察者或输出禁能后的故障分支格式化控制现场。
void PrintTiming(const control::ControlTiming &timing)
{
    ESP_LOGI("control_diag","CONTROL_TIMING cycle=%lu balance_cycle=%lu balancing=%u driving=%u starting=%u imu_updated=%u first_release=%u notify=%lu skipped=%lu",
        static_cast<unsigned long>(timing.cycle),static_cast<unsigned long>(timing.balance_cycle),timing.balancing,timing.driving,timing.starting,timing.imu_updated,timing.first_release,
        static_cast<unsigned long>(timing.notifications),static_cast<unsigned long>(timing.skipped_releases));
    ESP_LOGI("control_diag","CONTROL_STAGE stage=%s current_stage=%s dt_us=%lld elapsed_us=%lld off_us=%lld encoder_us=%lld imu_us=%lld outer_us=%lld",
        StageName(timing.stage),CurrentStageName(timing.current.stage),static_cast<long long>(timing.dt_us),static_cast<long long>(timing.elapsed_us),
        static_cast<long long>(timing.outputs_off_us),static_cast<long long>(timing.encoder_us),static_cast<long long>(timing.imu_us),static_cast<long long>(timing.outer_us));
    ESP_LOGI("control_diag","CONTROL_OUTPUT adc_us=%lld math_us=%lld pwm_us=%lld encoder_age_us=%lld current_age_us=%lld",
        static_cast<long long>(timing.current.adc_us),static_cast<long long>(timing.current.math_us),static_cast<long long>(timing.current.pwm_us),
        static_cast<long long>(timing.current.encoder_age_us),static_cast<long long>(timing.current.current_age_us));
}

} // namespace

void Initialize() { portENTER_CRITICAL(&lock); g_diag_crash = {}; portEXIT_CRITICAL(&lock); }

void Boot(BootStep step, const char *state, esp_err_t rc)
{
    portENTER_CRITICAL(&lock); g_diag_crash.boot_step=step; portEXIT_CRITICAL(&lock);
    ESP_LOGI("boot", "boot_step=%u %s rc=%s (0x%x)", static_cast<unsigned>(step), state, esp_err_to_name(rc), rc);
}

void Record(const ErrorInfo &error, bool fatal)
{ portENTER_CRITICAL(&lock); Commit(g_diag_crash,error,fatal); portEXIT_CRITICAL(&lock); }

bool ReadEvent(std::uint32_t after, Event &event, bool first_fault)
{
    portENTER_CRITICAL(&lock);
    const bool have_fault=first_fault && g_diag_crash.first_fault.event_seq!=0;
    const bool found=have_fault || NextEvent(g_diag_crash,after,event);
    if (have_fault) { event=g_diag_crash.first_fault; }
    portEXIT_CRITICAL(&lock);
    return found;
}

int FormatEvent(const Event &event, char *buffer, std::size_t capacity)
{
    const auto &e=event.error;
    const char *file=e.file ? e.file : "?";
    const char *base=std::strrchr(file,'/');
    if (base) { file=base+1; }
    base=std::strrchr(file,'\\');
    if (base) { file=base+1; }
    return std::snprintf(buffer,capacity,
        "seq=%lu flags=%u point=0x%04x code=%ld name=%s domain=%u raw=%ld file=%s line=%lu function=%s valid=%u value=%g threshold=%g channel=%d comparison=%d",
        static_cast<unsigned long>(event.event_seq),event.flags,static_cast<unsigned>(e.point_id),
        static_cast<long>(e.code),esp_err_to_name(e.code),static_cast<unsigned>(e.domain),static_cast<long>(e.raw_code),
        file,static_cast<unsigned long>(e.line),e.function ? e.function : "?",e.valid_fields,
        static_cast<double>(e.value),static_cast<double>(e.threshold),e.channel,e.comparison);
}

void CommitControlSnapshot(const ControlSnapshot &snapshot)
{ portENTER_CRITICAL(&lock); g_diag_crash.last_control=snapshot; portEXIT_CRITICAL(&lock); }

void CompleteBoot()
{ portENTER_CRITICAL(&lock); g_diag_crash.boot_complete=true; portEXIT_CRITICAL(&lock); }

void CommitControlTiming(const control::ControlTiming &timing)
{ portENTER_CRITICAL(&lock); CommitTiming(g_diag_crash,timing); portEXIT_CRITICAL(&lock); }

void PrintControlFault(const ErrorInfo &error)
{
    control::ControlTiming timing{};
    ControlSnapshot previous{};
    portENTER_CRITICAL(&lock);
    timing=g_diag_crash.fault_timing; previous=g_diag_crash.fault_control;
    portEXIT_CRITICAL(&lock);
    ESP_LOGE("control_diag","CONTROL_FAULT point=%u code=%s (0x%x) domain=%u raw=%ld file=%s:%lu function=%s",
        static_cast<unsigned>(error.point_id),esp_err_to_name(error.code),error.code,static_cast<unsigned>(error.domain),
        static_cast<long>(error.raw_code),error.file ? error.file : "?",static_cast<unsigned long>(error.line),error.function ? error.function : "?");
    ESP_LOGE("control_diag","CONTROL_ERROR_FIELDS valid=0x%x value=%g threshold=%g channel=%d comparison=%d",
        error.valid_fields,static_cast<double>(error.value),static_cast<double>(error.threshold),error.channel,error.comparison);
    PrintTiming(timing);
    ESP_LOGI("control_diag","CONTROL_LAST_VALID valid=%u seq=%lu pitch_deg=%g velocity_M0_rad_s=%g velocity_M1_rad_s=%g target_M0_A=%g target_M1_A=%g",
        previous.valid,static_cast<unsigned long>(previous.sequence),static_cast<double>(previous.pitch_deg),
        static_cast<double>(previous.velocity_M0),static_cast<double>(previous.velocity_M1),
        static_cast<double>(previous.target_M0),static_cast<double>(previous.target_M1));
    ESP_LOGE("control_diag","CONTROL_SUMMARY FAIL; stop latched, restart required; BLE diagnostic history retained");
}

void ObserveControlStart()
{
    bool reported_loop=false;
    for (;;) {
        control::ControlTiming first_loop{},first_balance{};
        bool fault=false;
        portENTER_CRITICAL(&lock);
        first_loop=g_diag_crash.first_loop; first_balance=g_diag_crash.first_balance;
        fault=g_diag_crash.first_fault.event_seq != 0;
        portEXIT_CRITICAL(&lock);
        if (first_loop.cycle != 0 && !reported_loop) {
            ESP_LOGI("control_diag","CONTROL_LOOP_ALIVE first frame completed; first dt is nominal, not measured");
            PrintTiming(first_loop);
            reported_loop=true;
        }
        if (first_balance.cycle != 0) {
            ESP_LOGI("control_diag","CONTROL_BALANCE_ACTIVE first balance frame completed; remote_active=%u; this is not a stability validation",first_balance.driving);
            if (first_balance.cycle != first_loop.cycle) { PrintTiming(first_balance); }
            return;
        }
        if (fault) { return; }
        vTaskDelay(pdMS_TO_TICKS(ObservePeriod_ms));
    }
}

} // namespace diagnostics
} // namespace vehicle
