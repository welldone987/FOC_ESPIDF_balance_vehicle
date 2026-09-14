#include "diagnostic_sink.hpp"

#include <cstdint>

#include "control_timing.hpp"
#include "diagnostic_text.hpp"
#include "diagnostics.hpp"
#include "diagnostics_config.hpp"

#include "esp_log.h"

namespace vehicle {
namespace diagnostics {
namespace sink {
namespace {

// serial_cursor和next_serial_us分别记录串口输出进度与下次允许输出的时刻。
std::uint32_t serial_cursor{};
std::int64_t next_serial_us{};
// first_loop_reported标记启动状态行是否已输出。
bool first_loop_reported=false;

// StageName()把ControlStage映射为固定诊断文本。
const char *StageName(control::ControlStage stage)
{
    constexpr const char *names[]={"WAIT","COMMAND","OUTPUTS_OFF","ENCODER","IMU","OUTER","CURRENT","PUBLISH","COMPLETE"};
    const auto index=static_cast<unsigned>(stage);
    return index < sizeof(names)/sizeof(names[0]) ? names[index] : "UNKNOWN";
}

// CurrentStageName()把CurrentStage映射为固定诊断文本。
const char *CurrentStageName(motor::CurrentStage stage)
{
    constexpr const char *names[]={"NOT_RUN","ADC","MATH","BEFORE_PWM","PWM","AFTER_PWM","ENABLE","COMPLETE"};
    const auto index=static_cast<unsigned>(stage);
    return index < sizeof(names)/sizeof(names[0]) ? names[index] : "UNKNOWN";
}

// PrintTiming()仅在低频观察者或输出禁能后的故障分支格式化控制现场。
void PrintTiming(const control::ControlTiming &timing)
{
    ESP_LOGI("control_diag","CONTROL_TIMING cycle=%lu driving=%u imu_updated=%u first_release=%u notify=%lu skipped=%lu",
        static_cast<unsigned long>(timing.cycle),timing.driving,timing.imu_updated,timing.first_release,
        static_cast<unsigned long>(timing.notifications),static_cast<unsigned long>(timing.skipped_releases));
    ESP_LOGI("control_diag","CONTROL_STAGE stage=%s current_stage=%s dt_us=%lld elapsed_us=%lld encoder_us=%lld imu_us=%lld outer_us=%lld",
        StageName(timing.stage),CurrentStageName(timing.current.stage),static_cast<long long>(timing.dt_us),static_cast<long long>(timing.elapsed_us),
        static_cast<long long>(timing.encoder_us),static_cast<long long>(timing.imu_us),static_cast<long long>(timing.outer_us));
    ESP_LOGI("control_diag","CONTROL_OUTPUT adc_us=%lld math_us=%lld pwm_us=%lld encoder_age_us=%lld current_age_us=%lld",
        static_cast<long long>(timing.current.adc_us),static_cast<long long>(timing.current.math_us),static_cast<long long>(timing.current.pwm_us),
        static_cast<long long>(timing.current.encoder_age_us),static_cast<long long>(timing.current.current_age_us));
}

} // namespace

void ReportSerial(std::int64_t now_us)
{
    if (now_us < next_serial_us) { return; }
    next_serial_us=now_us+SerialReportPeriod_us;
    // 首个完整周期只报告一次，替代app_main的启动观察者。
    if (!first_loop_reported) {
        const auto first_loop=FirstLoopTiming();
        if (first_loop.cycle != 0) {
            first_loop_reported=true;
            ESP_LOGI("control_diag","CONTROL_LOOP_ALIVE first frame completed; first dt is nominal, not measured");
            PrintTiming(first_loop);
        }
    }
    Event event{};
    if (ReadEvent(serial_cursor,event)) {
        char text[DiagnosticTextCapacity]{};
        FormatEvent(event,text,sizeof(text));
        ESP_LOGW("diagnostic","%s",text);
        serial_cursor=event.event_seq;
    }
}

void PrintControlFault(const ErrorInfo &error)
{
    control::ControlSnapshot previous{};
    control::ControlTiming timing{};
    FaultFrame(previous,timing);
    // 与BLE诊断特征共用同一渲染器，串口不再维护第二套字段。
    char text[DiagnosticTextCapacity]{};
    const Event fault_event{0U,error,3U};
    FormatEvent(fault_event,text,sizeof(text));
    ESP_LOGE("control_diag","CONTROL_FAULT %s",text);
    PrintTiming(timing);
    ESP_LOGI("control_diag","CONTROL_LAST_VALID valid=%u seq=%lu pitch_deg=%g velocity_M0_rad_s=%g velocity_M1_rad_s=%g target_M0_A=%g target_M1_A=%g",
        previous.valid,static_cast<unsigned long>(previous.sequence),static_cast<double>(previous.pitch_deg),
        static_cast<double>(previous.velocity_M0_rad_s),static_cast<double>(previous.velocity_M1_rad_s),
        static_cast<double>(previous.target_M0_A),static_cast<double>(previous.target_M1_A));
    ESP_LOGE("control_diag","CONTROL_SUMMARY FAIL; stop latched, restart required; BLE diagnostic history retained");
}

} // namespace sink
} // namespace diagnostics
} // namespace vehicle
