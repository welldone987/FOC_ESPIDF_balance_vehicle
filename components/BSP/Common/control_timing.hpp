#pragma once
#include <cstdint>

namespace vehicle {
// 定长现场，单位统一为us；只有控制所有者写入，完成/故障时复制到诊断区。
enum class ControlStage : std::uint8_t { waiting, command, outputs_off, encoder, imu, outer, current, publish, complete };
enum class CurrentStage : std::uint8_t { not_run, adc, math, before_pwm, pwm, after_pwm, enable, complete };
struct CurrentTiming {
    CurrentStage stage{};
    std::int64_t adc_us{}, math_us{}, pwm_us{};
    std::int64_t encoder_age_us{}, current_age_us{};
};
struct ControlTiming {
    std::uint32_t cycle{}, balance_cycle{}, notifications{}, skipped_releases{};
    ControlStage stage{};
    bool balancing{}, driving{}, starting{}, imu_updated{}, first_release{};
    std::int64_t dt_us{}, elapsed_us{}, outputs_off_us{}, encoder_us{}, imu_us{}, outer_us{};
    CurrentTiming current{};
};
// 首轮没有上一采样时刻，使用明确的初始化周期；随后必须使用实际间隔。
constexpr std::int64_t sampleInterval(std::int64_t now, std::int64_t previous, std::int64_t initial)
{ return previous == 0 ? initial : now - previous; }
}
