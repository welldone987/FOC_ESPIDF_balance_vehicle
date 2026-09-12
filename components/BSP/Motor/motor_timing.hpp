#pragma once

#include <cstdint>

namespace vehicle {
namespace motor {

// CurrentStage按执行顺序标记电流周期的当前阶段，供故障计时定格。
enum class CurrentStage : std::uint8_t {
    not_run, adc, math, before_pwm, pwm, after_pwm, enable, complete
};

// CurrentTiming记录电流采样、计算、PWM写入和数据时效，单位us。
struct CurrentTiming {
    // stage保存故障定格时已进入的阶段。
    CurrentStage stage{};
    // adc_us、math_us和pwm_us分别记录采样、计算和PWM写入的耗时，单位us。
    std::int64_t adc_us{}, math_us{}, pwm_us{};
    // encoder_age_us和current_age_us记录对应采样时刻到检查点的年龄，单位us。
    std::int64_t encoder_age_us{}, current_age_us{};
};

} // namespace motor
} // namespace vehicle
