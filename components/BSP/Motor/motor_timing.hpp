#pragma once

#include <cstdint>

namespace vehicle {
namespace motor {

enum class CurrentStage : std::uint8_t {
    not_run, adc, math, before_pwm, pwm, after_pwm, enable, complete
};

// CurrentTiming记录电流采样、计算、PWM写入和数据时效，单位us。
struct CurrentTiming {
    CurrentStage stage{};
    std::int64_t adc_us{}, math_us{}, pwm_us{};
    std::int64_t encoder_age_us{}, current_age_us{};
};

} // namespace motor
} // namespace vehicle
