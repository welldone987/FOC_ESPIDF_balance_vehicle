#include "current_control.hpp"
#include "vehicle_config.hpp"
#include <algorithm>
#include <cmath>

namespace vehicle::motor {
float projectQCurrent(float phase_a_a, float phase_b_a, float electrical_angle_rad)
{
    const float beta_a = (phase_a_a + 2.0f * phase_b_a) * 0.5773502691896258f;
    return beta_a * std::cos(electrical_angle_rad) - phase_a_a * std::sin(electrical_angle_rad);
}
CurrentPiOutput updateCurrentPi(CurrentPiState &state, float error_a, float dt_s, float voltage_limit_v)
{
    if (!std::isfinite(error_a) || !std::isfinite(dt_s) ||
        !std::isfinite(voltage_limit_v) || voltage_limit_v <= 0.0f ||
        dt_s <= 0.0f || dt_s > config::kMaximumControlGapS ||
        !std::isfinite(state.integral_v) || !std::isfinite(state.previous_error_a)) {
        state = {};
        return {};
    }
    const float proportional_v = config::kCurrentKpVPerA * error_a;
    const float delta_v = config::kCurrentKiVPerAS * dt_s * 0.5f *
        (error_a + state.previous_error_a);
    const float candidate_v = std::clamp(state.integral_v + delta_v, -voltage_limit_v, voltage_limit_v);
    const float candidate_output_v = proportional_v + candidate_v;
    // 用实际积分增量判断方向，避免误差翻转时梯形积分仍推向饱和。
    if (std::abs(candidate_output_v) <= voltage_limit_v || candidate_output_v * delta_v <= 0.0f) {
        state.integral_v = candidate_v;
    }
    state.integral_v = std::clamp(state.integral_v, -voltage_limit_v, voltage_limit_v);
    state.previous_error_a = error_a;
    const float requested_v = proportional_v + state.integral_v;
    if (!std::isfinite(requested_v)) { state = {}; return {}; }
    return {requested_v, std::clamp(requested_v, -voltage_limit_v, voltage_limit_v),
            std::abs(candidate_output_v) >= voltage_limit_v, true};
}
} // namespace vehicle::motor
