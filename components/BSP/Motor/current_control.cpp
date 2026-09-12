#include "current_control.hpp"
#include "motor_config.hpp"
#include <algorithm>
#include <cmath>

namespace vehicle {
namespace motor {
float ProjectQCurrent(float phase_a_A, float phase_b_A, float electrical_angle_rad)
{
    const float beta_A = (phase_a_A + 2.0f * phase_b_A) * 0.5773502691896258f;
    return beta_A * std::cos(electrical_angle_rad) - phase_a_A * std::sin(electrical_angle_rad);
}
CurrentPiOutput UpdateCurrentPi(CurrentPiState &state, float error_A, float dt_s, float voltage_limit_V)
{
    if (!std::isfinite(error_A) || !std::isfinite(dt_s) ||
        !std::isfinite(voltage_limit_V) || voltage_limit_V <= 0.0f ||
        dt_s <= 0.0f || dt_s > MaximumControlGap_s ||
        !std::isfinite(state.integral_V) || !std::isfinite(state.previous_error_A)) {
        state = {};
        return {};
    }
    const float proportional_V = IqKp_V_per_A * error_A;
    const float delta_V = IqKi_V_per_A_s * dt_s * 0.5f *
        (error_A + state.previous_error_A);
    const float candidate_V = std::clamp(state.integral_V + delta_V, -voltage_limit_V, voltage_limit_V);
    const float candidate_output_V = proportional_V + candidate_V;
    // 用实际积分增量判断方向，避免误差翻转时梯形积分仍推向饱和。
    if (std::abs(candidate_output_V) <= voltage_limit_V || candidate_output_V * delta_V <= 0.0f) {
        state.integral_V = candidate_V;
    }
    state.integral_V = std::clamp(state.integral_V, -voltage_limit_V, voltage_limit_V);
    state.previous_error_A = error_A;
    const float requested_V = proportional_V + state.integral_V;
    if (!std::isfinite(requested_V)) { state = {}; return {}; }
    return {requested_V, std::clamp(requested_V, -voltage_limit_V, voltage_limit_V),
            std::abs(candidate_output_V) >= voltage_limit_V, true};
}
} // namespace motor
} // namespace vehicle
