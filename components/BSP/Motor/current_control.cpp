#include "current_control.hpp"
#include "motor_config.hpp"
#include <algorithm>
#include <cmath>

namespace vehicle {
namespace motor {
float ProjectQCurrent(float phase_a_A, float phase_b_A, float electrical_angle_rad)
{
    // beta_A是Clarke变换后的β轴电流，单位A。
    const float beta_A = (phase_a_A + 2.0f * phase_b_A) * 0.5773502691896258f;
    // 把β轴电流投影到与电角度正交的q轴。
    return beta_A * std::cos(electrical_angle_rad) - phase_a_A * std::sin(electrical_angle_rad);
}
CurrentPiOutput UpdateCurrentPi(CurrentPiState &state, float error_A, float dt_s, float voltage_limit_V)
{
    // 非法输入复位状态，避免错误值继续参与积分。
    if (!std::isfinite(error_A) || !std::isfinite(dt_s) ||
        !std::isfinite(voltage_limit_V) || voltage_limit_V <= 0.0f ||
        dt_s <= 0.0f || dt_s > MaximumControlGap_s ||
        !std::isfinite(state.integral_V) || !std::isfinite(state.previous_error_A)) {
        state = {};
        return {};
    }
    // proportional_V是比例项电压，delta_V是本周期梯形积分增量，单位V。
    const float proportional_V = IqKp_V_per_A * error_A;
    const float delta_V = IqKi_V_per_A_s * dt_s * 0.5f *
        (error_A + state.previous_error_A);
    // candidate_V把积分项限制在电压上限内。
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
