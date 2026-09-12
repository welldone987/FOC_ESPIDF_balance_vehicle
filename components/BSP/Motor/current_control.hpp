#pragma once

namespace vehicle {
namespace motor {
struct CurrentPiState {
    float integral_V;
    float previous_error_A;
};
struct CurrentPiOutput {
    float requested_V;
    float applied_V;
    bool saturated;
    bool valid;
};
float ProjectQCurrent(float phase_a_A, float phase_b_A, float electrical_angle_rad);
// A误差、V输出、秒周期；梯形积分与第十一课一致，饱和时禁止继续积累。
CurrentPiOutput UpdateCurrentPi(CurrentPiState &state, float error_A, float dt_s, float voltage_limit_V);
} // namespace motor
} // namespace vehicle
