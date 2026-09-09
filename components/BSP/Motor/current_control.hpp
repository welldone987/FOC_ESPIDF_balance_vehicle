#pragma once

namespace vehicle::motor {
struct CurrentPiState {
    float integral_v;
    float previous_error_a;
};
struct CurrentPiOutput {
    float requested_v;
    float applied_v;
    bool saturated;
    bool valid;
};
float projectQCurrent(float phase_a_a, float phase_b_a, float electrical_angle_rad);
// A误差、V输出、秒周期；梯形积分与第十一课一致，饱和时禁止继续积累。
CurrentPiOutput updateCurrentPi(CurrentPiState &state, float error_a, float dt_s, float voltage_limit_v);
} // namespace vehicle::motor
