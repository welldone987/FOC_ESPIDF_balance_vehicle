#pragma once

namespace vehicle {
namespace motor {

// CurrentPiState保存Iq电流PI的跨周期状态。
struct CurrentPiState {
    // integral_V保存积分项电压，单位V。
    float integral_V;
    // previous_error_A保存上一周期的电流误差，单位A。
    float previous_error_A;
};
// CurrentPiOutput保存本周期的PI请求与限幅结果。
struct CurrentPiOutput {
    // requested_V是未限幅的PI输出，单位V。
    float requested_V;
    // applied_V是限幅后的实际输出电压，单位V。
    float applied_V;
    // saturated标记PI输出是否达到电压上限。
    bool saturated;
    // valid标记本周期结果是否可用于SVPWM。
    bool valid;
};
// ProjectQCurrent()用两相电流和电角度计算Iq反馈，单位A。
float ProjectQCurrent(float phase_a_A, float phase_b_A, float electrical_angle_rad);
// A误差、V输出、秒周期。
// 梯形积分与第十一课一致，饱和时禁止继续积累。
CurrentPiOutput UpdateCurrentPi(CurrentPiState &state, float error_A, float dt_s, float voltage_limit_V);
} // namespace motor
} // namespace vehicle
