#pragma once

namespace vehicle::control {
// 车辆坐标：前进、前倾、右轮更快产生的偏航为正。
struct ControlInput {
    float left_velocity_rad_s;
    float right_velocity_rad_s;
    float pitch_deg;
    float pitch_rate_deg_s;
    float throttle_velocity_rad_s;
    float yaw_rate_rad_s;
    bool driving;
};
struct ControlOutput {
    float target_pitch_deg;
    float balance_current_a;
    float turn_current_a;
    float left_target_a;
    float right_target_a;
};
// 仅由ControlTask持有；电流PI状态属于BSP电机对象。
struct ControllerState {
    float speed_reference;
    float yaw_reference;
    float speed_integral;
    float yaw_integral;
    float outer_elapsed_s;
    float target_pitch_deg;
    float turn_request_a;
    bool balance_saturated;
    bool turn_saturated;
};
void initialize(ControllerState &state);
// 姿态周期调用；内部累计实测dt，按外环周期更新速度和转向。
ControlOutput update(ControllerState &state, const ControlInput &input, float dt_s);
} // namespace vehicle::control
