#pragma once
#include <cstdint>

namespace vehicle {
namespace control {
// 车辆坐标：前进、前倾、右轮更快产生的偏航为正。
struct ControlInput {
    float velocity_M0_rad_s;
    float velocity_M1_rad_s;
    float pitch_rad;
    float pitch_rate_rad_s;
    float throttle_velocity_rad_s;
    float yaw_rate_rad_s;
    bool balancing;
    bool driving; // 命令新鲜有效时跟踪遥控目标，否则零速平衡。
};
struct ControlOutput {
    float target_pitch_rad;
    float balance_current_A;
    float turn_current_A;
    float target_M0_A;
    float target_M1_A;
    bool valid;
};
// 仅由ControlTask持有；电流PI状态属于BSP电机对象。
struct ControllerState {
    float speed_reference_rad_s;
    float yaw_reference_rad_s;
    float speed_integral_rad;
    float yaw_integral_A;
    float outer_elapsed_s;
    float target_pitch_rad;
    float turn_request_A;
    bool balance_saturated;
    bool turn_saturated;
};
void Initialize(ControllerState &state);
// 每个电流周期汇总饱和，供下一次外环更新冻结积分。
void ObserveCurrentSaturation(ControllerState &state, bool saturated);
// 姿态周期调用；内部累计实测dt，按外环周期更新速度和转向。
ControlOutput Update(ControllerState &state, const ControlInput &input, float dt_s);
} // namespace control
} // namespace vehicle
