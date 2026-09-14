#pragma once
#include <cstdint>

namespace vehicle {
namespace control {
/*
 * balance_controller实现三环级联的外两层。
 * 速度与偏航PI按OuterPeriod_s更新，姿态PD按每个姿态周期执行。
 * 输出target_M0_A/target_M1_A交给BSP电流环。
 */
// 车辆坐标：前进、前倾、右轮更快产生的偏航为正。
struct ControlInput {
    // velocity_M0_rad_s和velocity_M1_rad_s是滤波后的轮角速度，单位rad/s。
    float velocity_M0_rad_s;
    float velocity_M1_rad_s;
    // pitch_rad和pitch_rate_rad_s是姿态估计的俯仰角与角速度，单位rad、rad/s。
    float pitch_rad;
    float pitch_rate_rad_s;
    // throttle_velocity_rad_s和yaw_rate_rad_s是遥控目标，单位rad/s。
    float throttle_velocity_rad_s;
    float yaw_rate_rad_s;
    // driving标记命令新鲜有效时跟踪遥控目标，否则零速平衡。
    bool driving;
};
// ControlOutput保存外环请求与最终电流分配。
struct ControlOutput {
    // target_pitch_rad是速度环要求的目标俯仰角，单位rad。
    float target_pitch_rad;
    // balance_current_A和turn_current_A是平衡与差动电流分量，单位A。
    float balance_current_A;
    float turn_current_A;
    // target_M0_A和target_M1_A是最终每轮电流目标，单位A。
    float target_M0_A;
    float target_M1_A;
    // valid标记本轮输出是否可用于电流环。
    bool valid;
};
// ControllerState保存外环积分、斜坡和饱和标志的跨周期状态。
struct ControllerState {
    // speed_reference_rad_s和yaw_reference_rad_s是斜坡后的目标，单位rad/s。
    float speed_reference_rad_s;
    float yaw_reference_rad_s;
    // speed_integral_rad和yaw_integral_A是速度PI与偏航PI的积分项。
    float speed_integral_rad;
    float yaw_integral_A;
    // outer_elapsed_s累计到下次外环更新的时间，单位s。
    float outer_elapsed_s;
    // target_pitch_rad和turn_request_A保存外环最近一次输出。
    float target_pitch_rad;
    float turn_request_A;
    // balance_saturated和turn_saturated汇总电流环反馈的饱和。
    bool balance_saturated;
    bool turn_saturated;
};
void Initialize(ControllerState &state);
// 每个电流周期汇总饱和，供下一次外环更新冻结积分。
void ObserveCurrentSaturation(ControllerState &state, bool saturated);
// 姿态周期调用。
// 内部累计实测dt，按外环周期更新速度和转向。
ControlOutput Update(ControllerState &state, const ControlInput &input, float dt_s);
} // namespace control
} // namespace vehicle
