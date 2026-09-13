#pragma once

namespace vehicle {
namespace motor {

// PhaseDuty保存三相占空比和有效性，占空比范围0~1。
struct PhaseDuty {
    // a、b、c是三相占空比。
    float a;
    float b;
    float c;
    // valid为false时调用者必须关闭输出。
    bool valid;
};
// 第十一课六扇区中心对齐矢量分配。
// 输入Uq/电角度/母线参考V，输出无量纲占空比。
// 超出线性调制区返回无效，调用者必须关闭输出。
PhaseDuty CalculateSvpwmDuty(float uq_V, float electrical_angle_rad, float bus_reference_V);
} // namespace motor
} // namespace vehicle
