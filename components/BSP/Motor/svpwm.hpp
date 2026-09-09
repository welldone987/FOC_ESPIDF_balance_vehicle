#pragma once

namespace vehicle::motor {
struct PhaseDuty {
    float a;
    float b;
    float c;
    bool valid;
};
// 第十一课六扇区中心对齐矢量分配；输入V/rad，输出无量纲占空比。
// 超出线性调制区返回无效，调用者必须关闭输出。
PhaseDuty calculateSvpwmDuty(float uq_v, float electrical_angle_rad, float bus_reference_v);
} // namespace vehicle::motor
