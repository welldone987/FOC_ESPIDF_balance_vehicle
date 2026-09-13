#include "svpwm.hpp"
#include <algorithm>
#include <cmath>

namespace vehicle {
namespace motor {
PhaseDuty CalculateSvpwmDuty(float uq_V, float electrical_angle_rad, float bus_reference_V)
{
    // Pi和Sqrt3分别用于相位归一化和调制比计算。
    constexpr float Pi = 3.14159265358979323846f;
    constexpr float Sqrt3 = 1.7320508075688772f;
    // RoundingTolerance吸收浮点舍入误差，避免有效扇区被误判为超限。
    constexpr float RoundingTolerance = 2.0e-6f;
    if (!std::isfinite(uq_V) || !std::isfinite(electrical_angle_rad) ||
        !std::isfinite(bus_reference_V) || bus_reference_V <= 0.0f ||
        std::abs(uq_V) > bus_reference_V / Sqrt3) { return {}; }
    // 负Uq反转矢量，+Pi/2将转子d轴角转换为q轴电压矢量角。
    float angle_rad = std::fmod(electrical_angle_rad, 2.0f * Pi) + Pi / 2.0f;
    if (uq_V < 0.0f) { angle_rad += Pi; }
    angle_rad = std::fmod(angle_rad, 2.0f * Pi);
    if (angle_rad < 0.0f) { angle_rad += 2.0f * Pi; }
    // sector是把矢量角归一化到1~6后的扇区号。
    const int sector = std::min(6, static_cast<int>(angle_rad / (Pi / 3.0f)) + 1);
    // amplitude_ratio是Uq相对母线参考的调制比。
    const float amplitude_ratio = Sqrt3 * std::abs(uq_V) / bus_reference_V;
    // 两个基本矢量按正弦扇区位置分配作用比例。
    const float active_vector_1_ratio = amplitude_ratio * std::sin(sector * Pi / 3.0f - angle_rad);
    const float active_vector_2_ratio = amplitude_ratio * std::sin(angle_rad - (sector - 1) * Pi / 3.0f);
    // zero_vector_ratio为负表示超出线性调制区。
    const float zero_vector_ratio = 1.0f - active_vector_1_ratio - active_vector_2_ratio;
    if (active_vector_1_ratio < -RoundingTolerance || active_vector_2_ratio < -RoundingTolerance ||
        zero_vector_ratio < -RoundingTolerance) { return {}; }
    // x、y、z复用第十一课六扇区表的查表参数。
    const float x = active_vector_1_ratio;
    const float y = active_vector_2_ratio;
    const float z = zero_vector_ratio * 0.5f;
    PhaseDuty duty{};
    // 保留例程的六扇区表。
    // 两个电机共用此纯函数，PWM输出对象相互独立。
    switch (sector) {
    case 1: duty = {x + y + z, y + z, z, true}; break;
    case 2: duty = {x + z, x + y + z, z, true}; break;
    case 3: duty = {z, x + y + z, y + z, true}; break;
    case 4: duty = {z, x + z, x + y + z, true}; break;
    case 5: duty = {y + z, z, x + y + z, true}; break;
    case 6: duty = {x + y + z, z, x + z, true}; break;
    default: return {};
    }
    duty.a = std::clamp(duty.a, 0.0f, 1.0f);
    duty.b = std::clamp(duty.b, 0.0f, 1.0f);
    duty.c = std::clamp(duty.c, 0.0f, 1.0f);
    return duty;
}
} // namespace motor
} // namespace vehicle
