#pragma once

#include <cstdint>

#include "motor_timing.hpp"

namespace vehicle {
namespace control {

enum class ControlStage : std::uint8_t {
    waiting, command, outputs_off, encoder, imu, outer, current, publish, complete
};

// ControlTiming由控制任务写入，并在周期完成或故障时复制到诊断区。
struct ControlTiming {
    std::uint32_t cycle{}, balance_cycle{}, notifications{}, skipped_releases{};
    ControlStage stage{};
    bool balancing{}, driving{}, starting{}, imu_updated{}, first_release{};
    std::int64_t dt_us{}, elapsed_us{}, outputs_off_us{}, encoder_us{}, imu_us{}, outer_us{};
    motor::CurrentTiming current{};
};

// SampleInterval()为首轮返回显式初始周期，后续返回相邻采样时刻差。
constexpr std::int64_t SampleInterval(
    std::int64_t now, std::int64_t previous, std::int64_t initial)
{
    return previous == 0 ? initial : now - previous;
}

} // namespace control
} // namespace vehicle
