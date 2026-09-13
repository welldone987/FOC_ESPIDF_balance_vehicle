#pragma once

#include <cstdint>

#include "motor_timing.hpp"

namespace vehicle {
namespace control {

// ControlStage按执行顺序标记控制周期的当前阶段，供故障计时定格。
enum class ControlStage : std::uint8_t {
    waiting, command, outputs_off, encoder, imu, outer, current, publish, complete
};

// ControlTiming由控制任务写入，并在周期完成或故障时复制到诊断区。
struct ControlTiming {
    // cycle、balance_cycle、notifications和skipped_releases记录周期、平衡周期、通知与本轮合并数。
    std::uint32_t cycle{}, balance_cycle{}, notifications{}, skipped_releases{};
    // stage保存故障定格时已进入的阶段。
    ControlStage stage{};
    // balancing、driving和starting分别标记平衡使能、目标有效和首次平衡。
    // imu_updated记录本轮是否读取IMU。
    // first_release标记首次控制周期。
    bool balancing{}, driving{}, starting{}, imu_updated{}, first_release{};
    // dt_us、elapsed_us和各阶段耗时分别记录周期间隔与执行耗时，单位us。
    std::int64_t dt_us{}, elapsed_us{}, outputs_off_us{}, encoder_us{}, imu_us{}, outer_us{};
    // current保存电流环内部的阶段计时。
    motor::CurrentTiming current{};
};

// SampleInterval()为首轮返回显式初始周期，后续返回相邻采样时刻差。
constexpr std::int64_t SampleInterval(
    std::int64_t now_us, std::int64_t previous_us, std::int64_t initial_us)
{
    return previous_us == 0 ? initial_us : now_us - previous_us;
}

} // namespace control
} // namespace vehicle
