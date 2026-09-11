#pragma once
#include <array>
#include <cstdint>
#include "error_info.hpp"
namespace vehicle {
namespace current_sensor {

// PhaseCurrents保存一台电机的三相电流，单位A。
struct PhaseCurrents { float a; float b; float c; };

// Sample保存两台电机的相电流、ADC采样时刻和有效标志。
struct Sample {
    std::array<PhaseCurrents, 2> phases_a;
    std::int64_t started_us;
    bool valid;
};
// initialize()仅在公共使能关闭且无相电流时校准静态零点。
esp_err_t initialize(ErrorInfo *error=nullptr);
// read()读取两台电机的A/B相电流并重建C相电流。
esp_err_t read(Sample *out, ErrorInfo *error=nullptr);
// release()释放ADC校准、通道和一次性采样单元。
void release();

} // namespace current_sensor
} // namespace vehicle
