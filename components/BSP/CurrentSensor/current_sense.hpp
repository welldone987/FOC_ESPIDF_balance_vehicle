#pragma once
#include <array>
#include <cstdint>
#include "error_info.hpp"
namespace vehicle {
namespace current_sensor {

/*
 * 相电流采样模块通过ADC1四通道读取两台电机的INA240A2输出。
 * Initialize()在公共使能关闭时校准四路静态零偏。
 * Read()返回M0/M1的A/B相电流并按ia+ib+ic=0重建第三相。
 */
// PhaseCurrents保存一台电机的三相电流，单位A。
struct PhaseCurrents { float a; float b; float c; };

// Sample保存两台电机的相电流、ADC采样时刻和有效标志。
struct Sample {
    // phases_a按M0、M1顺序保存两相实测电流，第三相由Read()重建。
    std::array<PhaseCurrents, 2> phases_a;
    // started_us保存本批ADC采样的开始时刻，单位us。
    std::int64_t started_us;
    // valid标记本批数据是否可用于控制。
    bool valid;
};
// Initialize()仅在公共使能关闭且无相电流时校准静态零点。
esp_err_t Initialize(ErrorInfo *error=nullptr);
// Read()读取两台电机的A/B相电流并重建C相电流。
esp_err_t Read(Sample *out, ErrorInfo *error=nullptr);
// Release()释放ADC校准、通道和一次性采样单元。
void Release();

} // namespace current_sensor
} // namespace vehicle
