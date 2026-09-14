#pragma once
#include <array>
#include <cstdint>
#include "error_config.hpp"
namespace vehicle {
namespace current_sensor {

/*
 * 相电流采样模块通过ADC1四通道读取两台电机的INA240A2输出。
 * Initialize()在公共使能关闭时校准四路静态零偏，并把采样路径切换到ADC连续DMA。
 * Start()/Suspend()随电机初始化与锁存停机关闭DMA；Read()排空DMA池取最新扫描。
 * 输出保持M0/M1的A/B相电流并按ia+ib+ic=0重建第三相。
 */
// PhaseCurrents保存一台电机的三相电流，单位A。
struct PhaseCurrents { float phase_a_A; float phase_b_A; float phase_c_A; };

// Sample保存两台电机的相电流、ADC采样时刻和有效标志。
struct Sample {
    // phase_currents按M0、M1顺序保存两相实测电流，第三相由Read()重建。
    std::array<PhaseCurrents, 2> phase_currents;
    // started_us保存本批ADC采样的保守时刻，单位us。
    std::int64_t started_us;
    // valid标记本批数据是否可用于控制。
    bool valid;
};
// Initialize()仅在公共使能关闭且无相电流时校准静态零点，并建立DMA采样配置；失败时填充error。
esp_err_t Initialize(ErrorInfo *error=nullptr);
// Start()启动DMA采样；必须在电机对齐结束后调用，未启动时Read()返回状态错误。
esp_err_t Start(ErrorInfo *error=nullptr);
// Suspend()停止DMA采样；仅由锁存停机路径调用，重复调用为空操作。
esp_err_t Suspend(ErrorInfo *error=nullptr);
// Read()排空DMA池取最新一次四通道扫描，无完整扫描时返回超时错误。
esp_err_t Read(Sample *out, ErrorInfo *error=nullptr);
// Release()释放DMA、ADC校准、通道和一次性采样单元。
void Release();

} // namespace current_sensor
} // namespace vehicle
