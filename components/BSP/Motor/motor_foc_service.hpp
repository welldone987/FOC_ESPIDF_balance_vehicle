#pragma once
#include "esp_err.h"
namespace vehicle::motor {
struct WheelState {
    float left_velocity_rad_s;
    float right_velocity_rad_s;
    bool valid;
};
struct CurrentCommand {
    float left_target_a;
    float right_target_a;
};
// 参数确认门通过后才会进行编码器对齐（可能转动）；ADC零偏在驱动关闭时采集。
esp_err_t initialize();
// 采样有效才运行d/q电流PI；失败锁存关闭双轮输出。
WheelState runFocAndReadWheelState();
void stageTarget(const CurrentCommand &command);
void disableOutputs();
} // namespace vehicle::motor
