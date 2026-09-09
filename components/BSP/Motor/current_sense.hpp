#pragma once
#include <array>
#include <cstdint>
#include "esp_err.h"
namespace vehicle::motor::current_sense {
struct PhaseCurrents { float a; float b; float c; }; // A
struct Sample {
    std::array<PhaseCurrents, 2> phases_a;
    std::int64_t started_us;
    bool valid;
};
// ControlTask独占；仅公共使能关闭且无相电流时允许零偏校准。
esp_err_t initialize();
Sample read();
void release();
} // namespace vehicle::motor::current_sense
