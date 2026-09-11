#pragma once
#include <array>
#include <cstdint>
#include "error_info.hpp"
namespace vehicle::motor::current_sense {
struct PhaseCurrents { float a; float b; float c; }; // A
struct Sample {
    std::array<PhaseCurrents, 2> phases_a;
    std::int64_t started_us;
    bool valid;
};
// ControlTask独占；仅公共使能关闭且无相电流时允许零偏校准。
esp_err_t initialize(ErrorInfo *error=nullptr);
esp_err_t read(Sample *out, ErrorInfo *error=nullptr);
void release();
} // namespace vehicle::motor::current_sense
