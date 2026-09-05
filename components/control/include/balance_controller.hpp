#pragma once

#include <cstdint>

namespace vehicle {
namespace control {

struct ControlInput {
    float left_velocity_rad_s;
    float right_velocity_rad_s;
    float pitch_deg;
    float steering_voltage_v;
    float throttle_velocity_rad_s;
};

struct ControlOutput {
    float target_pitch_deg;
    float balance_voltage_v;
    float steering_voltage_v;
    float left_target_v;
    float right_target_v;
};

// Preserves the minimal main.cpp cascade:
// average wheel velocity -> speed PID -> target pitch -> balance PID -> motor mix.
class BalanceController {
public:
    BalanceController();
    ControlOutput update(const ControlInput &input);

private:
    class Pid {
    public:
        Pid(float p, float i, float d, float ramp, float limit);
        float update(float error);

    private:
        float p_;
        float i_;
        float d_;
        float ramp_;
        float limit_;
        float error_prev_;
        float output_prev_;
        float integral_prev_;
        std::int64_t timestamp_prev_us_;
    };

    class LowPass {
    public:
        explicit LowPass(float time_constant_s);
        float update(float value);

    private:
        float time_constant_s_;
        float previous_output_;
        std::int64_t timestamp_prev_us_;
    };

    Pid balance_pid_;
    Pid speed_pid_;
    LowPass pitch_command_filter_;
    LowPass throttle_filter_;
    LowPass steering_filter_;
};

} // namespace control
} // namespace vehicle
