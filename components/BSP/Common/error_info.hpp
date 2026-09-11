#pragma once
#include <cstdint>
#include "esp_err.h"
namespace vehicle {
enum class ErrorDomain : std::uint8_t { application, esp, nimble, simplefoc };
enum class ErrorPoint : std::uint16_t {
    none=0, boot_resource=1, nvs=2, core_dump=3,
    power_map=0x100, power_unit, power_channel, power_calibration, power_raw, power_mv, undervoltage,
    current_unit=0x200, current_map, current_channel, current_calibration, current_raw,
    current_mv, current_range, current_timeout, offset_mean, offset_noise, phase_limit, reconstructed_limit, current_state,
    motor_gate=0x300, enable_gpio, disable_gpio, left_driver, right_driver,
    left_encoder_init, right_encoder_init, left_encoder_read, right_encoder_read,
    right_alignment, left_alignment, wheel_dt, wheel_invalid, wheel_age, current_dt,
    command_invalid, left_pi_svpwm, right_pi_svpwm, output_age, motor_state, current_output_age,
    imu_bus=0x400, imu_device, imu_read, imu_write, imu_id, imu_pmu_timeout, imu_foc_timeout, imu_state, imu_filter,
    imu_soft_reset, imu_accel_normal, imu_gyro_normal, imu_accel_range, imu_accel_conf,
    imu_gyro_range, imu_gyro_conf, imu_foc_config, imu_foc_start, imu_foc_offset,
    control_gap=0x500, attitude_gap, fall, control_output, control_timer,
    ble_init=0x600, ble_name, ble_count, ble_services, ble_address=0x605,
    ble_adv_fields, ble_scan_fields, ble_advertise, ble_ready_timeout, ble_reset,
    wifi_init=0x700
};
struct ErrorInfo {
    esp_err_t code{};
    ErrorPoint point_id{};
    ErrorDomain domain{};
    std::int32_t raw_code{};
    const char *file{};
    const char *function{};
    std::uint32_t line{};
    float value{}, threshold{};
    std::int16_t channel{-1};
    std::uint8_t valid_fields{}; // bit0=value, bit1=threshold, bit2=channel
    std::int8_t comparison{}; // -1: <=, +1: >, 0: other
};
inline esp_err_t errorAt(ErrorInfo *out, esp_err_t code, ErrorPoint point,
    ErrorDomain domain, std::int32_t raw, const char *file, const char *function,
    std::uint32_t line, float value=0, float threshold=0, int channel=-1,
    std::uint8_t valid=0, std::int8_t comparison=0)
{
    if (out) { *out = {code, point, domain, raw, file, function, line, value, threshold,
        static_cast<std::int16_t>(channel), valid, comparison}; }
    return code;
}
}
#define VEHICLE_ERROR(out, code, point, domain, raw, ...) \
    ::vehicle::errorAt(out, code, ::vehicle::ErrorPoint::point, ::vehicle::ErrorDomain::domain, raw, \
        __FILE__, __func__, __LINE__ __VA_OPT__(,) __VA_ARGS__)
