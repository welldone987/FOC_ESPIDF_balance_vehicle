#pragma once

#include <cstdint>

#include "esp_err.h"

namespace vehicle {

// ErrorDomain标识错误码来自应用、ESP-IDF、NimBLE或SimpleFOC。
enum class ErrorDomain : std::uint8_t { application, esp, nimble, simplefoc };

/*
 * ErrorPoint为启动、采样、控制和通信故障保留稳定的诊断编号。
 * 枚举顺序承载既有数值：driver/encoder/alignment/pi_svpwm的M1条目位置
 * 对应原right_*编号，重命名时不能重排。
 */
enum class ErrorPoint : std::uint16_t {
    none=0, boot_resource=1, nvs=2, core_dump=3,
    power_map=0x100, power_unit, power_channel, power_calibration, power_raw, power_mv, undervoltage,
    current_unit=0x200, current_map, current_channel, current_calibration, current_raw,
    current_mv, current_range, current_timeout, offset_mean, offset_noise, phase_limit, reconstructed_limit, current_state,
    motor_gate=0x300, enable_gpio, disable_gpio, driver_M0, driver_M1,
    encoder_init_M0, encoder_init_M1, encoder_read_M0, encoder_read_M1,
    alignment_M1, alignment_M0, wheel_dt, wheel_invalid, wheel_age, current_dt,
    command_invalid, pi_svpwm_M0, pi_svpwm_M1, output_age, motor_state, current_output_age,
    imu_bus=0x400, imu_device, imu_read, imu_write, imu_id, imu_pmu_timeout, imu_foc_timeout, imu_state, imu_filter,
    imu_soft_reset, imu_accel_normal, imu_gyro_normal, imu_accel_range, imu_accel_conf,
    imu_gyro_range, imu_gyro_conf, imu_foc_config, imu_foc_start, imu_foc_offset,
    control_gap=0x500, attitude_gap, fall, control_output, control_timer,
    ble_init=0x600, ble_name, ble_count, ble_services, ble_address=0x605,
    ble_adv_fields, ble_scan_fields, ble_advertise, ble_ready_timeout, ble_reset, ble_notify,
    // 追加编号，不改变已有错误点。
    // 0x620段用于上位机本地错误，raw保留DOMException.code。
    ble_connect=0x60c, ble_disconnect, ble_command, ble_command_timeout, ble_conn_update,
    ble_web_write_timeout=0x620, ble_web_gatt, ble_web_telemetry,
    wifi_init=0x700
};

// ErrorInfo保存错误来源、现场数值和源码位置，供启动与运行诊断复制。
struct ErrorInfo {
    // code保存原始esp_err_t，point_id标识发生位置。
    esp_err_t code{};
    ErrorPoint point_id{};
    // domain和raw_code区分错误来源域与原始错误码。
    ErrorDomain domain{};
    std::int32_t raw_code{};
    // file、function和line保存报错源码位置。
    const char *file{};
    const char *function{};
    std::uint32_t line{};
    // value和threshold保存现场值与阈值，单位由调用点决定。
    float value{}, threshold{};
    // channel保存通道编号，-1表示不适用。
    std::int16_t channel{-1};
    // valid_fields的bit0、bit1和bit2依次表示value、threshold和channel有效。
    std::uint8_t valid_fields{};
    // comparison使用-1表示小于等于、1表示大于、0表示其他关系。
    std::int8_t comparison{};
};

// ErrorAt()填写一帧ErrorInfo并透传原始esp_err_t。
inline esp_err_t ErrorAt(ErrorInfo *out, esp_err_t code, ErrorPoint point,
    ErrorDomain domain, std::int32_t raw, const char *file, const char *function,
    std::uint32_t line, float value=0, float threshold=0, int channel=-1,
    std::uint8_t valid=0, std::int8_t comparison=0)
{
    if (out) {
        *out = {code, point, domain, raw, file, function, line, value, threshold,
            static_cast<std::int16_t>(channel), valid, comparison};
    }
    return code;
}

} // namespace vehicle

#define VEHICLE_ERROR(out, code, point, domain, raw, ...) \
    ::vehicle::ErrorAt(out, code, ::vehicle::ErrorPoint::point, ::vehicle::ErrorDomain::domain, raw, \
        __FILE__, __func__, __LINE__ __VA_OPT__(,) __VA_ARGS__)
