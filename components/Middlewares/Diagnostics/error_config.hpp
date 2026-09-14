#pragma once

#include <cstdint>

#include "esp_err.h"

namespace vehicle {

/*
 * 错误诊断契约（跨层共享）：
 * - ErrorPoint 编号、名字与单位是稳定诊断契约，禁止重排或改名；新增点只追加在段尾。
 * - 只有本头文件允许 BSP 以 include 方式使用（纯类型，不建立链接依赖）。
 * - ErrorInfo 是旁路详情，只服务定位；功能传递一律走函数返回的 esp_err_t。
 */

// 错误点单表：X(名字, 编号, 单位)；单位是 value/threshold 的打印后缀，空串表示无单位。
// 0x604 为历史空洞保留；0x620 段为网页本地错误，固件不产生。
#define VEHICLE_ERROR_POINT_TABLE(X) \
    /* 0x000 通用 */ \
    X(none, 0x000, "") X(boot_resource, 0x001, "") X(nvs, 0x002, "") \
    /* 0x100 电源 */ \
    X(power_map, 0x100, "") X(power_unit, 0x101, "") X(power_channel, 0x102, "") \
    X(power_calibration, 0x103, "") X(power_raw, 0x104, "") X(power_mv, 0x105, "") \
    X(undervoltage, 0x106, "V") \
    /* 0x200 电流采样 */ \
    X(current_unit, 0x200, "") X(current_map, 0x201, "") X(current_channel, 0x202, "") \
    X(current_calibration, 0x203, "") X(current_raw, 0x204, "cnt") X(current_mv, 0x205, "cnt") \
    X(current_range, 0x206, "mV") X(current_timeout, 0x207, "us") \
    X(offset_mean, 0x208, "mV") X(offset_noise, 0x209, "mV") \
    X(phase_limit, 0x20a, "A") X(reconstructed_limit, 0x20b, "A") X(current_state, 0x20c, "") \
    X(current_dma_init, 0x20d, "") X(current_dma_start, 0x20e, "") X(current_dma_read, 0x20f, "B") \
    /* 0x300 电机 */ \
    X(motor_gate, 0x300, "") X(enable_gpio, 0x301, "") X(disable_gpio, 0x302, "") \
    X(driver_M0, 0x303, "") X(driver_M1, 0x304, "") \
    X(encoder_init_M0, 0x305, "") X(encoder_init_M1, 0x306, "") \
    X(encoder_read_M0, 0x307, "") X(encoder_read_M1, 0x308, "") \
    X(alignment_M1, 0x309, "") X(alignment_M0, 0x30a, "") \
    X(wheel_dt, 0x30b, "s") X(wheel_invalid, 0x30c, "") X(wheel_age, 0x30d, "us") \
    X(current_dt, 0x30e, "s") X(command_invalid, 0x30f, "") \
    X(pi_svpwm_M0, 0x310, "") X(pi_svpwm_M1, 0x311, "") \
    X(output_age, 0x312, "us") X(motor_state, 0x313, "") X(current_output_age, 0x314, "us") \
    /* 0x400 IMU */ \
    X(imu_bus, 0x400, "") X(imu_device, 0x401, "") X(imu_read, 0x402, "") X(imu_write, 0x403, "") \
    X(imu_id, 0x404, "") X(imu_pmu_timeout, 0x405, "") X(imu_foc_timeout, 0x406, "") \
    X(imu_state, 0x407, "") X(imu_filter, 0x408, "deg") \
    X(imu_soft_reset, 0x409, "") X(imu_accel_normal, 0x40a, "") X(imu_gyro_normal, 0x40b, "") \
    X(imu_accel_range, 0x40c, "") X(imu_accel_conf, 0x40d, "") X(imu_gyro_range, 0x40e, "") \
    X(imu_gyro_conf, 0x40f, "") X(imu_foc_config, 0x410, "") X(imu_foc_start, 0x411, "") \
    X(imu_foc_offset, 0x412, "") \
    /* 0x500 控制 */ \
    X(control_gap, 0x500, "s") X(attitude_gap, 0x501, "s") X(fall, 0x502, "rad") \
    X(control_output, 0x503, "") X(control_timer, 0x504, "") \
    /* 0x600 BLE */ \
    X(ble_init, 0x600, "") X(ble_name, 0x601, "") X(ble_count, 0x602, "") X(ble_services, 0x603, "") \
    X(ble_address, 0x605, "") X(ble_adv_fields, 0x606, "") X(ble_scan_fields, 0x607, "") \
    X(ble_advertise, 0x608, "") X(ble_ready_timeout, 0x609, "") X(ble_reset, 0x60a, "") \
    X(ble_notify, 0x60b, "cnt") \
    X(ble_connect, 0x60c, "") X(ble_disconnect, 0x60d, "ms") X(ble_command, 0x60e, "B") \
    X(ble_command_timeout, 0x60f, "us") X(ble_conn_update, 0x610, "") \
    /* 0x620 网页本地错误 */ \
    X(ble_web_write_timeout, 0x620, "") X(ble_web_gatt, 0x621, "") X(ble_web_telemetry, 0x622, "") \
    /* 0x700 Wi-Fi */ \
    X(wifi_init, 0x700, "")

// ErrorPoint 由单表生成；编号即诊断契约。
enum class ErrorPoint : std::uint16_t {
#define VEHICLE_POINT_ENUM(name, value, unit) name = value,
    VEHICLE_ERROR_POINT_TABLE(VEHICLE_POINT_ENUM)
#undef VEHICLE_POINT_ENUM
};

// PointUnit()返回错误点的打印单位，空串表示无单位。
constexpr const char *PointUnit(ErrorPoint point)
{
    switch (point) {
#define VEHICLE_POINT_UNIT(name, value, unit) case ErrorPoint::name: return unit;
        VEHICLE_ERROR_POINT_TABLE(VEHICLE_POINT_UNIT)
#undef VEHICLE_POINT_UNIT
    }
    return "";
}

// PointValue()把错误点转换为契约编号，供段边界断言使用。
constexpr std::uint16_t PointValue(ErrorPoint point)
{
    return static_cast<std::uint16_t>(point);
}

// valid_fields 的位定义：依次表示 value、threshold、channel 有效。
inline constexpr std::uint8_t ErrorValue = 1U << 0U;
inline constexpr std::uint8_t ErrorThreshold = 1U << 1U;
inline constexpr std::uint8_t ErrorChannel = 1U << 2U;

// ErrorInfo 保存错误来源、现场数值和源码位置，供诊断复制与打印。
struct ErrorInfo {
    // code保存函数返回值与失败类别。
    esp_err_t code{};
    // point_id标识发生位置，是查表主键。
    ErrorPoint point_id{};
    // raw_code保存外部库原始码；esp域时与code相同，application域为0。
    std::int32_t raw_code{};
    // file和line保存报错源码位置。
    const char *file{};
    std::uint32_t line{};
    // value和threshold保存现场值与界限，单位由PointUnit()给出。
    float value{};
    float threshold{};
    // channel保存通道/实例编号，-1表示不适用。
    std::int16_t channel{-1};
    // valid_fields的bit0、bit1和bit2依次表示value、threshold和channel有效。
    std::uint8_t valid_fields{};
};

// ErrorAt()填写一帧ErrorInfo并透传原始esp_err_t。
inline esp_err_t ErrorAt(ErrorInfo *out, esp_err_t code, ErrorPoint point,
    std::int32_t raw, const char *file, std::uint32_t line,
    float value=0, float threshold=0, int channel=-1, std::uint8_t valid=0)
{
    if (out) {
        *out = {code, point, raw, file, line, value, threshold,
            static_cast<std::int16_t>(channel), valid};
    }
    return code;
}

} // namespace vehicle

// 段边界断言：每段首等于基址、段尾小于下一基址；新增点越界时编译失败。
static_assert(::vehicle::PointValue(::vehicle::ErrorPoint::none) == 0x000);
static_assert(::vehicle::PointValue(::vehicle::ErrorPoint::nvs) < 0x100);
static_assert(::vehicle::PointValue(::vehicle::ErrorPoint::power_map) == 0x100);
static_assert(::vehicle::PointValue(::vehicle::ErrorPoint::undervoltage) < 0x200);
static_assert(::vehicle::PointValue(::vehicle::ErrorPoint::current_unit) == 0x200);
static_assert(::vehicle::PointValue(::vehicle::ErrorPoint::current_dma_read) < 0x300);
static_assert(::vehicle::PointValue(::vehicle::ErrorPoint::motor_gate) == 0x300);
static_assert(::vehicle::PointValue(::vehicle::ErrorPoint::current_output_age) < 0x400);
static_assert(::vehicle::PointValue(::vehicle::ErrorPoint::imu_bus) == 0x400);
static_assert(::vehicle::PointValue(::vehicle::ErrorPoint::imu_foc_offset) < 0x500);
static_assert(::vehicle::PointValue(::vehicle::ErrorPoint::control_gap) == 0x500);
static_assert(::vehicle::PointValue(::vehicle::ErrorPoint::control_timer) < 0x600);
static_assert(::vehicle::PointValue(::vehicle::ErrorPoint::ble_init) == 0x600);
static_assert(::vehicle::PointValue(::vehicle::ErrorPoint::ble_conn_update) < 0x620);
static_assert(::vehicle::PointValue(::vehicle::ErrorPoint::ble_web_write_timeout) == 0x620);
static_assert(::vehicle::PointValue(::vehicle::ErrorPoint::ble_web_telemetry) < 0x700);
static_assert(::vehicle::PointValue(::vehicle::ErrorPoint::wifi_init) == 0x700);

#define VEHICLE_ERROR(out, code, point, raw, ...) \
    ::vehicle::ErrorAt(out, code, ::vehicle::ErrorPoint::point, raw, \
        __FILE__, __LINE__ __VA_OPT__(,) __VA_ARGS__)
