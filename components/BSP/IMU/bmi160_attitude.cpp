#include "bmi160_attitude.hpp"

#include <cmath>
#include <cstdint>

#include "board_pins.hpp"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_bus.h"
#include "imu_config.hpp"

namespace vehicle {
namespace imu {
namespace {

/*
 * BMI160初始化路径配置PMU、加速度计、陀螺仪和FOC偏置。
 * ReadAttitude()把I2C原始帧转换为物理量，再用时间戳维护互补滤波状态。
 */

// Pi把atan2结果的弧度换算为度。
constexpr float Pi = 3.14159265358979323846f;

// 以下常量是BMI160数据手册的寄存器地址。
constexpr std::uint8_t RegChipId = 0x00U;
constexpr std::uint8_t RegPmuStatus = 0x03U;
constexpr std::uint8_t RegGyroData = 0x0CU;
constexpr std::uint8_t RegStatus = 0x1BU;
constexpr std::uint8_t RegAccelConf = 0x40U;
constexpr std::uint8_t RegAccelRange = 0x41U;
constexpr std::uint8_t RegGyroConf = 0x42U;
constexpr std::uint8_t RegGyroRange = 0x43U;
constexpr std::uint8_t RegFocConf = 0x69U;
constexpr std::uint8_t RegOffset6 = 0x77U;
constexpr std::uint8_t RegCmd = 0x7EU;

// 以下常量是BMI160的命令值和配置位掩码。
constexpr std::uint8_t ChipId = 0xD1U;
constexpr std::uint8_t CmdSoftReset = 0xB6U;
constexpr std::uint8_t CmdAccelNormal = 0x11U;
constexpr std::uint8_t CmdGyroNormal = 0x15U;
constexpr std::uint8_t CmdStartFoc = 0x03U;
constexpr std::uint8_t PmuNormalMask = 0x3CU;
constexpr std::uint8_t PmuBothNormal = 0x14U;
constexpr std::uint8_t FocGyroEnable = 0x40U;
constexpr std::uint8_t FocReady = 0x08U;
constexpr std::uint8_t GyroOffsetEnable = 0x80U;
constexpr std::uint8_t AccelRange2G = 0x03U;
constexpr std::uint8_t GyroRange1000Dps = 0x01U;
constexpr std::uint8_t OdrMask = 0x0FU;
constexpr std::uint8_t Odr800Hz = 0x0BU;

// bus保存BMI160与M0编码器共享的I2C0总线句柄。
i2c_bus_handle_t bus = nullptr;
// device保存BMI160设备句柄。
i2c_bus_device_handle_t device = nullptr;
// initialized为true后ReadAttitude()才允许访问传感器。
bool initialized = false;
// last_pitch_deg保存最近一次互补滤波输出的俯仰角，单位deg。
float last_pitch_deg = 0.0f;
// previous_sample_us保存上一帧采样时刻。
// previous_sample_us为0表示尚未建立基准。
std::int64_t previous_sample_us = 0;

// ReadRegister()从BMI160指定寄存器读取一个字节。
esp_err_t ReadRegister(std::uint8_t address, std::uint8_t *value, ErrorInfo *error)
{
    if (device == nullptr || value == nullptr) {
        return VEHICLE_ERROR(error, ESP_ERR_INVALID_STATE, imu_state, application, 0);
    }
    const esp_err_t rc = i2c_bus_read_byte(device, address, value);
    return rc == ESP_OK ? ESP_OK : VEHICLE_ERROR(error, rc, imu_read, esp, rc, address, 0, -1, 1);
}

// ReadRegisters()从连续寄存器地址读取一段原始数据。
esp_err_t ReadRegisters(std::uint8_t address, std::uint8_t *data, std::size_t length, ErrorInfo *error)
{
    if (device == nullptr || data == nullptr || length == 0U) {
        return VEHICLE_ERROR(error, ESP_ERR_INVALID_ARG, imu_state, application, 0);
    }
    const esp_err_t rc = i2c_bus_read_bytes(device, address, length, data);
    return rc == ESP_OK ? ESP_OK : VEHICLE_ERROR(error, rc, imu_read, esp, rc, address, 0, -1, 1);
}

// WriteRegister()向BMI160指定寄存器写入一个字节。
esp_err_t WriteRegister(std::uint8_t address, std::uint8_t value, ErrorInfo *error)
{
    if (device == nullptr) {
        return VEHICLE_ERROR(error, ESP_ERR_INVALID_STATE, imu_state, application, 0);
    }
    const esp_err_t rc = i2c_bus_write_byte(device, address, value);
    if (rc == ESP_OK) { return ESP_OK; }
    ErrorPoint point=ErrorPoint::imu_write;
    switch (address) {
    case RegCmd:
        point=value == CmdSoftReset ? ErrorPoint::imu_soft_reset :
              value == CmdAccelNormal ? ErrorPoint::imu_accel_normal :
              value == CmdGyroNormal ? ErrorPoint::imu_gyro_normal : ErrorPoint::imu_foc_start;
        break;
    case RegAccelRange: point=ErrorPoint::imu_accel_range; break;
    case RegAccelConf: point=ErrorPoint::imu_accel_conf; break;
    case RegGyroRange: point=ErrorPoint::imu_gyro_range; break;
    case RegGyroConf: point=ErrorPoint::imu_gyro_conf; break;
    case RegFocConf: point=ErrorPoint::imu_foc_config; break;
    case RegOffset6: point=ErrorPoint::imu_foc_offset; break;
    default: break;
    }
    return ErrorAt(error,rc,point,ErrorDomain::esp,rc,__FILE__,__func__,__LINE__,value,0,address,5);
}

// SignedWord()按BMI160低字节在前的格式把两个字节还原为有符号计数。
std::int16_t SignedWord(const std::uint8_t *bytes)
{
    const std::uint16_t value =
        static_cast<std::uint16_t>(bytes[0]) |
        (static_cast<std::uint16_t>(bytes[1]) << 8U);
    return static_cast<std::int16_t>(value);
}

// WaitForPmuNormal(ErrorInfo *error)等待加速度计和陀螺仪都进入正常工作状态。
esp_err_t WaitForPmuNormal(ErrorInfo *error)
{
    const std::int64_t deadline = esp_timer_get_time() + 250000LL;
    while (esp_timer_get_time() < deadline) {
        std::uint8_t status = 0U;
        const esp_err_t result = ReadRegister(RegPmuStatus, &status, error);
        if (result != ESP_OK) {
            return result;
        }
        if ((status & PmuNormalMask) == PmuBothNormal) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(1U));
    }
    return VEHICLE_ERROR(error, ESP_ERR_TIMEOUT, imu_pmu_timeout, application, 0);
}

// CalibrateGyroOffset(ErrorInfo *error)启动BMI160陀螺仪FOC并保存硬件偏置使能位。
esp_err_t CalibrateGyroOffset(ErrorInfo *error)
{
    std::uint8_t value = 0U;
    esp_err_t result = ReadRegister(RegFocConf, &value, error);
    if (result != ESP_OK) {
        return result;
    }

    result = WriteRegister(RegFocConf, static_cast<std::uint8_t>(value | FocGyroEnable), error);
    if (result != ESP_OK) {
        return result;
    }

    result = WriteRegister(RegCmd, CmdStartFoc, error);
    if (result != ESP_OK) {
        return result;
    }

    const std::int64_t deadline = esp_timer_get_time() +
        static_cast<std::int64_t>(FocTimeout_ms) * 1000LL;
    while (esp_timer_get_time() < deadline) {
        result = ReadRegister(RegStatus, &value, error);
        if (result != ESP_OK) {
            return result;
        }
        if ((value & FocReady) != 0U) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1U));
    }

    if ((value & FocReady) == 0U) {
        return VEHICLE_ERROR(error, ESP_ERR_TIMEOUT, imu_foc_timeout, application, 0);
    }

    result = ReadRegister(RegOffset6, &value, error);
    if (result != ESP_OK) {
        return result;
    }
    return WriteRegister(
        RegOffset6,
        static_cast<std::uint8_t>(value | GyroOffsetEnable), error);
}

} // namespace

esp_err_t Initialize(ErrorInfo *error)
{
    if (initialized) {
        return ESP_OK;
    }

    i2c_config_t i2c_config{};
    i2c_config.mode = I2C_MODE_MASTER;
    i2c_config.sda_io_num = pins::i2c_sda_M0;
    i2c_config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_config.scl_io_num = pins::i2c_scl_M0;
    i2c_config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_config.master.clk_speed = I2cFrequency_Hz;
    i2c_config.clk_flags = 0U;

    bus = i2c_bus_create(I2C_NUM_0, &i2c_config);
    if (bus == nullptr) {
        return VEHICLE_ERROR(error, ESP_FAIL, imu_bus, application, 0);
    }

    device = i2c_bus_device_create(bus, Address, 0U);
    if (device == nullptr) {
        return VEHICLE_ERROR(error, ESP_FAIL, imu_device, application, 0);
    }

    std::uint8_t value = 0U;
    esp_err_t result = ReadRegister(RegChipId, &value, error);
    if (result != ESP_OK || value != ChipId) {
        return result != ESP_OK ? result : VEHICLE_ERROR(error, ESP_ERR_NOT_FOUND, imu_id, application, 0, value, ChipId, -1, 3);
    }

    result = WriteRegister(RegCmd, CmdSoftReset, error);
    if (result != ESP_OK) {
        return result;
    }
    vTaskDelay(pdMS_TO_TICKS(5U));

    result = WriteRegister(RegCmd, CmdAccelNormal, error);
    if (result != ESP_OK) {
        return result;
    }
    vTaskDelay(pdMS_TO_TICKS(10U));

    result = WriteRegister(RegCmd, CmdGyroNormal, error);
    if (result != ESP_OK) {
        return result;
    }
    vTaskDelay(pdMS_TO_TICKS(100U));

    result = WaitForPmuNormal(error);
    if (result != ESP_OK) {
        return result;
    }

    // 量程为±2g和±1000dps，芯片ODR为800Hz。
    // ControlTask按200Hz读取最新帧。
    // 只替换ODR低四位，保留配置寄存器中的滤波带宽位。
    result = WriteRegister(RegAccelRange, AccelRange2G, error);
    if (result != ESP_OK) {
        return result;
    }
    result = ReadRegister(RegAccelConf, &value, error);
    if (result != ESP_OK) {
        return result;
    }
    value = static_cast<std::uint8_t>((value & ~OdrMask) | Odr800Hz);
    result = WriteRegister(RegAccelConf, value, error);
    if (result != ESP_OK) {
        return result;
    }
    result = WriteRegister(RegGyroRange, GyroRange1000Dps, error);
    if (result != ESP_OK) {
        return result;
    }
    result = ReadRegister(RegGyroConf, &value, error);
    if (result != ESP_OK) {
        return result;
    }
    value = static_cast<std::uint8_t>((value & ~OdrMask) | Odr800Hz);
    result = WriteRegister(RegGyroConf, value, error);
    if (result != ESP_OK) {
        return result;
    }

    result = CalibrateGyroOffset(error);
    if (result != ESP_OK) {
        return result;
    }

    vTaskDelay(pdMS_TO_TICKS(20U));
    initialized = true;
    last_pitch_deg = 0.0f;
    previous_sample_us = 0;
    return ESP_OK;
}

void ResetEstimator()
{
    // 重新建立积分起点，避免初始化前的时间间隔进入下一次姿态计算。
    last_pitch_deg = 0.0f;
    previous_sample_us = 0;
}

esp_err_t ReadAttitude(AttitudeSample *out, ErrorInfo *error)
{
    if (!out || !initialized) { return VEHICLE_ERROR(error, ESP_ERR_INVALID_STATE, imu_state, application, 0); }
    *out = {};

    // raw按陀螺仪Y轴、加速度计X/Y/Z轴的连续寄存器布局保存一帧数据。
    std::uint8_t raw[12]{};
    const esp_err_t result = ReadRegisters(RegGyroData, raw, sizeof(raw), error);
    if (result != ESP_OK) {
        return result;
    }

    // 传感器原始计数按配置量程换算为g和deg/s。
    const float acceleration_x_g =
        static_cast<float>(SignedWord(raw + 6U)) / AccelerationScale;
    const float acceleration_y_g =
        static_cast<float>(SignedWord(raw + 8U)) / AccelerationScale;
    const float acceleration_z_g =
        static_cast<float>(SignedWord(raw + 10U)) / AccelerationScale;
    const float gyro_y_deg_s =
        static_cast<float>(SignedWord(raw + 2U)) / GyroScale;

    // 加速度计俯仰角使用X轴与重力方向的反正切，并保持车辆坐标符号。
    const float accelerometer_pitch_deg =
        std::atan2(
            acceleration_x_g,
            acceleration_z_g + std::fabs(acceleration_y_g)) *
        (-180.0f / Pi);

    // interval_s是本次采样与上次采样之间的实测间隔，单位s。
    const std::int64_t now_us = esp_timer_get_time();
    float interval_s = 0.0f;
    if (previous_sample_us != 0 && now_us >= previous_sample_us) {
        interval_s = static_cast<float>(now_us - previous_sample_us) * 1.0e-6f;
    }

    // 陀螺仪积分提供短期响应，加速度计角度修正长期漂移。
    // 独立平衡在初始化完成后立即启动，首帧用测得姿态建基准，不能从0缓慢爬升后才发现倾倒。
    const float gyro_weight = ComplementaryTimeConstant_s /
        (ComplementaryTimeConstant_s + interval_s);
    const float next_pitch_deg = previous_sample_us == 0 ? accelerometer_pitch_deg :
        gyro_weight *
            (last_pitch_deg + gyro_y_deg_s * interval_s) +
        (1.0f - gyro_weight) * accelerometer_pitch_deg;
    if (!std::isfinite(next_pitch_deg) || !std::isfinite(gyro_y_deg_s)) { return VEHICLE_ERROR(error, ESP_ERR_INVALID_RESPONSE, imu_filter, application, 0, next_pitch_deg, 0, -1, 1); }
    last_pitch_deg = next_pitch_deg;
    previous_sample_us = now_us;
    *out = {last_pitch_deg, gyro_y_deg_s, true};
    return ESP_OK;
}

} // namespace imu
} // namespace vehicle
