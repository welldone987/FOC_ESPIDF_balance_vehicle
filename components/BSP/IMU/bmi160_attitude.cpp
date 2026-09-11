#include "bmi160_attitude.hpp"

#include <cmath>
#include <cstdint>

#include "board_pins.hpp"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_bus.h"
#include "vehicle_config.hpp"

namespace vehicle {
namespace imu {
namespace {

/*
 * BMI160初始化路径配置PMU、加速度计、陀螺仪和FOC偏置。
 * readAttitude()把I2C原始帧转换为物理量，再用时间戳维护互补滤波状态。
 */

constexpr float kPi = 3.14159265358979323846f;

constexpr std::uint8_t kRegChipId = 0x00U;
constexpr std::uint8_t kRegPmuStatus = 0x03U;
constexpr std::uint8_t kRegGyroData = 0x0CU;
constexpr std::uint8_t kRegStatus = 0x1BU;
constexpr std::uint8_t kRegAccelConf = 0x40U;
constexpr std::uint8_t kRegAccelRange = 0x41U;
constexpr std::uint8_t kRegGyroConf = 0x42U;
constexpr std::uint8_t kRegGyroRange = 0x43U;
constexpr std::uint8_t kRegFocConf = 0x69U;
constexpr std::uint8_t kRegOffset6 = 0x77U;
constexpr std::uint8_t kRegCmd = 0x7EU;

constexpr std::uint8_t kChipId = 0xD1U;
constexpr std::uint8_t kCmdSoftReset = 0xB6U;
constexpr std::uint8_t kCmdAccelNormal = 0x11U;
constexpr std::uint8_t kCmdGyroNormal = 0x15U;
constexpr std::uint8_t kCmdStartFoc = 0x03U;
constexpr std::uint8_t kPmuNormalMask = 0x3CU;
constexpr std::uint8_t kPmuBothNormal = 0x14U;
constexpr std::uint8_t kFocGyroEnable = 0x40U;
constexpr std::uint8_t kFocReady = 0x08U;
constexpr std::uint8_t kGyroOffsetEnable = 0x80U;
constexpr std::uint8_t kAccelRange2G = 0x03U;
constexpr std::uint8_t kGyroRange1000Dps = 0x01U;
constexpr std::uint8_t kOdrMask = 0x0FU;
constexpr std::uint8_t kOdr1600Hz = 0x0CU;

constexpr float kComplementaryGyroWeight = 0.98f;
constexpr float kComplementaryAccelWeight = 0.02f;

// bus和device保持BMI160共享I2C0总线的模块级句柄。
i2c_bus_handle_t bus = nullptr;
i2c_bus_device_handle_t device = nullptr;
// initialized控制公开读数接口是否允许访问传感器。
bool initialized = false;
// last_pitch_deg和previous_sample_us保存互补滤波器的跨周期状态。
float last_pitch_deg = 0.0f;
std::int64_t previous_sample_us = 0;

// readRegister()从BMI160指定寄存器读取一个字节。
esp_err_t readRegister(std::uint8_t address, std::uint8_t *value, ErrorInfo *error)
{
    if (device == nullptr || value == nullptr) {
        return VEHICLE_ERROR(error, ESP_ERR_INVALID_STATE, imu_state, application, 0);
    }
    const esp_err_t rc = i2c_bus_read_byte(device, address, value);
    return rc == ESP_OK ? ESP_OK : VEHICLE_ERROR(error, rc, imu_read, esp, rc, address, 0, -1, 1);
}

// readRegisters()从连续寄存器地址读取一段原始数据。
esp_err_t readRegisters(std::uint8_t address, std::uint8_t *data, std::size_t length, ErrorInfo *error)
{
    if (device == nullptr || data == nullptr || length == 0U) {
        return VEHICLE_ERROR(error, ESP_ERR_INVALID_ARG, imu_state, application, 0);
    }
    const esp_err_t rc = i2c_bus_read_bytes(device, address, length, data);
    return rc == ESP_OK ? ESP_OK : VEHICLE_ERROR(error, rc, imu_read, esp, rc, address, 0, -1, 1);
}

// writeRegister()向BMI160指定寄存器写入一个字节。
esp_err_t writeRegister(std::uint8_t address, std::uint8_t value, ErrorInfo *error)
{
    if (device == nullptr) {
        return VEHICLE_ERROR(error, ESP_ERR_INVALID_STATE, imu_state, application, 0);
    }
    const esp_err_t rc = i2c_bus_write_byte(device, address, value);
    if (rc == ESP_OK) { return ESP_OK; }
    ErrorPoint point=ErrorPoint::imu_write;
    switch (address) {
    case kRegCmd:
        point=value == kCmdSoftReset ? ErrorPoint::imu_soft_reset :
              value == kCmdAccelNormal ? ErrorPoint::imu_accel_normal :
              value == kCmdGyroNormal ? ErrorPoint::imu_gyro_normal : ErrorPoint::imu_foc_start;
        break;
    case kRegAccelRange: point=ErrorPoint::imu_accel_range; break;
    case kRegAccelConf: point=ErrorPoint::imu_accel_conf; break;
    case kRegGyroRange: point=ErrorPoint::imu_gyro_range; break;
    case kRegGyroConf: point=ErrorPoint::imu_gyro_conf; break;
    case kRegFocConf: point=ErrorPoint::imu_foc_config; break;
    case kRegOffset6: point=ErrorPoint::imu_foc_offset; break;
    default: break;
    }
    return errorAt(error,rc,point,ErrorDomain::esp,rc,__FILE__,__func__,__LINE__,value,0,address,5);
}

// signedWord()按BMI160低字节在前的格式把两个字节还原为有符号计数。
std::int16_t signedWord(const std::uint8_t *bytes)
{
    const std::uint16_t value =
        static_cast<std::uint16_t>(bytes[0]) |
        (static_cast<std::uint16_t>(bytes[1]) << 8U);
    return static_cast<std::int16_t>(value);
}

// waitForPmuNormal(ErrorInfo *error)等待加速度计和陀螺仪都进入正常工作状态。
esp_err_t waitForPmuNormal(ErrorInfo *error)
{
    const std::int64_t deadline = esp_timer_get_time() + 250000LL;
    while (esp_timer_get_time() < deadline) {
        std::uint8_t status = 0U;
        const esp_err_t result = readRegister(kRegPmuStatus, &status, error);
        if (result != ESP_OK) {
            return result;
        }
        if ((status & kPmuNormalMask) == kPmuBothNormal) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(1U));
    }
    return VEHICLE_ERROR(error, ESP_ERR_TIMEOUT, imu_pmu_timeout, application, 0);
}

// calibrateGyroOffset(ErrorInfo *error)启动BMI160陀螺仪FOC并保存硬件偏置使能位。
esp_err_t calibrateGyroOffset(ErrorInfo *error)
{
    std::uint8_t value = 0U;
    esp_err_t result = readRegister(kRegFocConf, &value, error);
    if (result != ESP_OK) {
        return result;
    }

    result = writeRegister(kRegFocConf, static_cast<std::uint8_t>(value | kFocGyroEnable), error);
    if (result != ESP_OK) {
        return result;
    }

    result = writeRegister(kRegCmd, kCmdStartFoc, error);
    if (result != ESP_OK) {
        return result;
    }

    const std::int64_t deadline = esp_timer_get_time() +
        static_cast<std::int64_t>(config::kBmi160FocTimeoutMs) * 1000LL;
    while (esp_timer_get_time() < deadline) {
        result = readRegister(kRegStatus, &value, error);
        if (result != ESP_OK) {
            return result;
        }
        if ((value & kFocReady) != 0U) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1U));
    }

    if ((value & kFocReady) == 0U) {
        return VEHICLE_ERROR(error, ESP_ERR_TIMEOUT, imu_foc_timeout, application, 0);
    }

    result = readRegister(kRegOffset6, &value, error);
    if (result != ESP_OK) {
        return result;
    }
    return writeRegister(
        kRegOffset6,
        static_cast<std::uint8_t>(value | kGyroOffsetEnable), error);
}

} // namespace

esp_err_t initialize(ErrorInfo *error)
{
    if (initialized) {
        return ESP_OK;
    }

    i2c_config_t i2c_config{};
    i2c_config.mode = I2C_MODE_MASTER;
    i2c_config.sda_io_num = board::pins::kI2c0Sda;
    i2c_config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_config.scl_io_num = board::pins::kI2c0Scl;
    i2c_config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_config.master.clk_speed = config::kI2cFrequencyHz;
    i2c_config.clk_flags = 0U;

    bus = i2c_bus_create(I2C_NUM_0, &i2c_config);
    if (bus == nullptr) {
        return VEHICLE_ERROR(error, ESP_FAIL, imu_bus, application, 0);
    }

    device = i2c_bus_device_create(bus, config::kBmi160Address, 0U);
    if (device == nullptr) {
        return VEHICLE_ERROR(error, ESP_FAIL, imu_device, application, 0);
    }

    std::uint8_t value = 0U;
    esp_err_t result = readRegister(kRegChipId, &value, error);
    if (result != ESP_OK || value != kChipId) {
        return result != ESP_OK ? result : VEHICLE_ERROR(error, ESP_ERR_NOT_FOUND, imu_id, application, 0, value, kChipId, -1, 3);
    }

    result = writeRegister(kRegCmd, kCmdSoftReset, error);
    if (result != ESP_OK) {
        return result;
    }
    vTaskDelay(pdMS_TO_TICKS(5U));

    result = writeRegister(kRegCmd, kCmdAccelNormal, error);
    if (result != ESP_OK) {
        return result;
    }
    vTaskDelay(pdMS_TO_TICKS(10U));

    result = writeRegister(kRegCmd, kCmdGyroNormal, error);
    if (result != ESP_OK) {
        return result;
    }
    vTaskDelay(pdMS_TO_TICKS(100U));

    result = waitForPmuNormal(error);
    if (result != ESP_OK) {
        return result;
    }

    // 量程保持参考实现的±2g和±1000dps，数据更新率保持1600Hz。
    // 只替换ODR低四位，保留配置寄存器中的滤波带宽位。
    result = writeRegister(kRegAccelRange, kAccelRange2G, error);
    if (result != ESP_OK) {
        return result;
    }
    result = readRegister(kRegAccelConf, &value, error);
    if (result != ESP_OK) {
        return result;
    }
    value = static_cast<std::uint8_t>((value & ~kOdrMask) | kOdr1600Hz);
    result = writeRegister(kRegAccelConf, value, error);
    if (result != ESP_OK) {
        return result;
    }
    result = writeRegister(kRegGyroRange, kGyroRange1000Dps, error);
    if (result != ESP_OK) {
        return result;
    }
    result = readRegister(kRegGyroConf, &value, error);
    if (result != ESP_OK) {
        return result;
    }
    value = static_cast<std::uint8_t>((value & ~kOdrMask) | kOdr1600Hz);
    result = writeRegister(kRegGyroConf, value, error);
    if (result != ESP_OK) {
        return result;
    }

    result = calibrateGyroOffset(error);
    if (result != ESP_OK) {
        return result;
    }

    vTaskDelay(pdMS_TO_TICKS(20U));
    initialized = true;
    last_pitch_deg = 0.0f;
    previous_sample_us = 0;
    return ESP_OK;
}

void resetEstimator()
{
    // 重新建立积分起点，避免初始化前的时间间隔进入下一次姿态计算。
    last_pitch_deg = 0.0f;
    previous_sample_us = esp_timer_get_time();
}

esp_err_t readAttitude(AttitudeSample *out, ErrorInfo *error)
{
    if (!out || !initialized) { return VEHICLE_ERROR(error, ESP_ERR_INVALID_STATE, imu_state, application, 0); }
    *out = {};

    // raw按陀螺仪Y轴、加速度计X/Y/Z轴的连续寄存器布局保存一帧数据。
    std::uint8_t raw[12]{};
    const esp_err_t result = readRegisters(kRegGyroData, raw, sizeof(raw), error);
    if (result != ESP_OK) {
        return result;
    }

    // 传感器原始计数按配置量程换算为g和deg/s。
    const float acceleration_x_g =
        static_cast<float>(signedWord(raw + 6U)) / config::kBmi160AccelerationScale;
    const float acceleration_y_g =
        static_cast<float>(signedWord(raw + 8U)) / config::kBmi160AccelerationScale;
    const float acceleration_z_g =
        static_cast<float>(signedWord(raw + 10U)) / config::kBmi160AccelerationScale;
    const float gyro_y_deg_s =
        static_cast<float>(signedWord(raw + 2U)) / config::kBmi160GyroScale;

    // 加速度计俯仰角使用X轴与重力方向的反正切，并保持车辆坐标符号。
    const float accelerometer_pitch_deg =
        std::atan2(
            acceleration_x_g,
            acceleration_z_g + std::fabs(acceleration_y_g)) *
        (-180.0f / kPi);

    // interval_s是本次采样与上次采样之间的实测间隔，单位s。
    const std::int64_t now_us = esp_timer_get_time();
    float interval_s = 0.0f;
    if (previous_sample_us != 0 && now_us >= previous_sample_us) {
        interval_s = static_cast<float>(now_us - previous_sample_us) * 1.0e-6f;
    }

    // 陀螺仪积分提供短期响应，加速度计角度修正长期漂移。
    const float next_pitch_deg =
        kComplementaryGyroWeight *
            (last_pitch_deg + gyro_y_deg_s * interval_s) +
        kComplementaryAccelWeight * accelerometer_pitch_deg;
    if (!std::isfinite(next_pitch_deg) || !std::isfinite(gyro_y_deg_s)) { return VEHICLE_ERROR(error, ESP_ERR_INVALID_RESPONSE, imu_filter, application, 0, next_pitch_deg, 0, -1, 1); }
    last_pitch_deg = next_pitch_deg;
    previous_sample_us = now_us;
    *out = {last_pitch_deg, gyro_y_deg_s, true};
    return ESP_OK;
}

} // namespace imu
} // namespace vehicle
