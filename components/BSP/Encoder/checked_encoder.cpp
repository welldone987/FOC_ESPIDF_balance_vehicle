#include "checked_encoder.hpp"

#include <cstdint>

#include "encoder_config.hpp"

namespace vehicle {
namespace encoder {

CheckedEncoder::CheckedEncoder(i2c_port_t port, gpio_num_t scl, gpio_num_t sda)
    : port_(port), scl_(scl), sda_(sda)
{
}

esp_err_t CheckedEncoder::Initialize(ErrorInfo *error, ErrorPoint point)
{
    i2c_config_t bus_config{};
    bus_config.mode = I2C_MODE_MASTER;
    bus_config.sda_io_num = sda_;
    bus_config.scl_io_num = scl_;
    bus_config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    bus_config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    bus_config.master.clk_speed = I2cFrequency_Hz;
    bus_ = i2c_bus_create(port_, &bus_config);
    if (bus_) {
        device_ = i2c_bus_device_create(bus_, As5600Address, 0);
    }
    if (!device_) {
        healthy_ = false;
        return ErrorAt(error, ESP_FAIL, point, ErrorDomain::application, 0,
            __FILE__, __func__, __LINE__);
    }
    Sensor::init();
    if (!healthy_) {
        return ErrorAt(error, raw_error_, point, ErrorDomain::esp, raw_error_,
            __FILE__, __func__, __LINE__);
    }
    return ESP_OK;
}

bool CheckedEncoder::Refresh()
{
    if (!healthy_) {
        return false;
    }
    // raw按大端保存AS5600的12位原始角度寄存器内容。
    std::uint8_t raw[2]{};
    raw_error_ = i2c_bus_read_bytes(
        device_, As5600RawAngleRegister, sizeof(raw), raw);
    healthy_ = healthy_ && raw_error_ == ESP_OK;
    if (healthy_) {
        // count屏蔽到低12位，一个圆周对应4096个计数。
        const unsigned count = ((raw[0] << 8) | raw[1]) & 0x0fff;
        angle_rad_ = (count * 360.0f / 4096.0f) * (3.14159265358979f / 180.0f);
    }
    return healthy_;
}

float CheckedEncoder::getSensorAngle()
{
    if (!cached_) {
        Refresh();
    }
    // 返回-1让SimpleFOC识别本次读取失败。
    return healthy_ ? angle_rad_ : -1.0f;
}

void CheckedEncoder::set_cached(bool cached)
{
    cached_ = cached;
}

bool CheckedEncoder::healthy() const
{
    return healthy_;
}

float CheckedEncoder::angle_rad() const
{
    return angle_rad_;
}

esp_err_t CheckedEncoder::raw_error() const
{
    return raw_error_;
}

} // namespace encoder
} // namespace vehicle
