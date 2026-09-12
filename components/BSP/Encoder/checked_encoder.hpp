#pragma once

#include "error_info.hpp"
#include "esp_simplefoc.h"

namespace vehicle {
namespace encoder {

/*
 * CheckedEncoder通过指定I2C控制器读取一只AS5600。
 * Refresh()更新缓存角度和健康状态，SimpleFOC对齐与电机服务消费同一缓存。
 */
class CheckedEncoder final : public Sensor {
public:
    CheckedEncoder(i2c_port_t port, gpio_num_t scl, gpio_num_t sda);

    // Initialize()创建AS5600设备并初始化SimpleFOC传感器状态。
    esp_err_t Initialize(ErrorInfo *error, ErrorPoint point);
    // Refresh()读取AS5600原始角度并更新angle_rad()。
    bool Refresh();
    // getSensorAngle()向SimpleFOC返回当前机械角，单位rad。
    float getSensorAngle() override;

    void set_cached(bool cached);
    bool healthy() const;
    float angle_rad() const;
    esp_err_t raw_error() const;

private:
    i2c_port_t port_;
    gpio_num_t scl_;
    gpio_num_t sda_;
    i2c_bus_handle_t bus_{};
    i2c_bus_device_handle_t device_{};
    bool cached_{false};
    bool healthy_{true};
    float angle_rad_{};
    esp_err_t raw_error_{ESP_OK};
};

} // namespace encoder
} // namespace vehicle
