#pragma once

#include "error_info.hpp"
#include "esp_simplefoc.h"

namespace vehicle {
namespace encoder {

/*
 * CheckedEncoder通过指定I2C控制器读取一只AS5600。
 * refresh()更新缓存角度和健康状态，SimpleFOC对齐与电机服务消费同一缓存。
 */
class CheckedEncoder final : public Sensor {
public:
    CheckedEncoder(i2c_port_t port, gpio_num_t scl, gpio_num_t sda);

    // initialize()创建AS5600设备并初始化SimpleFOC传感器状态。
    esp_err_t initialize(ErrorInfo *error, ErrorPoint point);
    // refresh()读取AS5600原始角度并更新angleRad()。
    bool refresh();
    // getSensorAngle()向SimpleFOC返回当前机械角，单位rad。
    float getSensorAngle() override;

    void setCached(bool cached);
    bool healthy() const;
    float angleRad() const;
    esp_err_t rawError() const;

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
