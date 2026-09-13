#pragma once

#include "error_info.hpp"
#include "esp_simplefoc.h"

namespace vehicle {
namespace encoder {

/*
 * CheckedEncoder通过指定I2C控制器读取一只AS5600。
 * Refresh()更新缓存角度和健康状态，SimpleFOC对齐与电机服务消费同一缓存。
 */
// CheckedEncoder向上层提供带健康状态的AS5600角度缓存。
class CheckedEncoder final : public Sensor {
public:
    CheckedEncoder(i2c_port_t port, gpio_num_t scl, gpio_num_t sda);

    // Initialize()创建AS5600设备并初始化SimpleFOC传感器状态。
    esp_err_t Initialize(ErrorInfo *error, ErrorPoint point);
    // Refresh()读取AS5600原始角度并更新angle_rad()。
    bool Refresh();
    // getSensorAngle()向SimpleFOC返回当前机械角，单位rad。
    float getSensorAngle() override;

    // set_cached()控制getSensorAngle()是否复用最近一次Refresh()结果。
    void set_cached(bool cached);
    // healthy()返回最近一次I2C操作后的传感器可用状态。
    bool healthy() const;
    // angle_rad()返回最近一次成功读取的机械角，单位rad。
    float angle_rad() const;
    // raw_error()返回最近一次I2C读取的esp_err_t。
    esp_err_t raw_error() const;

private:
    // port_保存AS5600挂载的I2C控制器编号。
    i2c_port_t port_;
    // scl_保存总线时钟线GPIO。
    gpio_num_t scl_;
    // sda_保存总线数据线GPIO。
    gpio_num_t sda_;
    // bus_保存i2c_bus_create()返回的总线句柄。
    i2c_bus_handle_t bus_{};
    // device_保存AS5600设备句柄。
    i2c_bus_device_handle_t device_{};
    // cached_为true时getSensorAngle()不再触发I2C读取。
    bool cached_{false};
    // healthy_在任一I2C操作失败后变为false。
    bool healthy_{true};
    // angle_rad_保存最近一次读取并换算后的机械角，单位rad。
    float angle_rad_{};
    // raw_error_保存最近一次i2c_bus读取的原始错误码。
    esp_err_t raw_error_{ESP_OK};
};

} // namespace encoder
} // namespace vehicle
