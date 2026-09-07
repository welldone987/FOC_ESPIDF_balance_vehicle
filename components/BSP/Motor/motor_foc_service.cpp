#include "motor_foc_service.hpp"

#include "board_pins.hpp"
#include "esp_simplefoc.h"

namespace vehicle {
namespace motor {
namespace {

/*
 * 电机模块把AS5600角度、MCPWM三相输出和SimpleFOC电机对象绑定到DengFOC V4硬件。
 * runFocAndReadWheelState()按参考顺序消费上一周期目标并提供本周期轮速。
 */
// kMotorPolePairs和kSupplyVoltageV保存电机极对数与供电电压，电压单位为V。
constexpr int kMotorPolePairs = 7;
constexpr float kSupplyVoltageV = 12.0f;
// kSensorAlignmentVoltageV保存FOC编码器对齐使用的电压，单位V。
constexpr float kSensorAlignmentVoltageV = 2.0f;
// kVelocityPid*保存SimpleFOC轮速估计PID参数。
constexpr float kVelocityPidP = 0.01f;
constexpr float kVelocityPidI = 0.10f;
constexpr float kVelocityPidD = 0.0f;

// kMotor0McpwmGroup和kMotor1McpwmGroup分别绑定左右电机的MCPWM资源组。
constexpr int kMotor0McpwmGroup = 0;
constexpr int kMotor1McpwmGroup = 1;

// AS5600构造参数依次为I2C控制器、SCL和SDA；两个同地址器件使用不同控制器。
AS5600 left_sensor{I2C_NUM_0, board::pins::kI2c0Scl, board::pins::kI2c0Sda};
AS5600 right_sensor{I2C_NUM_1, board::pins::kI2c1Scl, board::pins::kI2c1Sda};

BLDCMotor left_motor{kMotorPolePairs};
BLDCMotor right_motor{kMotorPolePairs};

BLDCDriver3PWM left_driver{
    board::pins::kMotor0PwmA,
    board::pins::kMotor0PwmB,
    board::pins::kMotor0PwmC,
    board::pins::kMotor0Enable,
};

BLDCDriver3PWM right_driver{
    board::pins::kMotor1PwmA,
    board::pins::kMotor1PwmB,
    board::pins::kMotor1PwmC,
    board::pins::kMotor1Enable,
};

// initialized表示编码器、驱动器和FOC对齐已经完成。
bool initialized = false;

// disableDrivers()同时关闭左右三相驱动器的输出使能。
void disableDrivers()
{
    left_driver.disable();
    right_driver.disable();
}

} // namespace

esp_err_t initialize()
{
    if (initialized) {
        return ESP_OK;
    }

    // M0 AS5600使用I2C0，M1 AS5600使用I2C1；BMI160随后与M0共享I2C0。
    left_sensor.init();
    right_sensor.init();

    left_motor.linkSensor(&left_sensor);
    right_motor.linkSensor(&right_sensor);

    // 保留参考实现的轮速估计PID；平衡应用当前仍以电压模式力矩作为执行量。
    left_motor.PID_velocity.P = kVelocityPidP;
    left_motor.PID_velocity.I = kVelocityPidI;
    left_motor.PID_velocity.D = kVelocityPidD;
    right_motor.PID_velocity.P = kVelocityPidP;
    right_motor.PID_velocity.I = kVelocityPidI;
    right_motor.PID_velocity.D = kVelocityPidD;

    left_motor.voltage_sensor_align = kSensorAlignmentVoltageV;
    right_motor.voltage_sensor_align = kSensorAlignmentVoltageV;
    left_driver.voltage_power_supply = kSupplyVoltageV;
    right_driver.voltage_power_supply = kSupplyVoltageV;

    // 显式分配MCPWM组，使双电机硬件资源映射固定。
    if (left_driver.init(kMotor0McpwmGroup) == 0) {
        return ESP_FAIL;
    }
    left_motor.linkDriver(&left_driver);

    if (right_driver.init(kMotor1McpwmGroup) == 0) {
        left_driver.disable();
        return ESP_FAIL;
    }
    right_motor.linkDriver(&right_driver);

    left_motor.torque_controller = TorqueControlType::voltage;
    right_motor.torque_controller = TorqueControlType::voltage;
    left_motor.controller = MotionControlType::torque;
    right_motor.controller = MotionControlType::torque;

    // 保留参考启动顺序：先初始化M1，再初始化M0。
    if (right_motor.init() == 0 || left_motor.init() == 0) {
        disableDrivers();
        return ESP_FAIL;
    }

    // initFOC()执行编码器方向和电角度零点对齐，可能驱动车轮转动。
    if (right_motor.initFOC() == 0 || left_motor.initFOC() == 0) {
        disableDrivers();
        return ESP_FAIL;
    }

    initialized = true;
    return ESP_OK;
}

WheelState runFocAndReadWheelState()
{
    if (!initialized) {
        return WheelState{0.0f, 0.0f, false};
    }

    // 先执行FOC，再由move()消费stageTarget()在上一周期写入的目标。
    left_motor.loopFOC();
    right_motor.loopFOC();
    left_motor.move();
    right_motor.move();

    return WheelState{
        left_motor.shaft_velocity,
        right_motor.shaft_velocity,
        true,
    };
}

void stageTarget(const VoltageCommand &command)
{
    if (!initialized) {
        return;
    }

    left_motor.target = command.left_target_v;
    right_motor.target = command.right_target_v;
}

void disableOutputs()
{
    if (!initialized) {
        return;
    }

    left_motor.target = 0.0f;
    right_motor.target = 0.0f;
    left_motor.disable();
    right_motor.disable();
}

bool isInitialized()
{
    return initialized;
}

} // namespace motor
} // namespace vehicle
