#include "motor_foc_service.h"

#include <Arduino.h>
#include <SimpleFOC.h>
#include <Wire.h>

#include "vehicle_config.h"

namespace vehicle {

namespace {

// 两套硬件对象保持静态生命周期，仅由本文件的函数访问。
// left_sensor和right_sensor分别绑定Wire与Wire1上的AS5600。
MagneticSensorI2C left_sensor{AS5600_I2C};
MagneticSensorI2C right_sensor{AS5600_I2C};
// left_motor和right_motor分别保存左右电机FOC状态与下一周期target。
BLDCMotor left_motor{config::motor_pole_pairs};
BLDCMotor right_motor{config::motor_pole_pairs};
// left_driver和right_driver分别驱动左右电机三相PWM及使能引脚。
BLDCDriver3PWM left_driver{config::motor0_pwm_a_pin,
                          config::motor0_pwm_b_pin,
                          config::motor0_pwm_c_pin,
                          config::motor0_enable_pin};
BLDCDriver3PWM right_driver{config::motor1_pwm_a_pin,
                           config::motor1_pwm_b_pin,
                           config::motor1_pwm_c_pin,
                           config::motor1_enable_pin};

} // namespace

// initializePinsAndEncoders()把配置引脚、Wire/Wire1和左右AS5600连接到模块内硬件对象。
void initializePinsAndEncoders() {
  pinMode(config::motor0_pwm_a_pin, INPUT_PULLUP);
  pinMode(config::motor0_pwm_b_pin, INPUT_PULLUP);
  pinMode(config::motor0_pwm_c_pin, INPUT_PULLUP);
  pinMode(config::motor1_pwm_a_pin, INPUT_PULLUP);
  pinMode(config::motor1_pwm_b_pin, INPUT_PULLUP);
  pinMode(config::motor1_pwm_c_pin, INPUT_PULLUP);

  // Wire连接左AS5600，并与BMI160共用I2C0总线。
  Wire.begin(config::i2c0_sda_pin,
             config::i2c0_scl_pin,
             config::i2c_frequency_hz);
  // Wire1通过I2C1总线连接右AS5600。
  Wire1.begin(config::i2c1_sda_pin,
              config::i2c1_scl_pin,
              config::i2c_frequency_hz);
  left_sensor.init(&Wire);
  right_sensor.init(&Wire1);

  pinMode(config::motor1_enable_pin, OUTPUT);
  digitalWrite(config::motor1_enable_pin, HIGH);
}

// initializeMotors()为两台电机配置传感器、驱动器、控制模式和FOC启动顺序。
void initializeMotors() {
  // linkSensor()把每台BLDCMotor连接到对应的MagneticSensorI2C。
  left_motor.linkSensor(&left_sensor);
  right_motor.linkSensor(&right_sensor);

  // PID_velocity为左右电机使用同一组速度环参数。
  right_motor.PID_velocity.P = config::simplefoc_velocity_pid_p;
  right_motor.PID_velocity.I = config::simplefoc_velocity_pid_i;
  right_motor.PID_velocity.D = config::simplefoc_velocity_pid_d;
  left_motor.PID_velocity.P = config::simplefoc_velocity_pid_p;
  left_motor.PID_velocity.I = config::simplefoc_velocity_pid_i;
  left_motor.PID_velocity.D = config::simplefoc_velocity_pid_d;

  // voltage_sensor_align和voltage_power_supply分别提供对齐电压与母线供电电压。
  left_motor.voltage_sensor_align =
      config::motor_sensor_alignment_voltage_v;
  left_driver.voltage_power_supply = config::motor_supply_voltage_v;
  left_driver.init();
  left_motor.linkDriver(&left_driver);

  right_motor.voltage_sensor_align =
      config::motor_sensor_alignment_voltage_v;
  right_driver.voltage_power_supply = config::motor_supply_voltage_v;
  right_driver.init();
  right_motor.linkDriver(&right_driver);

  // torque_controller和controller把两台电机设为电压转矩控制模式。
  left_motor.torque_controller = TorqueControlType::voltage;
  right_motor.torque_controller = TorqueControlType::voltage;
  left_motor.controller = MotionControlType::torque;
  right_motor.controller = MotionControlType::torque;

  right_motor.useMonitoring(Serial);
  left_motor.useMonitoring(Serial);

  right_motor.init();
  left_motor.init();
  right_motor.initFOC();
  left_motor.initFOC();
}

// runFocAndReadWheelState()先让move()消费上一周期target，再返回本周期轮速快照。
WheelState runFocAndReadWheelState() {
  left_motor.loopFOC();
  right_motor.loopFOC();
  left_motor.move();
  right_motor.move();

  // state保存本次FOC与运动更新后的左右轮轴速度，单位rad/s。
  WheelState state{};
  state.left_velocity_rad_s = left_motor.shaft_velocity;
  state.right_velocity_rad_s = right_motor.shaft_velocity;
  return state;
}

// stageTarget()把command.left_target_v和command.right_target_v写入对应电机target，供下一周期move()使用。
void stageTarget(const MotorVoltageCommand &command) {
  left_motor.target = command.left_target_v;
  right_motor.target = command.right_target_v;
}

} // namespace vehicle
