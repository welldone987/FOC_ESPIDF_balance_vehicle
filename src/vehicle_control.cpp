#include "vehicle_control.h"

#include <Arduino.h>
#include <esp_timer.h>
#include "telemetry_runtime.h"

#include "attitude_system.h"
#include "command_input.h"
#include "motor_foc_service.h"
#include "vehicle_safety.h"
#include "vehicle_config.h"

namespace vehicle {
namespace {

// 算法对象在启动前构造并持续保存历史，控制周期内不重建。
AttitudeEstimator attitude_estimator{};
VehicleController controller{};
// telemetry保存最近一轮完整快照，仅由控制执行链读写。
TelemetrySnapshot telemetry{};
// previous_cycle_started_us记录上一轮起点，单位us；首轮周期差仍为0。
uint32_t previous_cycle_started_us{0U};
// startup_bus_voltage_v和imu_ready保存启动检查结果，不构成运行期安全联锁。
float startup_bus_voltage_v{0.0F};
bool imu_ready{false};

} // namespace

// mixMotorVoltages()按motor0_direction和motor1_direction生成左右目标电压。
MotorVoltageCommand VehicleController::mixMotorVoltages(
    float balance_voltage_v,
    float steering_voltage_v) {
  // command保存混合后的左右电机目标电压。
  MotorVoltageCommand command{};
  command.left_target_v =
      static_cast<float>(config::motor0_direction) *
      (balance_voltage_v + steering_voltage_v);
  command.right_target_v =
      static_cast<float>(config::motor1_direction) *
      (balance_voltage_v - steering_voltage_v);
  return command;
}

VehicleController::VehicleController()
    : balance_pid_{config::balance_pid_p,
                   config::balance_pid_i,
                   config::balance_pid_d,
                   config::balance_pid_ramp,
                   config::balance_pid_limit_v},
      speed_pid_{config::speed_pid_p,
                 config::speed_pid_i,
                 config::speed_pid_d,
                 config::speed_pid_ramp,
                 config::speed_pid_limit_deg},
      pitch_command_filter_{config::pitch_command_filter_time_constant_s},
      throttle_filter_{config::throttle_filter_time_constant_s},
      steering_filter_{config::steering_filter_time_constant_s} {}

ControlOutput VehicleController::update(const RemoteCommand &command,
                                        const WheelState &wheels,
                                        const AttitudeEstimate &attitude) {
  // directed_average_velocity_rad_s保存方向统一后的左右轮平均角速度，单位rad/s。
  const float directed_average_velocity_rad_s{
      (static_cast<float>(config::motor0_direction) *
           wheels.left_velocity_rad_s +
       static_cast<float>(config::motor1_direction) *
           wheels.right_velocity_rad_s) /
      2.0F};
  // filtered_throttle_velocity_rad_s保存滤波后的目标轮速，单位rad/s。
  const float filtered_throttle_velocity_rad_s{
      throttle_filter_(command.throttle_velocity_rad_s)};
  // speed_error_rad_s保存当前平均轮速与滤波后目标轮速之差，单位rad/s。
  const float speed_error_rad_s{
      directed_average_velocity_rad_s - filtered_throttle_velocity_rad_s};
  // target_pitch_deg保存速度外环生成的目标俯仰角，单位deg。
  const float target_pitch_deg{
      pitch_command_filter_(speed_pid_(speed_error_rad_s))};
  // pitch_error_deg保存config::pitch_offset_deg与估计俯仰角之差，单位deg。
  const float pitch_error_deg{
      config::pitch_offset_deg - attitude.pitch_deg};
  // balance_voltage_v保存姿态内环输出的共同驱动电压，单位V。
  const float balance_voltage_v{
      balance_pid_(pitch_error_deg + target_pitch_deg)};
  // steering_voltage_v保存转向低通输出的差分电压，单位V。
  const float steering_voltage_v{
      steering_filter_(command.steering_voltage_v)};

  // output收集本轮控制中间量和左右电机目标电压。
  ControlOutput output{};
  output.pitch_error_deg = pitch_error_deg;
  output.target_pitch_deg = target_pitch_deg;
  output.balance_voltage_v = balance_voltage_v;
  output.steering_voltage_v = steering_voltage_v;
  output.motor_voltage =
      mixMotorVoltages(balance_voltage_v, steering_voltage_v);
  return output;
}

void initializeVehicle() {
  startup_bus_voltage_v = waitForStartupVoltage();
  Serial.begin(config::serial_baud);
  initializePinsAndEncoders();
  // BLE初始化失败时把服务启动结果记录到串口。
  if (!beginCommandInput()) {
    Serial.println("BLE 初始化请求失败");
  }
  imu_ready = initializeImu();
  initializeMotors();
  attitude_estimator.reset(millis());
}

void runVehicleControlOnce() {
  // cycle_started_us记录本轮控制周期起点，单位us。
  const uint32_t cycle_started_us{micros()};
  const int64_t device_time_us{esp_timer_get_time()};

  // runFocAndReadWheelState()消耗上一轮target并返回本轮左右轮速快照。
  const WheelState wheels{runFocAndReadWheelState()};
  // sampled_ms作为本轮IMU采样和姿态估计的统一时戳，单位ms。
  const uint32_t sampled_ms{millis()};
  // imu保存本轮BMI160换算后的ImuSample。
  const ImuSample imu{readImuSample(sampled_ms)};
  // attitude保存本轮互补滤波得到的AttitudeEstimate。
  const AttitudeEstimate attitude{attitude_estimator.update(imu)};
  // command保存本轮读取的RemoteCommand。
  const RemoteCommand command{latestRemoteCommand()};
  // control保存本轮速度、平衡和转向控制输出。
  const ControlOutput control{controller.update(command, wheels, attitude)};
  // stageTarget()把本轮control.motor_voltage保存为下一轮电机目标。
  stageTarget(control.motor_voltage);

  telemetry.sequence += 1U;
  telemetry.device_time_us = device_time_us;
  telemetry.sampled_ms = sampled_ms;
  telemetry.control_interval_us =
      previous_cycle_started_us == 0U
          ? 0U
          : cycle_started_us - previous_cycle_started_us;
  telemetry.command = command;
  telemetry.imu = imu;
  telemetry.attitude = attitude;
  telemetry.wheels = wheels;
  telemetry.control = control;
  telemetry.bus_voltage_v = startup_bus_voltage_v;
  telemetry.imu_ready = imu_ready;
  previous_cycle_started_us = cycle_started_us;
  publishTelemetry(telemetry);
}

const TelemetrySnapshot &latestTelemetry() {
  return telemetry;
}

} // namespace vehicle
