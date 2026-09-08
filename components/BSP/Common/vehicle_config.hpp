#pragma once

#include <cstddef>
#include <cstdint>

namespace vehicle {
namespace config {

/*
 * 车辆配置把控制周期、传感器量程、执行器缩放和通信协议参数集中提供给各模块。
 * 控制器使用这里的单位与限幅，BSP驱动使用这里的硬件量程和方向约定。
 */

// kControlPeriodUs是控制任务相邻释放时刻的目标间隔，单位us。
inline constexpr std::uint32_t kControlRateHz = 1000U;
inline constexpr std::uint64_t kControlPeriodUs =
    1000000ULL / kControlRateHz;

// TCP调试开关：修改后重新编译；关闭时Wi-Fi仍连接，但不监听或发送TCP。
inline constexpr bool kWifiTcpDebugEnabled = true;

// BLE字符串和缩放常量保持网页控制端的整数协议不变。
inline constexpr char kBleDeviceName[] = "平衡车";
inline constexpr char kBleServiceUuid[] = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
inline constexpr char kBleCommandUuid[] = "6e400002-b5a3-f393-e0a9-e50e24dcca9e";
inline constexpr char kBleInitialValue[] = "欢迎来到平衡车";
inline constexpr std::size_t kMaximumBleCommandLength = 63U;
inline constexpr float kMaximumThrottleVelocityRadS = 10.0f;
inline constexpr float kMaximumSteeringVoltageV = 10.0f;
inline constexpr float kBleFullScaleSteering = 1500.0f;
inline constexpr float kBleFullScaleThrottle = 40.0f;
// 新协议100%转向保持原网页X=100的实际差分电压，避免放大15倍。
inline constexpr float kRemoteSteeringVoltageV =
    kMaximumSteeringVoltageV * 100.0f / kBleFullScaleSteering;

// BMI160通过I2C0读取；原始加速度和陀螺仪计数分别按这里的量程因子换算。
inline constexpr std::uint8_t kBmi160Address = 0x69U;
inline constexpr std::uint32_t kI2cFrequencyHz = 400000U;
inline constexpr float kBmi160AccelerationScale = 16384.0f;
inline constexpr float kBmi160GyroScale = 32.8f;
inline constexpr std::uint32_t kBmi160FocTimeoutMs = 500U;

// 电机方向把左右编码器速度和目标电压统一到车辆前进坐标系。
inline constexpr int kMotorPolePairs = 7;
inline constexpr float kMotorSupplyVoltageV = 12.0f;
inline constexpr float kMotorSensorAlignmentVoltageV = 2.0f;
inline constexpr float kSimpleFocVelocityPidP = 0.01f;
inline constexpr float kSimpleFocVelocityPidI = 0.10f;
inline constexpr float kSimpleFocVelocityPidD = 0.0f;
inline constexpr float kMotor0Direction = 1.0f;
inline constexpr float kMotor1Direction = 1.0f;

// kBatteryVoltageScale恢复7.5k/1k分压前的母线电压，kStartupUndervoltageThresholdV用于启动检查。
inline constexpr float kStartupUndervoltageThresholdV = 9.0f;
inline constexpr float kBatteryVoltageScale = 8.5f;
inline constexpr std::uint32_t kAdcDefaultVrefMv = 1100U;

// 速度环输出目标俯仰角，姿态环输出左右电机的平衡电压，单位分别为deg和V。
inline constexpr float kPitchOffsetDeg = 1.8f;
inline constexpr float kBalancePidP = 0.31f;
inline constexpr float kBalancePidI = 0.0f;
inline constexpr float kBalancePidD = 0.001f;
inline constexpr float kBalancePidRamp = 100000.0f;
inline constexpr float kBalancePidLimitV = 6.0f;
inline constexpr float kSpeedPidP = 1.50f;
inline constexpr float kSpeedPidI = 0.0f;
inline constexpr float kSpeedPidD = 0.05f;
inline constexpr float kSpeedPidRamp = 10000.0f;
inline constexpr float kSpeedPidLimitDeg = 6.0f;

// 三个一阶低通时间常数分别作用于目标俯仰角、油门轮速和转向电压，单位s。
inline constexpr float kPitchCommandFilterTfS = 0.07f;
inline constexpr float kThrottleFilterTfS = 0.5f;
inline constexpr float kSteeringFilterTfS = 0.1f;

} // namespace config
} // namespace vehicle
