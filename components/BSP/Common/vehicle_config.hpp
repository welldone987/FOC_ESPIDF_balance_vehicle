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

// 电机方向把左右编码器速度和目标电流统一到车辆前进坐标系。
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

// 三环调度：电流1kHz，姿态500Hz，速度/转向100Hz；频率不是实测带宽。
inline constexpr unsigned kAttitudeDivider = 2U;
inline constexpr float kOuterPeriodS = 0.010f;
inline constexpr float kMaximumControlGapS = 0.010f;
inline constexpr float kFallAngleDeg = 30.0f;
inline constexpr float kRadToDeg = 57.295779513f;
inline constexpr float kGravityMps2 = 9.81f;

// 必须完成相序、极性、坐标、允许电流及参数台架核验后显式修改。
// false时在驱动器初始化前返回ESP_ERR_INVALID_STATE，不执行电机对齐。
inline constexpr bool kCurrentHardwareVerified = false;
inline constexpr float kCurrentShuntOhm = 0.01f;
inline constexpr float kCurrentAmplifierGain = 50.0f;
inline constexpr unsigned kCurrentOffsetSamples = 1000U;
inline constexpr int kCurrentAdcMinMv = 150;
inline constexpr int kCurrentAdcMaxMv = 2450;
inline constexpr int kCurrentOffsetMinMv = 1300;
inline constexpr int kCurrentOffsetMaxMv = 1900;
inline constexpr int kCurrentOffsetNoiseMv = 100;
inline constexpr float kCurrentLimitA = 1.0f;
inline constexpr float kPhaseTripA = 1.3f;
inline constexpr std::int64_t kCurrentSampleMaxAgeUs = 2000;
// OUT1/OUT2暂按A/B相、正增益；硬件确认开关同时声明这两项已实测。
inline constexpr float kCurrentPolarity = 1.0f;
// 台架占位值：不是实测电机参数，也不直接复制MIL增益。
inline constexpr float kCurrentKp = 0.5f;  // V/A
inline constexpr float kCurrentKi = 20.0f; // V/(A*s)
inline constexpr float kCurrentFilterS = 0.0005f;
// d/q分别限制到3V，矢量幅值<=4.25V，小于标称12V母线的一半。
inline constexpr float kCurrentAxisVoltageV = 3.0f;

// 几何及外环均为待标定值；正前倾/正前进坐标须与IMU、编码器核对。
inline constexpr float kWheelRadiusM = 0.04f;
inline constexpr float kWheelTrackM = 0.18f;
inline constexpr float kDriveSpeedLimitRadS = 2.0f;
inline constexpr float kWheelAccelerationRadS2 = 5.0f;
inline constexpr float kYawRateLimitRadS = 0.5f;
inline constexpr float kYawAccelerationRadS2 = 0.5f;
inline constexpr float kPitchOffsetDeg = 1.8f;
inline constexpr float kPitchLimitDeg = 3.0f;
inline constexpr float kAttitudeKp = 0.08f;  // A/deg，待标定
inline constexpr float kAttitudeKd = 0.01f;  // A/(deg/s)，待标定
inline constexpr float kSpeedKp = 1.0f;      // deg/(rad/s)，待标定
inline constexpr float kSpeedKi = 0.0f;      // 外环初调关闭积分
inline constexpr float kYawKp = 0.2f;        // A/(rad/s)，待标定
inline constexpr float kYawKi = 0.0f;
// 前馈已接入，初调关闭；完成对应辨识后再逐项启用。
inline constexpr float kAccelerationFeedforward = 0.0f;
inline constexpr float kYawAccelerationFeedforward = 0.0f;
inline constexpr float kYawRateFeedforward = 0.0f;
static_assert(kCurrentLimitA > 0.0f && kPhaseTripA > kCurrentLimitA);
static_assert(kWheelRadiusM > 0.0f && kWheelTrackM > 0.0f);
static_assert(kAttitudeDivider > 0U);

} // namespace config
} // namespace vehicle
