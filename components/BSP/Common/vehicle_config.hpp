#pragma once

#include <cstddef>
#include <cstdint>

namespace vehicle {
namespace config {

inline constexpr char kBleDeviceName[] = "平衡车";
inline constexpr char kBleServiceUuid[] = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
inline constexpr char kBleCommandUuid[] = "6e400002-b5a3-f393-e0a9-e50e24dcca9e";
inline constexpr char kBleInitialValue[] = "欢迎来到平衡车";
inline constexpr std::size_t kMaximumBleCommandLength = 63U;
inline constexpr float kMaximumThrottleVelocityRadS = 10.0f;
inline constexpr float kMaximumSteeringVoltageV = 10.0f;
inline constexpr float kBleFullScaleSteering = 1500.0f;
inline constexpr float kBleFullScaleThrottle = 40.0f;

inline constexpr std::uint8_t kBmi160Address = 0x69U;
inline constexpr std::uint32_t kI2cFrequencyHz = 400000U;
inline constexpr float kBmi160AccelerationScale = 16384.0f;
inline constexpr float kBmi160GyroScale = 32.8f;
inline constexpr std::uint32_t kBmi160FocTimeoutMs = 500U;

inline constexpr int kMotorPolePairs = 7;
inline constexpr float kMotorSupplyVoltageV = 12.0f;
inline constexpr float kMotorSensorAlignmentVoltageV = 2.0f;
inline constexpr float kSimpleFocVelocityPidP = 0.01f;
inline constexpr float kSimpleFocVelocityPidI = 0.10f;
inline constexpr float kSimpleFocVelocityPidD = 0.0f;
inline constexpr float kMotor0Direction = 1.0f;
inline constexpr float kMotor1Direction = 1.0f;

inline constexpr float kStartupUndervoltageThresholdV = 9.0f;
inline constexpr float kBatteryVoltageScale = 8.5f;
inline constexpr std::uint32_t kStartupPowerPollIntervalMs = 100U;
inline constexpr std::uint32_t kAdcDefaultVrefMv = 1100U;

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
inline constexpr float kPitchCommandFilterTfS = 0.07f;
inline constexpr float kThrottleFilterTfS = 0.5f;
inline constexpr float kSteeringFilterTfS = 0.1f;

} // namespace config
} // namespace vehicle
