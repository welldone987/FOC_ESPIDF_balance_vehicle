#include "attitude_system.h"
#include "vehicle_config.h"
#include <Arduino.h>
#include <Wire.h>
#include <esp_timer.h>
#include <cmath>

namespace vehicle {
namespace {
// 返回真实事务结果；运行期不打印、不重试、不读取FIFO。
bool readRegisters(uint8_t address, uint8_t *data, uint8_t length) {
  Wire.beginTransmission(config::bmi160_i2c_address);
  Wire.write(address);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(config::bmi160_i2c_address, length) != length) return false;
  for (uint8_t i = 0; i < length; ++i) data[i] = static_cast<uint8_t>(Wire.read());
  return true;
}
bool writeRegister(uint8_t address, uint8_t value) {
  Wire.beginTransmission(config::bmi160_i2c_address);
  Wire.write(address); Wire.write(value);
  return Wire.endTransmission() == 0;
}
int16_t signedWord(const uint8_t *bytes) {
  const uint16_t raw{static_cast<uint16_t>(bytes[0] | (static_cast<uint16_t>(bytes[1]) << 8))};
  return static_cast<int16_t>(raw);
}
}

bool initializeImu() {
  // BMI160 Rev1.0: CMD复位/正常模式，ACC/GYR_CONF设置1600Hz、OSR2。
  // 用有界寄存器事务替代库中无超时的自动校准等待。
  uint8_t value{};
  if (!readRegisters(0x00, &value, 1) || value != 0xD1) return false;
  if (!writeRegister(0x7E, 0xB6)) return false;
  delay(100);
  if (!writeRegister(0x7E, 0x11)) return false;
  delay(10);
  if (!writeRegister(0x7E, 0x15)) return false;
  delay(100);
  if (!writeRegister(0x40, 0x1C) || !writeRegister(0x41, 0x03) ||
      !writeRegister(0x42, 0x1C) || !writeRegister(0x43, 0x01)) return false;
  delay(20);
  uint8_t settings[4]{};
  if (!readRegisters(0x40, settings, 4) || settings[0] != 0x1C ||
      settings[1] != 0x03 || settings[2] != 0x1C || settings[3] != 0x01) return false;
  if (!readRegisters(0x03, &value, 1) || (value & 0x3C) != 0x14) return false;
  if (!readRegisters(0x02, &value, 1) || value != 0U) return false;
  // 静止校准陀螺仪；FOC_CONF.gyr_en，CMD.start_foc，STATUS.foc_rdy。
  if (!writeRegister(0x69, 0x40) || !writeRegister(0x7E, 0x03)) return false;
  const uint32_t started{millis()};
  do {
    delay(1);
    if (!readRegisters(0x1B, &value, 1)) return false;
    if ((value & 0x08) != 0U) break;
    if (millis() - started >= 500U) return false;
  } while (true);
  if (!readRegisters(0x77, &value, 1) || !writeRegister(0x77, value | 0x80)) return false;
  delay(20);
  return readImuSample().valid;
}

ImuSample readImuSample() {
  ImuSample sample{};
  sample.read_started_us = esp_timer_get_time();
  uint8_t raw[12]{};
  if (!readRegisters(0x0C, raw, sizeof(raw))) return sample;
  sample.pitch_rate_deg_s = signedWord(raw + 2) / config::bmi160_gyro_scale;
  sample.acceleration_x_g = signedWord(raw + 6) / config::bmi160_acceleration_scale;
  sample.acceleration_y_g = signedWord(raw + 8) / config::bmi160_acceleration_scale;
  sample.acceleration_z_g = signedWord(raw + 10) / config::bmi160_acceleration_scale;
  sample.valid = true;
  return sample;
}

void AttitudeEstimator::reset() { pitch_deg_ = 0.0F; previous_read_us_ = 0; }
AttitudeEstimate AttitudeEstimator::update(const ImuSample &sample) {
  AttitudeEstimate result{};
  if (!sample.valid) return result;
  const int64_t elapsed{sample.read_started_us - previous_read_us_};
  const float acc_pitch{static_cast<float>(std::atan2(sample.acceleration_x_g,
      sample.acceleration_z_g + std::fabs(sample.acceleration_y_g)) * -180.0 / PI)};
  if (previous_read_us_ == 0) {
    pitch_deg_ = acc_pitch;
  } else {
    if (elapsed <= 0 || elapsed > config::maximum_attitude_interval_us) return result;
    result.interval_s = static_cast<float>(elapsed) * 1.0e-6F;
    const float alpha{config::attitude_time_constant_s /
        (config::attitude_time_constant_s + result.interval_s)};
    pitch_deg_ = alpha * (pitch_deg_ + sample.pitch_rate_deg_s * result.interval_s) +
                 (1.0F - alpha) * acc_pitch;
  }
  previous_read_us_ = sample.read_started_us;
  result.pitch_deg = pitch_deg_;
  result.pitch_rate_deg_s = sample.pitch_rate_deg_s;
  result.valid = std::isfinite(pitch_deg_);
  return result;
}
}
