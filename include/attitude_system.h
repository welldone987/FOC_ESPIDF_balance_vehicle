#pragma once
#include "app_types.h"
namespace vehicle {
bool initializeImu();
ImuSample readImuSample();
class AttitudeEstimator {
public:
  void reset();
  AttitudeEstimate update(const ImuSample &sample);
private:
  float pitch_deg_{0.0F};
  int64_t previous_read_us_{0};
};
}
