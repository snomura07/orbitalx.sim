#pragma once

#include "core/common/Types.h"
#include "core/Sensor/BatterySensor.h"
#include "core/Sensor/Encoder.h"

class Cpu {
 public:
  Cpu(const SimParams& params, const Encoder& encoder, const BatterySensor& batterySensor);

  double updatePwm(double desiredVelocityMmS, double dtS);
  double desiredVelocityMmS() const;

 private:
  SimParams params;
  const Encoder* encoder{nullptr};
  const BatterySensor* batterySensor{nullptr};
  double rampCmdVelocityMmS{0.0};
  double integralTerm{0.0};
};
