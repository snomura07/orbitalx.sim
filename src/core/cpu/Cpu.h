#pragma once

#include "core/common/Types.h"
#include "core/Sensor/BatterySensor.h"
#include "core/Sensor/Encoder.h"

struct DriveCommand {
  double pwmL{0.0};
  double pwmR{0.0};
  double dutyL{0.0};
  double dutyR{0.0};
};

class Cpu {
 public:
  Cpu(const SimParams& params, const Encoder& encoder, const BatterySensor& batterySensor);

  DriveCommand updateDriveCommand(double desiredVelocityMmS, double dtS);
  double desiredVelocityMmS() const;

 private:
  SimParams params;
  const Encoder* encoder{nullptr};
  const BatterySensor* batterySensor{nullptr};
  double rampCmdVelocityMmS{0.0};
  double integralTerm{0.0};
};
