#pragma once

#include "core/common/Types.h"

struct MotorStepResult {
  double vmotV{0.0};
  double bemfV{0.0};
  double currentA{0.0};
  double torqueMotorNm{0.0};
  double torqueWheelNm{0.0};
  double driveForceN{0.0};
};

class Motor {
 public:
  explicit Motor(const SimParams& params);

  MotorStepResult evaluate(double duty, double vbatV, double wheelVMps) const;

 private:
  SimParams params;
};
