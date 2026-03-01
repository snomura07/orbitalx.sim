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

  DriveCommand updateDriveCommand(double desiredVelocityMmS,
                                  double linePositionMm,
                                  bool lineDetected,
                                  double dtS);
  double desiredVelocityMmS() const;
  double lastVelocityErrorMmS() const;
  double lastLineErrorMm() const;
  double lastBasePwm() const;
  double lastSteerPwm() const;
  double lastOmegaRefRadS() const;
  double lastOmegaErrorRadS() const;
  double lineKp() const;
  double lineKi() const;
  double lineKd() const;

 private:
  SimParams params;
  const Encoder* encoder{nullptr};
  const BatterySensor* batterySensor{nullptr};
  double rampCmdVelocityMmS{0.0};
  double speedIntegralTerm{0.0};
  double lineIntegralTerm{0.0};
  double prevLineErrorMm{0.0};
  double filteredLineErrorMm{0.0};
  bool hasFilteredLineError{false};
  bool hasPrevLineError{false};
  double yawrateIntegralTerm{0.0};
  double prevOmegaErrorRadS{0.0};
  bool hasPrevOmegaError{false};
  double lastVelocityError{0.0};
  double lastLineError{0.0};
  double lastOmegaRef{0.0};
  double lastOmegaError{0.0};
  double lastBasePwmValue{0.0};
  double lastSteerPwmValue{0.0};
};
