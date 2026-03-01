#include "core/plant/motor/Motor.h"

#include <algorithm>
#include <cmath>

namespace {
constexpr double kPi = 3.14159265358979323846;
}

Motor::Motor(const SimParams& paramsIn) : params(paramsIn) {}

MotorStepResult Motor::evaluate(double duty, double vbatV, double wheelVMps) const {
  MotorStepResult out{};

  const double dutyClamped = std::clamp(duty, 0.0, 1.0);
  out.vmotV = dutyClamped * vbatV;

  const double wheelRadiusM = (params.tireDiameterMm / 1000.0) * 0.5;
  const double wheelRps = (wheelRadiusM > 1e-9)
                               ? (wheelVMps / (2.0 * kPi * wheelRadiusM))
                               : 0.0;
  const double wheelRpm = wheelRps * 60.0;
  const double gearRatio = params.gearWheel / params.gearMotor;
  const double motorRpm = wheelRpm * gearRatio;

  out.bemfV = motorRpm / params.motorKnRpmV;
  out.currentA = (out.vmotV - out.bemfV) / params.motorROhm;
  out.currentA = std::max(0.0, out.currentA);

  out.torqueMotorNm = params.motorKtNmA * std::max(0.0, out.currentA - params.motorI0A);
  out.torqueWheelNm = out.torqueMotorNm * gearRatio * params.etaGear;
  out.driveForceN = (wheelRadiusM > 1e-9) ? (out.torqueWheelNm / wheelRadiusM) : 0.0;
  return out;
}
