#include "core/cpu/Cpu.h"

#include <algorithm>
#include <cmath>

Cpu::Cpu(const SimParams& paramsIn,
         const Encoder& encoderIn,
         const BatterySensor& batterySensorIn)
    : params(paramsIn), encoder(&encoderIn), batterySensor(&batterySensorIn) {}

DriveCommand Cpu::updateDriveCommand(double desiredVelocityMmS,
                                     double linePositionMm,
                                     bool lineDetected,
                                     double dtS) {
  constexpr double kLineErrorDeadbandMm = 0.25;
  constexpr double kLineFilterTauS = 0.02;
  constexpr double kLineDerivativeLimitMmS = 600.0;
  DriveCommand command{};
  const double vehicleMax = std::max(params.vehicleMaxVelocityMmS, 1.0);
  const double desiredTargetVelocityMmS = std::clamp(desiredVelocityMmS, 0.0, vehicleMax);
  const double currentVelocityMmS = (encoder != nullptr) ? encoder->readVelocityMmS() : 0.0;
  const double measuredBatteryV = (batterySensor != nullptr) ? batterySensor->readVoltageV() : 0.0;

  if (measuredBatteryV <= params.batteryVMin) {
    speedIntegralTerm = 0.0;
    lineIntegralTerm = 0.0;
    hasFilteredLineError = false;
    hasPrevLineError = false;
    lastVelocityError = 0.0;
    lastLineError = 0.0;
    lastBasePwmValue = 0.0;
    lastSteerPwmValue = 0.0;
    return command;
  }

  const double dvMax = params.accelerationMmSS * dtS;
  const double dv = std::clamp(desiredTargetVelocityMmS - rampCmdVelocityMmS, -dvMax, dvMax);
  rampCmdVelocityMmS += dv;
  rampCmdVelocityMmS = std::clamp(rampCmdVelocityMmS, 0.0, vehicleMax);

  const double ff = (params.pwmMax / vehicleMax) * rampCmdVelocityMmS;

  const double speedKp = params.speedKp * (params.pwmMax / vehicleMax);
  const double speedKi = params.speedKi * (params.pwmMax / vehicleMax);
  const double velocityError = rampCmdVelocityMmS - currentVelocityMmS;
  speedIntegralTerm += velocityError * dtS;
  speedIntegralTerm = std::clamp(speedIntegralTerm, -1000.0, 1000.0);
  lastVelocityError = velocityError;

  double lineInputMm = 0.0;
  double lineDerivative = 0.0;
  if (lineDetected) {
    double lineError = 0.0 - linePositionMm;
    if (std::abs(lineError) < kLineErrorDeadbandMm) {
      lineError = 0.0;
    }

    if (!hasFilteredLineError) {
      filteredLineErrorMm = lineError;
      hasFilteredLineError = true;
    } else {
      const double alpha = dtS / (kLineFilterTauS + dtS);
      filteredLineErrorMm += alpha * (lineError - filteredLineErrorMm);
    }

    const double lineErrorDelta =
        hasPrevLineError ? (filteredLineErrorMm - prevLineErrorMm) : 0.0;
    lineDerivative =
        (dtS > 1e-9)
            ? std::clamp(lineErrorDelta / dtS, -kLineDerivativeLimitMmS, kLineDerivativeLimitMmS)
            : 0.0;
    lineIntegralTerm += filteredLineErrorMm * dtS;
    lineIntegralTerm =
        std::clamp(lineIntegralTerm, -params.lineIntegralLimit, params.lineIntegralLimit);
    prevLineErrorMm = filteredLineErrorMm;
    hasPrevLineError = true;
    lineInputMm = filteredLineErrorMm;
    lastLineError = filteredLineErrorMm;
  } else {
    // Hold the last detected line error when the line is temporarily lost.
    lineInputMm = lastLineError;
    lineDerivative = 0.0;
  }

  const double steerPwmRaw = params.lineKp * lineInputMm +
                             params.lineKi * lineIntegralTerm +
                             params.lineKd * lineDerivative;
  const double steerPwm = std::clamp(steerPwmRaw, -params.pwmMax, params.pwmMax);

  // Preserve steering authority: reserve PWM headroom for differential component.
  const double baseHeadroom = std::max(0.0, params.pwmMax - std::abs(steerPwm));
  const double basePwmRaw = ff + speedKp * velocityError + speedKi * speedIntegralTerm;
  const double basePwm = std::clamp(basePwmRaw, 0.0, baseHeadroom);
  lastBasePwmValue = basePwm;
  lastSteerPwmValue = steerPwm;

  const double pwmL = std::clamp(basePwm - steerPwm, -params.pwmMax, params.pwmMax);
  const double pwmR = std::clamp(basePwm + steerPwm, -params.pwmMax, params.pwmMax);
  command.pwmL = pwmL;
  command.pwmR = pwmR;
  command.dutyL = pwmL / params.pwmMax;
  command.dutyR = pwmR / params.pwmMax;
  return command;
}

double Cpu::desiredVelocityMmS() const {
  return rampCmdVelocityMmS;
}

double Cpu::lastVelocityErrorMmS() const {
  return lastVelocityError;
}

double Cpu::lastLineErrorMm() const {
  return lastLineError;
}

double Cpu::lastBasePwm() const {
  return lastBasePwmValue;
}

double Cpu::lastSteerPwm() const {
  return lastSteerPwmValue;
}

double Cpu::lineKp() const {
  return params.lineKp;
}

double Cpu::lineKi() const {
  return params.lineKi;
}

double Cpu::lineKd() const {
  return params.lineKd;
}
