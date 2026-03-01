#include "core/cpu/Cpu.h"

#include <algorithm>

Cpu::Cpu(const SimParams& paramsIn,
         const Encoder& encoderIn,
         const BatterySensor& batterySensorIn)
    : params(paramsIn), encoder(&encoderIn), batterySensor(&batterySensorIn) {}

DriveCommand Cpu::updateDriveCommand(double desiredVelocityMmS, double dtS) {
  DriveCommand command{};
  const double vehicleMax = std::max(params.vehicleMaxVelocityMmS, 1.0);
  const double trialMax = std::clamp(params.trialMaxVelocityMmS, 0.0, vehicleMax);
  const double desiredTargetVelocityMmS = std::clamp(desiredVelocityMmS, 0.0, trialMax);
  const double currentVelocityMmS = (encoder != nullptr) ? encoder->readVelocityMmS() : 0.0;
  const double measuredBatteryV = (batterySensor != nullptr) ? batterySensor->readVoltageV() : 0.0;

  if (measuredBatteryV <= params.batteryVMin) {
    integralTerm = 0.0;
    return command;
  }

  const double dvMax = params.accelerationMmSS * dtS;
  const double dv = std::clamp(desiredTargetVelocityMmS - rampCmdVelocityMmS, -dvMax, dvMax);
  rampCmdVelocityMmS += dv;
  rampCmdVelocityMmS = std::clamp(rampCmdVelocityMmS, 0.0, trialMax);

  const double ff = (params.pwmMax / vehicleMax) * rampCmdVelocityMmS;

  const double kp = 0.45 * (params.pwmMax / vehicleMax);
  const double ki = 0.10 * (params.pwmMax / vehicleMax);
  const double error = rampCmdVelocityMmS - currentVelocityMmS;
  integralTerm += error * dtS;
  integralTerm = std::clamp(integralTerm, -1000.0, 1000.0);

  const double pwm = std::clamp(ff + kp * error + ki * integralTerm, 0.0, params.pwmMax);
  command.pwmL = pwm;
  command.pwmR = pwm;
  command.dutyL = pwm / params.pwmMax;
  command.dutyR = pwm / params.pwmMax;
  return command;
}

double Cpu::desiredVelocityMmS() const {
  return rampCmdVelocityMmS;
}
