#include "app/Run.h"

#include "core/cpu/Cpu.h"
#include "core/Sensor/BatterySensor.h"
#include "core/Sensor/Encoder.h"
#include "core/Sensor/LineSensor.h"
#include "core/plant/battery/Battery.h"
#include "core/plant/motor/Motor.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

namespace {
constexpr double kDt = 0.01;  // 10ms
volatile std::sig_atomic_t gKeepRunning = 1;

void onSigint(int) {
  gKeepRunning = 0;
}

std::string formatArray15(const std::array<double, 15>& values) {
  std::ostringstream oss;
  oss << '[';
  for (size_t i = 0; i < values.size(); ++i) {
    if (i != 0) {
      oss << ", ";
    }
    oss << std::fixed << std::setprecision(2) << values[i];
  }
  oss << ']';
  return oss.str();
}
}  // namespace

Run::Run(const SimParams& paramsIn) : params(paramsIn) {}

int Run::run() {
  gKeepRunning = 1;
  std::signal(SIGINT, onSigint);

  Battery battery(params);
  VehicleState state{};
  Encoder encoder(state);
  BatterySensor batterySensor(battery);
  LineSensor lineSensor{};
  Motor motorL(params);
  Motor motorR(params);
  Cpu cpu(params, encoder, batterySensor);
  state.vbatV = batterySensor.readVoltageV();
  double elapsedS = 0.0;

  auto nextTick = std::chrono::steady_clock::now();
  while (gKeepRunning) {
    const double pwm = cpu.updatePwm(params.desiredVelocityMmS, kDt);
    state.dutyL = pwm / params.pwmMax;
    state.dutyR = pwm / params.pwmMax;

    const double vMps = encoder.readVelocityMmS() / 1000.0;
    const double vbatV = batterySensor.readVoltageV();
    const auto l = motorL.evaluate(state.dutyL, vbatV, vMps);
    const auto r = motorR.evaluate(state.dutyR, vbatV, vMps);

    const double driveForceTotalN = l.driveForceN + r.driveForceN;
    const double resistN = params.resistF0N +
                           params.resistKvNPerMps * std::abs(vMps) +
                           params.resistK2NPerMps2 * vMps * vMps;
    const double massKg = params.massG / 1000.0;
    const double accelerationMps2 = (driveForceTotalN - resistN) / massKg;

    double vNextMps = vMps + accelerationMps2 * kDt;
    vNextMps = std::clamp(vNextMps, 0.0, params.vehicleMaxVelocityMmS / 1000.0);

    state.vMmS = vNextMps * 1000.0;
    state.vLMmS = state.vMmS;
    state.vRMmS = state.vMmS;

    const double trackM = params.wheelBaseMm / 1000.0;
    const double vLMps = state.vLMmS / 1000.0;
    const double vRMps = state.vRMmS / 1000.0;
    state.omegaRadS = (trackM > 1e-9) ? ((vRMps - vLMps) / trackM) : 0.0;

    state.pose.xMm += state.vMmS * std::cos(state.pose.thetaRad) * kDt;
    state.pose.yMm += state.vMmS * std::sin(state.pose.thetaRad) * kDt;
    state.pose.thetaRad += state.omegaRadS * kDt;

    const double totalCurrentA = l.currentA + r.currentA + params.batteryIMcuA;
    battery.update(totalCurrentA, kDt);

    state.vbatV = batterySensor.readVoltageV();
    state.vmotLV = state.dutyL * state.vbatV;
    state.vmotRV = state.dutyR * state.vbatV;
    state.iLA = l.currentA;
    state.iRA = r.currentA;

    const auto lineReading = lineSensor.read(params.whiteLineOffsetMm);
    const std::string lineAscii = lineSensor.renderLineAscii(params.whiteLineOffsetMm);

    std::cout << "\x1b[2J\x1b[H";
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Robot Trace Simulator (Ctrl+C to stop)\n";
    std::cout << "--------------------------------------\n";
    std::cout << "Time [s]             : " << elapsedS << '\n';
    std::cout << "Desired v [mm/s]     : " << cpu.desiredVelocityMmS() << '\n';
    std::cout << "Velocity [mm/s]      : " << state.vMmS << '\n';
    std::cout << "Omega [rad/s]        : " << state.omegaRadS << '\n';
    std::cout << "Battery Vbat [V]     : " << state.vbatV << '\n';
    std::cout << "Battery SoC [%]      : " << batterySensor.readSocPercent() << '\n';
    std::cout << "Motor V L/R [V]      : " << state.vmotLV << " / " << state.vmotRV << '\n';
    std::cout << "Motor I L/R [A]      : " << state.iLA << " / " << state.iRA << '\n';
    std::cout << "Pose x,y [mm]        : " << state.pose.xMm << ", " << state.pose.yMm << '\n';
    std::cout << "Pose theta [rad]     : " << state.pose.thetaRad << '\n';
    std::cout << "PWM Duty L/R         : " << state.dutyL << " / " << state.dutyR << '\n';
    std::cout << "Line Pattern         : " << lineAscii << '\n';
    std::cout << "Line Sensor[15]      : " << formatArray15(lineReading.values) << '\n';
    if (lineReading.detected) {
      std::cout << "Estimated x_hat [mm] : " << lineReading.xHatMm << '\n';
    } else {
      std::cout << "Estimated x_hat [mm] : UNDETECTED\n";
    }
    std::cout.flush();

    elapsedS += kDt;
    nextTick += std::chrono::milliseconds(10);
    std::this_thread::sleep_until(nextTick);
  }

  std::cout << "\nSimulation stopped.\n";
  return 0;
}

