#include "app/Run.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

namespace {
constexpr double kDt = 0.01;
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

Run::Run(const SimParams& paramsIn)
    : params(paramsIn),
      battery(params),
      state(),
      encoder(state),
      batterySensor(battery),
      lineSensor(),
      motorL(params),
      motorR(params),
      cpu(params, encoder, batterySensor),
      wsClient(params.wsHost, params.wsPort, params.wsPath) {
  state.vbatV = batterySensor.readVoltageV();
}

int Run::run() {
  gKeepRunning = 1;
  std::signal(SIGINT, onSigint);

  auto nextTick = std::chrono::steady_clock::now();
  while (gKeepRunning) {
    driveCommand = cpu.updateDriveCommand(params.desiredVelocityMmS, kDt);
    applyDriveCommand(driveCommand);
    updatePlant();
    updateBattery();
    updateSensors();
    publishTelemetry();
    renderConsole();

    elapsedS += kDt;
    nextTick += std::chrono::milliseconds(10);
    std::this_thread::sleep_until(nextTick);
  }

  wsClient.close();
  std::cout << "\nSimulation stopped.\n";
  return 0;
}

void Run::applyDriveCommand(const DriveCommand& command) {
  state.dutyL = command.dutyL;
  state.dutyR = command.dutyR;
}

void Run::updatePlant() {
  const double vMps = encoder.readVelocityMmS() / 1000.0;
  const double vbatV = batterySensor.readVoltageV();

  motorLeftResult = motorL.evaluate(state.dutyL, vbatV, vMps);
  motorRightResult = motorR.evaluate(state.dutyR, vbatV, vMps);

  const double driveForceTotalN = motorLeftResult.driveForceN + motorRightResult.driveForceN;
  const double resistN = params.resistF0N +
                         params.resistKvNPerMps * std::abs(vMps) +
                         params.resistK2NPerMps2 * vMps * vMps;
  const double massKg = params.massG / 1000.0;
  const double accelerationMps2 = (driveForceTotalN - resistN) / massKg;
  const double velocityMaxMps = params.vehicleMaxVelocityMmS / 1000.0;
  const double nextVelocityMps = std::clamp(vMps + accelerationMps2 * kDt, 0.0, velocityMaxMps);

  state.vMmS = nextVelocityMps * 1000.0;
  state.vLMmS = state.vMmS;
  state.vRMmS = state.vMmS;

  const double trackM = params.wheelBaseMm / 1000.0;
  const double vLMps = state.vLMmS / 1000.0;
  const double vRMps = state.vRMmS / 1000.0;
  state.omegaRadS = (trackM > 1e-9) ? ((vRMps - vLMps) / trackM) : 0.0;

  state.pose.xMm += state.vMmS * std::cos(state.pose.thetaRad) * kDt;
  state.pose.yMm += state.vMmS * std::sin(state.pose.thetaRad) * kDt;
  state.pose.thetaRad += state.omegaRadS * kDt;
}

void Run::updateBattery() {
  const double totalCurrentA = motorLeftResult.currentA + motorRightResult.currentA + params.batteryIMcuA;
  battery.update(totalCurrentA, kDt);

  state.vbatV = batterySensor.readVoltageV();
  state.vmotLV = state.dutyL * state.vbatV;
  state.vmotRV = state.dutyR * state.vbatV;
  state.iLA = motorLeftResult.currentA;
  state.iRA = motorRightResult.currentA;
}

void Run::updateSensors() {
  lineReading = lineSensor.read(params.whiteLineOffsetMm);
  lineAscii = lineSensor.renderLineAscii(params.whiteLineOffsetMm);
}

void Run::publishTelemetry() {
  if (!params.wsEnabled) {
    return;
  }

  if (!wsClient.isConnected()) {
    if (reconnectCounter <= 0) {
      wsClient.connect();
      reconnectCounter = 100;
    } else {
      --reconnectCounter;
    }
  }
  if (!wsClient.isConnected()) {
    return;
  }

  std::ostringstream msg;
  msg << '{';
  msg << "\"ts\":" << elapsedS << ',';
  msg << "\"velocity\":" << state.vMmS << ',';
  msg << "\"desiredVelocity\":" << cpu.desiredVelocityMmS() << ',';
  msg << "\"batterySoc\":" << batterySensor.readSocPercent() << ',';
  msg << "\"lineDetected\":" << (lineReading.detected ? "true" : "false") << ',';
  msg << "\"xHat\":" << (lineReading.detected ? lineReading.xHatMm : 0.0) << ',';
  msg << "\"lineValues\":[";
  for (size_t i = 0; i < lineReading.values.size(); ++i) {
    if (i != 0) {
      msg << ',';
    }
    msg << lineReading.values[i];
  }
  msg << "],";
  msg << "\"pose\":{";
  msg << "\"x\":" << state.pose.xMm << ',';
  msg << "\"y\":" << state.pose.yMm << ',';
  msg << "\"theta\":" << state.pose.thetaRad;
  msg << "}}";
  if (!wsClient.sendText(msg.str())) {
    wsClient.close();
  }
}

void Run::renderConsole() const {
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
}

