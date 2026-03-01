#include "core/common/Types.h"
#include "core/cpu/Cpu.h"
#include "core/Sensor/BatterySensor.h"
#include "core/Sensor/Encoder.h"
#include "core/plant/battery/Battery.h"
#include "core/plant/motor/Motor.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

namespace {
constexpr double kDt = 0.01;  // 10ms
volatile std::sig_atomic_t gKeepRunning = 1;

std::string trim(const std::string& s) {
  const auto first = s.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = s.find_last_not_of(" \t\r\n");
  return s.substr(first, last - first + 1);
}

void assignParam(SimParams& p, const std::string& key, double val) {
  if (key == "vehicle_max_velocity_mm_s") p.vehicleMaxVelocityMmS = val;
  else if (key == "trial_max_velocity_mm_s") p.trialMaxVelocityMmS = val;
  else if (key == "acceleration_mm_ss") p.accelerationMmSS = val;
  else if (key == "battery_v_init") p.batteryVInit = val;
  else if (key == "tire_diameter_mm") p.tireDiameterMm = val;
  else if (key == "gear_motor") p.gearMotor = val;
  else if (key == "gear_wheel") p.gearWheel = val;
  else if (key == "mass_g") p.massG = val;
  else if (key == "wheel_base_mm") p.wheelBaseMm = val;
  else if (key == "desired_velocity_mm_s") p.desiredVelocityMmS = val;
  else if (key == "sim_duration_s") p.simDurationS = val;
  else if (key == "pwm_max") p.pwmMax = val;
  else if (key == "motor_R_ohm") p.motorROhm = val;
  else if (key == "motor_Kt_Nm_A") p.motorKtNmA = val;
  else if (key == "motor_kn_rpm_V") p.motorKnRpmV = val;
  else if (key == "motor_I0_A") p.motorI0A = val;
  else if (key == "battery_capacity_Ah") p.batteryCapacityAh = val;
  else if (key == "battery_v_full") p.batteryVFull = val;
  else if (key == "battery_v_empty") p.batteryVEmpty = val;
  else if (key == "battery_v_min") p.batteryVMin = val;
  else if (key == "battery_r_internal_ohm") p.batteryRInternalOhm = val;
  else if (key == "battery_i_mcu_A") p.batteryIMcuA = val;
  else if (key == "eta_gear") p.etaGear = val;
  else if (key == "resist_F0_N") p.resistF0N = val;
  else if (key == "resist_kv_N_per_mps") p.resistKvNPerMps = val;
  else if (key == "resist_k2_N_per_mps2") p.resistK2NPerMps2 = val;
}

void loadParams(const std::string& path, SimParams& p) {
  std::ifstream ifs(path);
  if (!ifs) {
    std::cerr << "config file not found: " << path << " (using defaults)\n";
    return;
  }

  std::string line;
  while (std::getline(ifs, line)) {
    const auto commentPos = line.find('#');
    if (commentPos != std::string::npos) {
      line = line.substr(0, commentPos);
    }
    line = trim(line);
    if (line.empty()) {
      continue;
    }
    const auto eq = line.find('=');
    if (eq == std::string::npos) {
      continue;
    }

    const std::string key = trim(line.substr(0, eq));
    const std::string value = trim(line.substr(eq + 1));
    if (key.empty() || value.empty()) {
      continue;
    }

    try {
      assignParam(p, key, std::stod(value));
    } catch (...) {
      std::cerr << "invalid config value: " << line << '\n';
    }
  }
}

void onSigint(int) {
  gKeepRunning = 0;
}
}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, onSigint);

  SimParams params{};
  const std::string configPath = (argc > 1) ? argv[1] : "config/sim_params.conf";
  loadParams(configPath, params);

  Battery battery(params);
  VehicleState state{};
  Encoder encoder(state);
  BatterySensor batterySensor(battery);
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
    std::cout.flush();

    elapsedS += kDt;
    nextTick += std::chrono::milliseconds(10);
    std::this_thread::sleep_until(nextTick);
  }

  std::cout << "\nSimulation stopped.\n";

  return 0;
}
