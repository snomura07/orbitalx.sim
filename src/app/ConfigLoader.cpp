#include "app/ConfigLoader.h"

#include <fstream>
#include <iostream>
#include <string>

namespace {
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
  else if (key == "white_line_offset_mm") p.whiteLineOffsetMm = val;
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
}  // namespace

SimParams ConfigLoader::load(const std::string& path) {
  SimParams params{};
  std::ifstream ifs(path);
  if (!ifs) {
    std::cerr << "config file not found: " << path << " (using defaults)\n";
    return params;
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
      assignParam(params, key, std::stod(value));
    } catch (...) {
      std::cerr << "invalid config value: " << line << '\n';
    }
  }

  return params;
}

