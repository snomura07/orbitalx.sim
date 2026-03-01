#include "core/plant/battery/Battery.h"

#include <algorithm>

namespace {
double clamp01(double x) {
  return std::clamp(x, 0.0, 1.0);
}
}  // namespace

Battery::Battery(const SimParams& paramsIn) : params(paramsIn) {
  const double denom = (params.batteryVFull - params.batteryVEmpty);
  if (denom > 1e-9) {
    socValue = clamp01((params.batteryVInit - params.batteryVEmpty) / denom);
  }
  vbatV = std::max(params.batteryVMin, params.batteryVInit);
}

void Battery::update(double totalCurrentA, double dtS) {
  const double capacityAs = params.batteryCapacityAh * 3600.0;
  if (capacityAs > 1e-9) {
    socValue -= (totalCurrentA * dtS) / capacityAs;
  }
  socValue = clamp01(socValue);
  vbatV = ocv(socValue) - totalCurrentA * params.batteryRInternalOhm;
  vbatV = std::clamp(vbatV, params.batteryVMin, params.batteryVFull);
}

double Battery::voltage() const {
  return vbatV;
}

double Battery::soc() const {
  return socValue;
}

double Battery::ocv(double socIn) const {
  const double socClamped = clamp01(socIn);
  return params.batteryVEmpty +
         (params.batteryVFull - params.batteryVEmpty) * socClamped;
}
