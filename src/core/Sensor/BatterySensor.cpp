#include "core/Sensor/BatterySensor.h"

#include "core/plant/battery/Battery.h"

BatterySensor::BatterySensor(const Battery& battery) : batteryModule(&battery) {}

void BatterySensor::connect(const Battery& battery) {
  batteryModule = &battery;
}

double BatterySensor::readVoltageV() const {
  return (batteryModule != nullptr) ? batteryModule->voltage() : 0.0;
}

double BatterySensor::readSocRatio() const {
  return (batteryModule != nullptr) ? batteryModule->soc() : 0.0;
}

double BatterySensor::readSocPercent() const {
  return readSocRatio() * 100.0;
}

