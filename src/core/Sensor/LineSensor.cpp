#include "core/Sensor/LineSensor.h"

#include <algorithm>

namespace {
constexpr double kSensorSizeMm = 2.0;
constexpr double kSensorHalfMm = kSensorSizeMm * 0.5;
constexpr double kPitchMm = 5.0;
constexpr double kLineHalfWidthMm = 9.5;
}

LineSensor::LineSensor() {
  const double centerIndex = (static_cast<double>(sensorXMm.size()) - 1.0) * 0.5;
  for (size_t i = 0; i < sensorXMm.size(); ++i) {
    sensorXMm[i] = (static_cast<double>(i) - centerIndex) * kPitchMm;
  }
}

LineSensorReading LineSensor::read(double offsetMm) const {
  LineSensorReading out{};
  out.sensorXMm = sensorXMm;

  const double lineLeft = offsetMm - kLineHalfWidthMm;
  const double lineRight = offsetMm + kLineHalfWidthMm;

  double sumW = 0.0;
  double sumXW = 0.0;
  for (size_t i = 0; i < sensorXMm.size(); ++i) {
    const double sensorLeft = sensorXMm[i] - kSensorHalfMm;
    const double sensorRight = sensorXMm[i] + kSensorHalfMm;
    const double overlapMm = overlapLength(sensorLeft, sensorRight, lineLeft, lineRight);
    const double value = std::clamp(overlapMm / kSensorSizeMm, 0.0, 1.0);
    out.values[i] = value;
    sumW += value;
    sumXW += sensorXMm[i] * value;
  }

  if (sumW > 1e-12) {
    out.detected = true;
    out.xHatMm = sumXW / sumW;
  }
  return out;
}

std::string LineSensor::renderLineAscii(double offsetMm) const {
  const auto reading = read(offsetMm);
  std::string out(reading.values.size(), '*');
  for (size_t i = 0; i < reading.values.size(); ++i) {
    if (reading.values[i] > 0.0) {
      out[i] = '#';
    }
  }
  return out;
}

double LineSensor::overlapLength(double l0, double r0, double l1, double r1) {
  return std::max(0.0, std::min(r0, r1) - std::max(l0, l1));
}
