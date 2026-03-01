#pragma once

#include <array>
#include <string>

struct LineSensorReading {
  std::array<double, 15> sensorXMm{};
  std::array<double, 15> values{};
  bool detected{false};
  double xHatMm{0.0};
};

class LineSensor {
 public:
  LineSensor();

  LineSensorReading read(double offsetMm) const;
  std::string renderLineAscii(double offsetMm) const;

 private:
  static double overlapLength(double l0, double r0, double l1, double r1);

  std::array<double, 15> sensorXMm{};
};

