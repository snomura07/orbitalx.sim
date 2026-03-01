#pragma once

class Battery;

class BatterySensor {
 public:
  explicit BatterySensor(const Battery& battery);

  void connect(const Battery& battery);
  double readVoltageV() const;
  double readSocRatio() const;
  double readSocPercent() const;

 private:
  const Battery* batteryModule{nullptr};
};

