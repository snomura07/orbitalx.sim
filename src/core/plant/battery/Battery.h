#pragma once

#include "core/common/Types.h"

class Battery {
 public:
  explicit Battery(const SimParams& params);

  void update(double totalCurrentA, double dtS);
  double voltage() const;
  double soc() const;

 private:
  double ocv(double soc) const;

  SimParams params;
  double socValue{1.0};
  double vbatV{8.4};
};
