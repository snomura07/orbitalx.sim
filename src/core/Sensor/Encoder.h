#pragma once

#include "core/common/Types.h"

class Encoder {
 public:
  explicit Encoder(const VehicleState& state);

  void connect(const VehicleState& state);
  double readVelocityMmS() const;
  double readOmegaRadS() const;

 private:
  const VehicleState* vehicleState{nullptr};
};

