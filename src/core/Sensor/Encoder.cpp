#include "core/Sensor/Encoder.h"

Encoder::Encoder(const VehicleState& state) : vehicleState(&state) {}

void Encoder::connect(const VehicleState& state) {
  vehicleState = &state;
}

double Encoder::readVelocityMmS() const {
  return (vehicleState != nullptr) ? vehicleState->vMmS : 0.0;
}

double Encoder::readOmegaRadS() const {
  return (vehicleState != nullptr) ? vehicleState->omegaRadS : 0.0;
}

