#pragma once

#include <string>

struct Pose {
  double xMm{0.0};
  double yMm{0.0};
  double thetaRad{0.0};
};

struct VehicleState {
  Pose pose{};
  double vMmS{0.0};
  double omegaRadS{0.0};
  double vLMmS{0.0};
  double vRMmS{0.0};
  double dutyL{0.0};
  double dutyR{0.0};
  double vbatV{0.0};
  double vmotLV{0.0};
  double vmotRV{0.0};
  double iLA{0.0};
  double iRA{0.0};
  double lateralErrorMm{0.0};
  double linePositionMm{0.0};
  double velocityErrorMmS{0.0};
};

struct SimParams {
  double vehicleMaxVelocityMmS{4000.0};
  double accelerationMmSS{3000.0};
  double batteryVInit{8.3};
  double tireDiameterMm{24.0};
  double gearMotor{20.0};
  double gearWheel{64.0};
  double massG{125.0};
  double wheelTreadMm{98.0};
  double desiredVelocityMmS{1000.0};
  double simDurationS{10.0};
  double controlCycleS{0.01};
  double whiteLineOffsetMm{0.0};
  double courseStraightLengthMm{5000.0};
  double courseCurveRadiusMm{300.0};
  double lineSensorLongitudinalWindowMm{60.0};

  double pwmMax{1500.0};
  double motorROhm{2.83};
  double motorKtNmA{0.00533};
  double motorKnRpmV{1790.0};
  double motorI0A{0.0226};

  double batteryCapacityAh{0.35};
  double batteryVFull{8.4};
  double batteryVEmpty{6.4};
  double batteryVMin{6.0};
  double batteryRInternalOhm{0.20};
  double batteryIMcuA{0.05};

  double etaGear{0.80};
  double resistF0N{0.10};
  double resistKvNPerMps{0.20};
  double resistK2NPerMps2{0.0};

  double speedKp{0.45};
  double speedKi{0.10};
  double lineKp{2.00};
  double lineKi{0.00};
  double lineKd{0.40};
  double lineIntegralLimit{500.0};
  double lineSteerPwmLimitRatio{0.35};

  bool wsEnabled{true};
  std::string wsHost{"web"};
  int wsPort{8080};
  std::string wsPath{"/ingest"};
};
