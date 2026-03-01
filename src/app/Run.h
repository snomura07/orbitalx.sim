#pragma once

#include "core/common/Types.h"
#include "core/cpu/Cpu.h"
#include "core/Sensor/BatterySensor.h"
#include "core/Sensor/Encoder.h"
#include "core/Sensor/LineSensor.h"
#include "core/io/WsClient.h"
#include "core/plant/battery/Battery.h"
#include "core/plant/motor/Motor.h"

#include <string>

class Run {
 public:
  explicit Run(const SimParams& params);

  int run();

 private:
  void applyDriveCommand(const DriveCommand& command);
  void updatePlant();
  void updateBattery();
  void updateSensors();
  void publishTelemetry();
  void renderConsole() const;

  SimParams params;
  Battery battery;
  VehicleState state;
  Encoder encoder;
  BatterySensor batterySensor;
  LineSensor lineSensor;
  Motor motorL;
  Motor motorR;
  Cpu cpu;
  WsClient wsClient;
  DriveCommand driveCommand;

  MotorStepResult motorLeftResult;
  MotorStepResult motorRightResult;
  LineSensorReading lineReading;
  std::string lineAscii;

  int reconnectCounter{0};
  double elapsedS{0.0};
};
