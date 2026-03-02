#pragma once

#include "core/common/Types.h"
#include "core/course/Course.h"
#include "core/cpu/Cpu.h"
#include "core/Sensor/BatterySensor.h"
#include "core/Sensor/Encoder.h"
#include "core/Sensor/LineSensor.h"
#include "core/io/WsClient.h"
#include "core/plant/battery/Battery.h"
#include "core/plant/motor/Motor.h"

#include <string>
#include <vector>

struct RunOptions {
  bool odometryTraceMode{false};
};

class Run {
 public:
  explicit Run(const SimParams& params, const RunOptions& options = {});

  int run();

 private:
  void applyDriveCommand(const DriveCommand& command);
  void updatePlant();
  void updateBattery();
  void updateSensors();
  void updateLapCounter();
  struct CourseRelativePose {
    double lateralMm{0.0};
    double longitudinalMm{0.0};
  };
  CourseRelativePose computeLineRelativePose() const;
  double computeCourseProgressMm() const;
  double sampleSpeedPlanVelocityMmS(double distanceMm) const;
  void maybeRecordOdometryTracePose();
  void maybeBuildSpeedPlanFromFirstLap();
  void publishOdometry();
  void renderConsole() const;

  SimParams params;
  RunOptions options;
  Course course;
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
  bool lineDetectedStable{false};
  int lineDetectHitCount{0};
  int lineDetectMissCount{0};

  int reconnectCounter{0};
  int lapCount{1};
  bool hasPrevCourseProgress{false};
  double prevCourseProgressMm{0.0};
  double courseProgressAccumulatedMm{0.0};
  double totalDistanceMm{0.0};
  std::vector<Pose> odometryTracePoints;
  double nextOdometrySampleDistanceMm{0.0};
  bool speedPlanReady{false};
  std::vector<double> speedPlanDistanceMm;
  std::vector<double> speedPlanVelocityMmS;
  std::vector<int> speedPlanSegmentTypes;
  double desiredVelocityInputMmS{0.0};
  double elapsedS{0.0};
  double dtS{0.01};
};
