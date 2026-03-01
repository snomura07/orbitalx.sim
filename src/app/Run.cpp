#include "app/Run.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

namespace {
constexpr double kPi = 3.14159265358979323846;
volatile std::sig_atomic_t gKeepRunning = 1;

void onSigint(int) {
  gKeepRunning = 0;
}

std::string formatArray15(const std::array<double, 15>& values) {
  std::ostringstream oss;
  oss << '[';
  for (size_t i = 0; i < values.size(); ++i) {
    if (i != 0) {
      oss << ", ";
    }
    oss << std::fixed << std::setprecision(2) << values[i];
  }
  oss << ']';
  return oss.str();
}

struct Point2 {
  double x{0.0};
  double y{0.0};
};

Point2 sensorPointFromPose(const Pose& pose, double sensorBaseMm) {
  return Point2{
      pose.xMm + sensorBaseMm * std::cos(pose.thetaRad),
      pose.yMm + sensorBaseMm * std::sin(pose.thetaRad),
  };
}

struct CourseCandidate {
  Point2 point{};
  Point2 tangent{};
};

double wrapAngleRad(double a) {
  double x = a;
  while (x > kPi) {
    x -= 2.0 * kPi;
  }
  while (x < -kPi) {
    x += 2.0 * kPi;
  }
  return x;
}

std::vector<int> classifyOdometrySegmentTypes(const std::vector<Pose>& trace, double stepMm) {
  const size_t n = trace.size();
  if (n < 2) {
    return {};
  }
  if (n < 3) {
    return std::vector<int>(n - 1, 0);
  }

  const double ds = std::max(stepMm, 1e-6);
  std::vector<double> thetaSeg;
  thetaSeg.reserve(n - 1);
  for (size_t i = 0; i + 1 < n; ++i) {
    const double dx = trace[i + 1].xMm - trace[i].xMm;
    const double dy = trace[i + 1].yMm - trace[i].yMm;
    thetaSeg.push_back(std::atan2(dy, dx));
  }

  std::vector<double> kappaPoint(n, 0.0);
  for (size_t i = 1; i + 1 < n; ++i) {
    const double dTheta = wrapAngleRad(thetaSeg[i] - thetaSeg[i - 1]);
    kappaPoint[i] = dTheta / ds;
  }
  kappaPoint[0] = kappaPoint[1];
  kappaPoint[n - 1] = kappaPoint[n - 2];

  std::vector<double> kappaSmooth(n, 0.0);
  constexpr int kRadius = 2;
  for (size_t i = 0; i < n; ++i) {
    double sum = 0.0;
    int count = 0;
    for (int k = -kRadius; k <= kRadius; ++k) {
      const int j = static_cast<int>(i) + k;
      if (j < 0 || j >= static_cast<int>(n)) {
        continue;
      }
      sum += kappaPoint[static_cast<size_t>(j)];
      ++count;
    }
    kappaSmooth[i] = (count > 0) ? (sum / static_cast<double>(count)) : 0.0;
  }

  constexpr double kStraight = 0.0008;
  constexpr double kCurve = 0.0016;
  std::vector<int> statePoint(n, 0);
  int state = 0;
  for (size_t i = 0; i < n; ++i) {
    const double ka = std::abs(kappaSmooth[i]);
    if (state == 0) {
      if (ka >= kCurve) {
        state = 1;
      }
    } else if (ka <= kStraight) {
      state = 0;
    }
    statePoint[i] = state;
  }

  std::vector<int> stateSeg(n - 1, 0);
  for (size_t i = 0; i + 1 < n; ++i) {
    stateSeg[i] = (statePoint[i] != 0 || statePoint[i + 1] != 0) ? 1 : 0;
  }
  return stateSeg;
}

double dist2(const Point2& a, const Point2& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return dx * dx + dy * dy;
}

CourseCandidate nearestOnSegment(const Point2& p, const Point2& a, const Point2& b) {
  const double abx = b.x - a.x;
  const double aby = b.y - a.y;
  const double ab2 = abx * abx + aby * aby;
  if (ab2 <= 1e-12) {
    return CourseCandidate{a, Point2{1.0, 0.0}};
  }
  const double apx = p.x - a.x;
  const double apy = p.y - a.y;
  const double t = std::clamp((apx * abx + apy * aby) / ab2, 0.0, 1.0);
  const double len = std::hypot(abx, aby);
  const double tx = (len > 1e-12) ? (abx / len) : 1.0;
  const double ty = (len > 1e-12) ? (aby / len) : 0.0;
  return CourseCandidate{
      Point2{a.x + t * abx, a.y + t * aby},
      Point2{tx, ty},
  };
}

CourseCandidate nearestOnRightArc(const Point2& p, double halfStraightMm, double radiusMm) {
  const Point2 center{halfStraightMm, radiusMm};
  const double vx = p.x - center.x;
  const double vy = p.y - center.y;
  const double len = std::hypot(vx, vy);
  Point2 q{};
  if (len <= 1e-12) {
    q = Point2{halfStraightMm + radiusMm, radiusMm};
  } else {
    q = Point2{center.x + radiusMm * (vx / len), center.y + radiusMm * (vy / len)};
  }
  if (q.x >= halfStraightMm) {
    const double nx = (q.x - center.x) / radiusMm;
    const double ny = (q.y - center.y) / radiusMm;
    return CourseCandidate{q, Point2{-ny, nx}};
  }
  const Point2 bottom{halfStraightMm, 0.0};
  const Point2 top{halfStraightMm, 2.0 * radiusMm};
  if (dist2(p, bottom) <= dist2(p, top)) {
    return CourseCandidate{bottom, Point2{1.0, 0.0}};
  }
  return CourseCandidate{top, Point2{-1.0, 0.0}};
}

CourseCandidate nearestOnLeftArc(const Point2& p, double halfStraightMm, double radiusMm) {
  const Point2 center{-halfStraightMm, radiusMm};
  const double vx = p.x - center.x;
  const double vy = p.y - center.y;
  const double len = std::hypot(vx, vy);
  Point2 q{};
  if (len <= 1e-12) {
    q = Point2{-halfStraightMm - radiusMm, radiusMm};
  } else {
    q = Point2{center.x + radiusMm * (vx / len), center.y + radiusMm * (vy / len)};
  }
  if (q.x <= -halfStraightMm) {
    const double nx = (q.x - center.x) / radiusMm;
    const double ny = (q.y - center.y) / radiusMm;
    return CourseCandidate{q, Point2{-ny, nx}};
  }
  const Point2 bottom{-halfStraightMm, 0.0};
  const Point2 top{-halfStraightMm, 2.0 * radiusMm};
  if (dist2(p, bottom) <= dist2(p, top)) {
    return CourseCandidate{bottom, Point2{1.0, 0.0}};
  }
  return CourseCandidate{top, Point2{-1.0, 0.0}};
}
}  // namespace

Run::Run(const SimParams& paramsIn, const RunOptions& optionsIn)
    : params(paramsIn),
      options(optionsIn),
      battery(params),
      state(),
      encoder(state),
      batterySensor(battery),
      lineSensor(),
      motorL(params),
      motorR(params),
      cpu(params, encoder, batterySensor),
      wsClient(params.wsHost, params.wsPort, params.wsPath),
      dtS(std::clamp(params.controlCycleS, 0.001, 0.05)) {
  state.vbatV = batterySensor.readVoltageV();
}

int Run::run() {
  constexpr double kOutputPeriodS = 0.01;
  gKeepRunning = 1;
  lapCount = 0;
  hasPrevCourseProgress = false;
  prevCourseProgressMm = 0.0;
  totalDistanceMm = 0.0;
  odometryTracePoints.clear();
  nextOdometrySampleDistanceMm = 0.0;
  if (options.odometryTraceMode) {
    odometryTracePoints.push_back(state.pose);
    nextOdometrySampleDistanceMm = 10.0;
  }
  std::signal(SIGINT, onSigint);
  std::cerr << "loaded params: desired_velocity_mm_s=" << params.desiredVelocityMmS
            << ", control_cycle_s=" << dtS
            << ", odometry_trace_mode=" << (options.odometryTraceMode ? "on" : "off") << '\n';

  auto nextTick = std::chrono::steady_clock::now();
  double outputAccumS = 0.0;
  while (gKeepRunning) {
    updatePlant();
    updateBattery();
    updateSensors();
    updateLapCounter();
    if (options.odometryTraceMode && lapCount > 1) {
      gKeepRunning = 0;
    }

    const double measuredLinePositionMm = lineReading.detected ? lineReading.xHatMm : 0.0;
    driveCommand = cpu.updateDriveCommand(
        params.desiredVelocityMmS, measuredLinePositionMm, lineReading.detected, dtS);
    applyDriveCommand(driveCommand);
    state.lateralErrorMm = cpu.lastLineErrorMm();
    state.velocityErrorMmS = cpu.lastVelocityErrorMmS();

    outputAccumS += dtS;
    if (outputAccumS >= kOutputPeriodS) {
      publishOdometry();
      renderConsole();
      outputAccumS = std::fmod(outputAccumS, kOutputPeriodS);
    }

    elapsedS += dtS;
    nextTick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(dtS));
    std::this_thread::sleep_until(nextTick);
  }

  publishOdometry();
  wsClient.close();
  std::cout << "\nSimulation stopped.\n";
  return 0;
}

void Run::applyDriveCommand(const DriveCommand& command) {
  state.dutyL = command.dutyL;
  state.dutyR = command.dutyR;
}

void Run::updatePlant() {
  const double vLMps = state.vLMmS / 1000.0;
  const double vRMps = state.vRMmS / 1000.0;
  const double vbatV = batterySensor.readVoltageV();

  motorLeftResult = motorL.evaluate(state.dutyL, vbatV, vLMps);
  motorRightResult = motorR.evaluate(state.dutyR, vbatV, vRMps);

  const auto wheelResistance = [&](double wheelVelMps) {
    if (std::abs(wheelVelMps) <= 1e-6) {
      return 0.0;
    }
    const double sign = (wheelVelMps >= 0.0) ? 1.0 : -1.0;
    const double absV = std::abs(wheelVelMps);
    return sign * (0.5 * params.resistF0N +
                   params.resistKvNPerMps * absV +
                   params.resistK2NPerMps2 * absV * absV);
  };
  const double massKg = params.massG / 1000.0;
  const double wheelMassKg = std::max(massKg * 0.5, 1e-6);
  const double accelLMps2 = (motorLeftResult.driveForceN - wheelResistance(vLMps)) / wheelMassKg;
  const double accelRMps2 = (motorRightResult.driveForceN - wheelResistance(vRMps)) / wheelMassKg;
  const double velocityMaxMps = params.vehicleMaxVelocityMmS / 1000.0;
  const double nextVLMps = std::clamp(vLMps + accelLMps2 * dtS, -velocityMaxMps, velocityMaxMps);
  const double nextVRMps = std::clamp(vRMps + accelRMps2 * dtS, -velocityMaxMps, velocityMaxMps);

  state.vLMmS = nextVLMps * 1000.0;
  state.vRMmS = nextVRMps * 1000.0;
  state.vMmS = (nextVLMps + nextVRMps) * 500.0;
  totalDistanceMm += std::abs(state.vMmS) * dtS;

  const double trackM = params.wheelTreadMm / 1000.0;
  state.omegaRadS = (trackM > 1e-9) ? ((nextVRMps - nextVLMps) / trackM) : 0.0;

  state.pose.xMm += state.vMmS * std::cos(state.pose.thetaRad) * dtS;
  state.pose.yMm += state.vMmS * std::sin(state.pose.thetaRad) * dtS;
  state.pose.thetaRad += state.omegaRadS * dtS;
  maybeRecordOdometryTracePose();
}

void Run::updateBattery() {
  const double totalCurrentA =
      std::abs(motorLeftResult.currentA) + std::abs(motorRightResult.currentA) + params.batteryIMcuA;
  battery.update(totalCurrentA, dtS);

  state.vbatV = batterySensor.readVoltageV();
  state.vmotLV = state.dutyL * state.vbatV;
  state.vmotRV = state.dutyR * state.vbatV;
  state.iLA = motorLeftResult.currentA;
  state.iRA = motorRightResult.currentA;
}

void Run::updateSensors() {
  const double lineWindowMm = std::max(params.lineSensorLongitudinalWindowMm, 1.0);
  const CourseRelativePose rel = computeLineRelativePose();
  if (std::abs(rel.longitudinalMm) > lineWindowMm) {
    lineReading = lineSensor.read(1e6);
    lineAscii = lineSensor.renderLineAscii(1e6);
    return;
  }

  const double lineOffsetMm = rel.lateralMm + params.whiteLineOffsetMm;
  lineReading = lineSensor.read(lineOffsetMm);
  lineAscii = lineSensor.renderLineAscii(lineOffsetMm);
  if (lineReading.detected) {
    state.linePositionMm = lineReading.xHatMm;
  }
}

void Run::updateLapCounter() {
  const double straightLengthMm = std::max(params.courseStraightLengthMm, 1.0);
  const double radiusMm = std::max(params.courseCurveRadiusMm, 1.0);
  const double lapLengthMm = 2.0 * straightLengthMm + 2.0 * kPi * radiusMm;
  if (lapLengthMm <= 1e-9) {
    return;
  }

  const double progressMm = computeCourseProgressMm();
  if (!hasPrevCourseProgress) {
    prevCourseProgressMm = progressMm;
    hasPrevCourseProgress = true;
    return;
  }

  const double deltaMm = progressMm - prevCourseProgressMm;
  if (deltaMm < -0.5 * lapLengthMm) {
    ++lapCount;
  } else if (deltaMm > 0.5 * lapLengthMm && lapCount > 0) {
    --lapCount;
  }

  prevCourseProgressMm = progressMm;
}

Run::CourseRelativePose Run::computeLineRelativePose() const {
  const double straightLengthMm = std::max(params.courseStraightLengthMm, 1.0);
  const double radiusMm = std::max(params.courseCurveRadiusMm, 1.0);
  const double halfStraightMm = straightLengthMm * 0.5;

  const Point2 p = sensorPointFromPose(state.pose, params.sensorBaseMm);
  const Point2 lowerA{-halfStraightMm, 0.0};
  const Point2 lowerB{halfStraightMm, 0.0};
  const Point2 upperA{halfStraightMm, 2.0 * radiusMm};
  const Point2 upperB{-halfStraightMm, 2.0 * radiusMm};

  const std::array<CourseCandidate, 4> candidates{
      nearestOnSegment(p, lowerA, lowerB),
      nearestOnSegment(p, upperA, upperB),
      nearestOnRightArc(p, halfStraightMm, radiusMm),
      nearestOnLeftArc(p, halfStraightMm, radiusMm),
  };

  const double fwdX = std::cos(state.pose.thetaRad);
  const double fwdY = std::sin(state.pose.thetaRad);
  CourseCandidate nearest = candidates[0];
  bool hasForwardCompatible = false;
  double bestScore = 1e30;

  for (size_t i = 0; i < candidates.size(); ++i) {
    const auto& c = candidates[i];
    const double d2 = dist2(p, c.point);
    const double headingAlign = c.tangent.x * fwdX + c.tangent.y * fwdY;
    const bool forwardCompatible = (headingAlign >= -0.15);
    if (!forwardCompatible && hasForwardCompatible) {
      continue;
    }
    const double headingPenalty = (headingAlign >= 0.0) ? 0.0 : (1.0 - headingAlign) * radiusMm;
    const double score = d2 + headingPenalty * headingPenalty;
    if (!hasForwardCompatible || forwardCompatible) {
      if (!hasForwardCompatible || score < bestScore) {
        nearest = c;
        bestScore = score;
      }
      if (forwardCompatible) {
        hasForwardCompatible = true;
      }
    }
  }

  const double dx = nearest.point.x - p.x;
  const double dy = nearest.point.y - p.y;
  const double rightX = std::sin(state.pose.thetaRad);
  const double rightY = -std::cos(state.pose.thetaRad);
  // Sensor X-axis treats right side as positive, so project onto vehicle-right.
  return CourseRelativePose{
      dx * rightX + dy * rightY,
      dx * fwdX + dy * fwdY,
  };
}

double Run::computeCourseProgressMm() const {
  const double straightLengthMm = std::max(params.courseStraightLengthMm, 1.0);
  const double radiusMm = std::max(params.courseCurveRadiusMm, 1.0);
  const double halfStraightMm = straightLengthMm * 0.5;
  const double lapLengthMm = 2.0 * straightLengthMm + 2.0 * kPi * radiusMm;

  const Point2 p{state.pose.xMm, state.pose.yMm};
  const Point2 lowerA{-halfStraightMm, 0.0};
  const Point2 lowerB{halfStraightMm, 0.0};
  const Point2 upperA{halfStraightMm, 2.0 * radiusMm};
  const Point2 upperB{-halfStraightMm, 2.0 * radiusMm};

  const std::array<CourseCandidate, 4> candidates{
      nearestOnSegment(p, lowerA, lowerB),
      nearestOnSegment(p, upperA, upperB),
      nearestOnRightArc(p, halfStraightMm, radiusMm),
      nearestOnLeftArc(p, halfStraightMm, radiusMm),
  };

  const double fwdX = std::cos(state.pose.thetaRad);
  const double fwdY = std::sin(state.pose.thetaRad);
  CourseCandidate nearest = candidates[0];
  bool hasForwardCompatible = false;
  double bestScore = 1e30;

  for (size_t i = 0; i < candidates.size(); ++i) {
    const auto& c = candidates[i];
    const double d2 = dist2(p, c.point);
    const double headingAlign = c.tangent.x * fwdX + c.tangent.y * fwdY;
    const bool forwardCompatible = (headingAlign >= -0.15);
    if (!forwardCompatible && hasForwardCompatible) {
      continue;
    }
    const double headingPenalty = (headingAlign >= 0.0) ? 0.0 : (1.0 - headingAlign) * radiusMm;
    const double score = d2 + headingPenalty * headingPenalty;
    if (!hasForwardCompatible || forwardCompatible) {
      if (!hasForwardCompatible || score < bestScore) {
        nearest = c;
        bestScore = score;
      }
      if (forwardCompatible) {
        hasForwardCompatible = true;
      }
    }
  }

  double sMm = 0.0;
  const double qx = nearest.point.x;
  const double qy = nearest.point.y;
  if (std::abs(qy) < 1e-6) {
    sMm = (qx >= 0.0) ? qx : (lapLengthMm + qx);
  } else if (std::abs(qy - 2.0 * radiusMm) < 1e-6) {
    sMm = halfStraightMm + kPi * radiusMm + (halfStraightMm - qx);
  } else if (qx >= halfStraightMm) {
    const double ang = std::atan2(qy - radiusMm, qx - halfStraightMm);
    sMm = halfStraightMm + radiusMm * (ang + kPi * 0.5);
  } else {
    double ang = std::atan2(qy - radiusMm, qx + halfStraightMm);
    if (ang < kPi * 0.5) {
      ang += 2.0 * kPi;
    }
    sMm = halfStraightMm + kPi * radiusMm + straightLengthMm + radiusMm * (ang - kPi * 0.5);
  }

  sMm = std::fmod(sMm, lapLengthMm);
  if (sMm < 0.0) {
    sMm += lapLengthMm;
  }
  return sMm;
}

void Run::maybeRecordOdometryTracePose() {
  if (!options.odometryTraceMode) {
    return;
  }
  while (totalDistanceMm >= nextOdometrySampleDistanceMm) {
    odometryTracePoints.push_back(state.pose);
    nextOdometrySampleDistanceMm += 10.0;
  }
}

void Run::publishOdometry() {
  if (!params.wsEnabled) {
    return;
  }

  if (!wsClient.isConnected()) {
    if (reconnectCounter <= 0) {
      wsClient.connect();
      reconnectCounter = 100;
    } else {
      --reconnectCounter;
    }
  }
  if (!wsClient.isConnected()) {
    return;
  }

  std::ostringstream msg;
  msg << '{';
  msg << "\"ts\":" << elapsedS << ',';
  msg << "\"velocity\":" << state.vMmS << ',';
  msg << "\"omega\":" << state.omegaRadS << ',';
  msg << "\"desiredVelocity\":" << cpu.desiredVelocityMmS() << ',';
  msg << "\"desiredVelocityInput\":" << params.desiredVelocityMmS << ',';
  msg << "\"batterySoc\":" << batterySensor.readSocPercent() << ',';
  msg << "\"batteryV\":" << state.vbatV << ',';
  msg << "\"lineDetected\":" << (lineReading.detected ? "true" : "false") << ',';
  msg << "\"xHat\":" << (lineReading.detected ? lineReading.xHatMm : 0.0) << ',';
  msg << "\"lineError\":" << state.lateralErrorMm << ',';
  msg << "\"velocityError\":" << state.velocityErrorMmS << ',';
  msg << "\"lineKp\":" << cpu.lineKp() << ',';
  msg << "\"lineKi\":" << cpu.lineKi() << ',';
  msg << "\"lineKd\":" << cpu.lineKd() << ',';
  msg << "\"omegaRef\":" << cpu.lastOmegaRefRadS() << ',';
  msg << "\"omegaError\":" << cpu.lastOmegaErrorRadS() << ',';
  msg << "\"basePwm\":" << cpu.lastBasePwm() << ',';
  msg << "\"steerPwm\":" << cpu.lastSteerPwm() << ',';
  msg << "\"lapCount\":" << lapCount << ',';
  msg << "\"totalDistanceMm\":" << totalDistanceMm << ',';
  msg << "\"lineValues\":[";
  for (size_t i = 0; i < lineReading.values.size(); ++i) {
    if (i != 0) {
      msg << ',';
    }
    msg << lineReading.values[i];
  }
  msg << "],";
  msg << "\"pose\":{";
  msg << "\"x\":" << state.pose.xMm << ',';
  msg << "\"y\":" << state.pose.yMm << ',';
  msg << "\"theta\":" << state.pose.thetaRad;
  msg << "},";
  const Point2 sensorPose = sensorPointFromPose(state.pose, params.sensorBaseMm);
  msg << "\"poseSensor\":{";
  msg << "\"x\":" << sensorPose.x << ',';
  msg << "\"y\":" << sensorPose.y;
  msg << "},";
  msg << "\"course\":{";
  msg << "\"straightLength\":" << params.courseStraightLengthMm << ',';
  msg << "\"curveRadius\":" << params.courseCurveRadiusMm;
  msg << "},";
  msg << "\"params\":{";
  msg << "\"vehicle_max_velocity_mm_s\":" << params.vehicleMaxVelocityMmS << ',';
  msg << "\"acceleration_mm_ss\":" << params.accelerationMmSS << ',';
  msg << "\"desired_velocity_mm_s\":" << params.desiredVelocityMmS << ',';
  msg << "\"control_cycle_s\":" << dtS << ',';
  msg << "\"wheel_tread_mm\":" << params.wheelTreadMm << ',';
  msg << "\"sensor_base_mm\":" << params.sensorBaseMm << ',';
  msg << "\"course_straight_length_mm\":" << params.courseStraightLengthMm << ',';
  msg << "\"course_curve_radius_mm\":" << params.courseCurveRadiusMm << ',';
  msg << "\"white_line_offset_mm\":" << params.whiteLineOffsetMm << ',';
  msg << "\"line_kp\":" << params.lineKp << ',';
  msg << "\"line_ki\":" << params.lineKi << ',';
  msg << "\"line_kd\":" << params.lineKd << ',';
  msg << "\"speed_kp\":" << params.speedKp << ',';
  msg << "\"speed_ki\":" << params.speedKi << ',';
  msg << "\"pwm_max\":" << params.pwmMax << ',';
  msg << "\"resist_F0_N\":" << params.resistF0N << ',';
  msg << "\"resist_kv_N_per_mps\":" << params.resistKvNPerMps << ',';
  msg << "\"resist_k2_N_per_mps2\":" << params.resistK2NPerMps2;
  msg << "},";
  msg << "\"odometryTraceMode\":" << (options.odometryTraceMode ? "true" : "false") << ',';
  msg << "\"odometryTraceStepMm\":10.0,";
  msg << "\"odometryTracePoints\":[";
  for (size_t i = 0; i < odometryTracePoints.size(); ++i) {
    if (i != 0) {
      msg << ',';
    }
    const Pose& p = odometryTracePoints[i];
    msg << "{\"x\":" << p.xMm << ",\"y\":" << p.yMm << ",\"theta\":" << p.thetaRad << '}';
  }
  msg << "],";
  const std::vector<int> odometrySegmentTypes =
      classifyOdometrySegmentTypes(odometryTracePoints, 10.0);
  msg << "\"odometrySegmentTypes\":[";
  for (size_t i = 0; i < odometrySegmentTypes.size(); ++i) {
    if (i != 0) {
      msg << ',';
    }
    msg << odometrySegmentTypes[i];
  }
  msg << "]}";
  if (!wsClient.sendText(msg.str())) {
    wsClient.close();
  }
}

void Run::renderConsole() const {
  std::cout << "\x1b[2J\x1b[H";
  std::cout << std::fixed << std::setprecision(3);
  std::cout << "Robot Trace Simulator (Ctrl+C to stop)\n";
  std::cout << "--------------------------------------\n";
  std::cout << "Time [s]             : " << elapsedS << '\n';
  std::cout << "Desired v [mm/s]     : " << cpu.desiredVelocityMmS() << '\n';
  std::cout << "Velocity [mm/s]      : " << state.vMmS << '\n';
  std::cout << "Omega [rad/s]        : " << state.omegaRadS << '\n';
  std::cout << "Battery Vbat [V]     : " << state.vbatV << '\n';
  std::cout << "Battery SoC [%]      : " << batterySensor.readSocPercent() << '\n';
  std::cout << "Motor V L/R [V]      : " << state.vmotLV << " / " << state.vmotRV << '\n';
  std::cout << "Motor I L/R [A]      : " << state.iLA << " / " << state.iRA << '\n';
  std::cout << "Pose x,y [mm]        : " << state.pose.xMm << ", " << state.pose.yMm << '\n';
  const Point2 sensorPose = sensorPointFromPose(state.pose, params.sensorBaseMm);
  std::cout << "Sensor x,y [mm]      : " << sensorPose.x << ", " << sensorPose.y << '\n';
  std::cout << "Pose theta [rad]     : " << state.pose.thetaRad << '\n';
  std::cout << "Wheel v L/R [mm/s]   : " << state.vLMmS << " / " << state.vRMmS << '\n';
  std::cout << "PWM Duty L/R         : " << state.dutyL << " / " << state.dutyR << '\n';
  std::cout << "Base/Steer PWM       : " << cpu.lastBasePwm() << " / " << cpu.lastSteerPwm() << '\n';
  std::cout << "Line Gain Kp/Ki/Kd   : " << cpu.lineKp() << " / " << cpu.lineKi() << " / " << cpu.lineKd() << '\n';
  std::cout << "Omega ref/err [rad/s]: " << cpu.lastOmegaRefRadS() << " / " << cpu.lastOmegaErrorRadS() << '\n';
  std::cout << "Line err [mm]        : " << state.lateralErrorMm << '\n';
  std::cout << "Speed err [mm/s]     : " << state.velocityErrorMmS << '\n';
  std::cout << "Lap Count            : " << lapCount << '\n';
  std::cout << "Total Dist [mm]      : " << totalDistanceMm << '\n';
  std::cout << "Line Pattern         : " << lineAscii << '\n';
  std::cout << "Line Sensor[15]      : " << formatArray15(lineReading.values) << '\n';
  if (lineReading.detected) {
    std::cout << "Estimated x_hat [mm] : " << lineReading.xHatMm << '\n';
  } else {
    std::cout << "Estimated x_hat [mm] : UNDETECTED\n";
  }
  std::cout.flush();
}
