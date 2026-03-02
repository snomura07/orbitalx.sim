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

CoursePoint sensorPointFromPose(const Pose& pose, double sensorBaseMm) {
  return CoursePoint{
      pose.xMm + sensorBaseMm * std::cos(pose.thetaRad),
      pose.yMm + sensorBaseMm * std::sin(pose.thetaRad),
  };
}

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

Pose lerpPose(const Pose& a, const Pose& b, double t) {
  return Pose{
      a.xMm + (b.xMm - a.xMm) * t,
      a.yMm + (b.yMm - a.yMm) * t,
      a.thetaRad + wrapAngleRad(b.thetaRad - a.thetaRad) * t,
  };
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

}  // namespace

Run::Run(const SimParams& paramsIn, const RunOptions& optionsIn)
    : params(paramsIn),
      options(optionsIn),
      course(Course::fromParams(params)),
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
  lapCount = 1;
  hasPrevCourseProgress = false;
  prevCourseProgressMm = 0.0;
  courseProgressAccumulatedMm = 0.0;
  totalDistanceMm = 0.0;
  odometryTracePoints.clear();
  nextOdometrySampleDistanceMm = 0.0;
  speedPlanReady = false;
  speedPlanDistanceMm.clear();
  speedPlanVelocityMmS.clear();
  speedPlanSegmentTypes.clear();
  lineDetectedStable = false;
  lineDetectHitCount = 0;
  lineDetectMissCount = 0;
  desiredVelocityInputMmS = params.desiredVelocityMmS;
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
    maybeBuildSpeedPlanFromFirstLap();
    if (options.odometryTraceMode && lapCount >= 3) {
      break;
    }

    desiredVelocityInputMmS = params.desiredVelocityMmS;
    if (options.odometryTraceMode && speedPlanReady && lapCount >= 2) {
      const double courseProgressMm = computeCourseProgressMm();
      desiredVelocityInputMmS = sampleSpeedPlanVelocityMmS(courseProgressMm);
    }
    const double measuredLinePositionMm = lineReading.detected ? lineReading.xHatMm : 0.0;
    driveCommand = cpu.updateDriveCommand(
        desiredVelocityInputMmS, measuredLinePositionMm, lineReading.detected, dtS);
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
  constexpr int kDetectOnCount = 2;
  constexpr int kDetectOffCount = 6;
  auto applyDetectDebounce = [&](LineSensorReading& reading) {
    if (reading.detected) {
      lineDetectHitCount = std::min(lineDetectHitCount + 1, kDetectOffCount);
      lineDetectMissCount = 0;
      if (!lineDetectedStable && lineDetectHitCount >= kDetectOnCount) {
        lineDetectedStable = true;
      }
    } else {
      lineDetectMissCount = std::min(lineDetectMissCount + 1, kDetectOffCount);
      lineDetectHitCount = 0;
      if (lineDetectedStable && lineDetectMissCount >= kDetectOffCount) {
        lineDetectedStable = false;
      }
    }
    reading.detected = lineDetectedStable;
    if (!reading.detected) {
      reading.xHatMm = state.linePositionMm;
    } else {
      state.linePositionMm = reading.xHatMm;
    }
  };

  const CourseRelativePose rel = computeLineRelativePose();
  const double lineOffsetMm = rel.lateralMm + params.whiteLineOffsetMm;
  lineReading = lineSensor.read(lineOffsetMm);
  lineAscii = lineSensor.renderLineAscii(lineOffsetMm);
  applyDetectDebounce(lineReading);
}

void Run::updateLapCounter() {
  const double lapLengthMm = std::max(course.lengthMm(), 1.0);
  if (lapLengthMm <= 1e-9) {
    return;
  }

  const double progressMm = computeCourseProgressMm();
  if (!hasPrevCourseProgress) {
    prevCourseProgressMm = progressMm;
    courseProgressAccumulatedMm = 0.0;
    lapCount = 1;
    hasPrevCourseProgress = true;
    return;
  }

  double deltaMm = progressMm - prevCourseProgressMm;
  if (deltaMm < -0.5 * lapLengthMm) {
    deltaMm += lapLengthMm;
  } else if (deltaMm > 0.5 * lapLengthMm) {
    deltaMm -= lapLengthMm;
  }
  const double maxForwardStepMm = std::max(params.vehicleMaxVelocityMmS * dtS * 4.0, 20.0);
  deltaMm = std::clamp(deltaMm, -maxForwardStepMm, maxForwardStepMm);
  // Keep lap progression monotonic against projection jitter.
  if (deltaMm > 0.0) {
    courseProgressAccumulatedMm += deltaMm;
  }
  lapCount = 1 + static_cast<int>(std::floor(courseProgressAccumulatedMm / lapLengthMm));

  prevCourseProgressMm = progressMm;
}

Run::CourseRelativePose Run::computeLineRelativePose() const {
  const CoursePoint p = sensorPointFromPose(state.pose, params.sensorBaseMm);
  const double fwdX = std::cos(state.pose.thetaRad);
  const double fwdY = std::sin(state.pose.thetaRad);
  const double sHintMm = hasPrevCourseProgress ? prevCourseProgressMm : -1.0;
  const Course::Projection nearest =
      course.project(p, CoursePoint{fwdX, fwdY}, sHintMm, 1.0);

  const double dx = nearest.point.xMm - p.xMm;
  const double dy = nearest.point.yMm - p.yMm;
  const double rightX = std::sin(state.pose.thetaRad);
  const double rightY = -std::cos(state.pose.thetaRad);
  // Sensor X-axis treats right side as positive, so project onto vehicle-right.
  return CourseRelativePose{
      dx * rightX + dy * rightY,
      dx * fwdX + dy * fwdY,
  };
}

double Run::computeCourseProgressMm() const {
  const double fwdX = std::cos(state.pose.thetaRad);
  const double fwdY = std::sin(state.pose.thetaRad);
  const double sHintMm = hasPrevCourseProgress ? prevCourseProgressMm : -1.0;
  const Course::Projection projection =
      course.project(CoursePoint{state.pose.xMm, state.pose.yMm}, CoursePoint{fwdX, fwdY}, sHintMm, 1.0);
  return projection.sMm;
}

double Run::sampleSpeedPlanVelocityMmS(double distanceMm) const {
  if (!speedPlanReady || speedPlanDistanceMm.empty() || speedPlanVelocityMmS.empty()) {
    return params.desiredVelocityMmS;
  }
  if (speedPlanDistanceMm.size() != speedPlanVelocityMmS.size()) {
    return params.desiredVelocityMmS;
  }

  const double maxDistanceMm = speedPlanDistanceMm.back();
  if (maxDistanceMm <= 1e-9) {
    return speedPlanVelocityMmS.front();
  }
  double sMm = std::fmod(distanceMm, maxDistanceMm);
  if (sMm < 0.0) {
    sMm += maxDistanceMm;
  }

  const auto it = std::lower_bound(speedPlanDistanceMm.begin(), speedPlanDistanceMm.end(), sMm);
  if (it == speedPlanDistanceMm.begin()) {
    return speedPlanVelocityMmS.front();
  }
  if (it == speedPlanDistanceMm.end()) {
    return speedPlanVelocityMmS.back();
  }
  const size_t i1 = static_cast<size_t>(std::distance(speedPlanDistanceMm.begin(), it));
  const size_t i0 = i1 - 1;
  const double s0 = speedPlanDistanceMm[i0];
  const double s1 = speedPlanDistanceMm[i1];
  const double v0 = speedPlanVelocityMmS[i0];
  const double v1 = speedPlanVelocityMmS[i1];
  const double denom = std::max(s1 - s0, 1e-9);
  const double t = std::clamp((sMm - s0) / denom, 0.0, 1.0);
  return v0 + (v1 - v0) * t;
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

void Run::maybeBuildSpeedPlanFromFirstLap() {
  if (!options.odometryTraceMode || speedPlanReady || lapCount < 2) {
    return;
  }
  if (odometryTracePoints.size() < 3) {
    return;
  }

  const double lapLengthMm = std::max(course.lengthMm(), 1.0);
  if (lapLengthMm <= 1e-9) {
    return;
  }

  std::vector<Pose> lapTrace;
  lapTrace.reserve(odometryTracePoints.size());
  lapTrace.push_back(odometryTracePoints.front());
  double accumulatedMm = 0.0;
  for (size_t i = 1; i < odometryTracePoints.size(); ++i) {
    const Pose& prev = odometryTracePoints[i - 1];
    const Pose& curr = odometryTracePoints[i];
    const double ds = std::hypot(curr.xMm - prev.xMm, curr.yMm - prev.yMm);
    if (ds <= 1e-9) {
      continue;
    }
    if (accumulatedMm + ds >= lapLengthMm) {
      const double remain = lapLengthMm - accumulatedMm;
      const double t = std::clamp(remain / ds, 0.0, 1.0);
      lapTrace.push_back(lerpPose(prev, curr, t));
      accumulatedMm = lapLengthMm;
      break;
    }
    lapTrace.push_back(curr);
    accumulatedMm += ds;
  }
  if (lapTrace.size() < 3 || accumulatedMm < 0.95 * lapLengthMm) {
    return;
  }

  std::vector<double> lapTraceSUnwrapped;
  lapTraceSUnwrapped.reserve(lapTrace.size());
  bool hasPrevS = false;
  double prevSWrapped = 0.0;
  double prevSUnwrapped = 0.0;
  for (size_t i = 0; i < lapTrace.size(); ++i) {
    const Pose& p = lapTrace[i];
    const Pose& p0 = (i > 0) ? lapTrace[i - 1] : lapTrace[i];
    const Pose& p1 = (i + 1 < lapTrace.size()) ? lapTrace[i + 1] : lapTrace[i];
    const double hxRaw = p1.xMm - p0.xMm;
    const double hyRaw = p1.yMm - p0.yMm;
    const double hNorm = std::hypot(hxRaw, hyRaw);
    const CoursePoint heading = (hNorm > 1e-9)
                                    ? CoursePoint{hxRaw / hNorm, hyRaw / hNorm}
                                    : CoursePoint{std::cos(p.thetaRad), std::sin(p.thetaRad)};
    const double sHint = hasPrevS ? prevSWrapped : -1.0;
    const Course::Projection proj =
        course.project(CoursePoint{p.xMm, p.yMm}, heading, sHint, 2.0);
    const double sWrapped = proj.sMm;
    if (!hasPrevS) {
      prevSWrapped = sWrapped;
      prevSUnwrapped = sWrapped;
      lapTraceSUnwrapped.push_back(prevSUnwrapped);
      hasPrevS = true;
      continue;
    }
    double deltaS = sWrapped - prevSWrapped;
    if (deltaS < -0.5 * lapLengthMm) {
      deltaS += lapLengthMm;
    } else if (deltaS > 0.5 * lapLengthMm) {
      deltaS -= lapLengthMm;
    }
    prevSUnwrapped += deltaS;
    if (prevSUnwrapped < lapTraceSUnwrapped.back()) {
      prevSUnwrapped = lapTraceSUnwrapped.back();
    }
    lapTraceSUnwrapped.push_back(prevSUnwrapped);
    prevSWrapped = sWrapped;
  }
  if (lapTraceSUnwrapped.size() != lapTrace.size()) {
    return;
  }
  std::vector<double> lapTraceDistanceMm(lapTrace.size(), 0.0);
  for (size_t i = 1; i < lapTrace.size(); ++i) {
    const double ds = std::hypot(
        lapTrace[i].xMm - lapTrace[i - 1].xMm,
        lapTrace[i].yMm - lapTrace[i - 1].yMm);
    lapTraceDistanceMm[i] = lapTraceDistanceMm[i - 1] + ds;
  }
  const double classifyStepMm = lapTraceDistanceMm.size() > 1
                                    ? std::max(lapTraceDistanceMm.back() / (lapTraceDistanceMm.size() - 1), 1.0)
                                    : 10.0;
  const std::vector<int> lapTraceSegmentTypes =
      classifyOdometrySegmentTypes(lapTrace, classifyStepMm);

  const double planStepMm = std::max(params.courseResampleStepMm, 1.0);
  const size_t planPointCount = static_cast<size_t>(std::ceil(lapLengthMm / planStepMm)) + 1;
  speedPlanDistanceMm.assign(planPointCount, 0.0);
  for (size_t i = 0; i < planPointCount; ++i) {
    speedPlanDistanceMm[i] = std::min(static_cast<double>(i) * planStepMm, lapLengthMm);
  }
  speedPlanSegmentTypes.assign((planPointCount > 0) ? (planPointCount - 1) : 0, 0);
  auto markCurveSpan = [&](double sStartMm, double sLengthMm) {
    if (speedPlanSegmentTypes.empty() || sLengthMm <= 0.0) {
      return;
    }
    double remain = sLengthMm;
    double start = std::fmod(sStartMm, lapLengthMm);
    if (start < 0.0) {
      start += lapLengthMm;
    }
    while (remain > 1e-9) {
      const double localEnd = std::min(lapLengthMm, start + remain);
      const int k0 = std::clamp(static_cast<int>(std::floor(start / planStepMm)), 0,
                                static_cast<int>(speedPlanSegmentTypes.size() - 1));
      const int k1 = std::clamp(static_cast<int>(std::ceil(localEnd / planStepMm)) - 1, 0,
                                static_cast<int>(speedPlanSegmentTypes.size() - 1));
      for (int k = k0; k <= k1; ++k) {
        speedPlanSegmentTypes[static_cast<size_t>(k)] = 1;
      }
      remain -= (localEnd - start);
      start = 0.0;
    }
  };
  for (size_t i = 0; i + 1 < lapTraceSUnwrapped.size() && i < lapTraceSegmentTypes.size(); ++i) {
    if (lapTraceSegmentTypes[i] == 0) {
      continue;
    }
    const double deltaS = lapTraceSUnwrapped[i + 1] - lapTraceSUnwrapped[i];
    if (deltaS <= 0.0) {
      continue;
    }
    markCurveSpan(lapTraceSUnwrapped[i], deltaS);
  }

  const double maxVelocityMmS = std::max(params.vehicleMaxVelocityMmS, 0.0);
  const double curveVelocityMmS = std::clamp(params.desiredVelocityMmS, 0.0, maxVelocityMmS);
  const double accelMmSS = std::max(params.accelerationMmSS, 1e-6);
  const double brakeDistanceMm =
      std::max(0.0, (maxVelocityMmS * maxVelocityMmS - curveVelocityMmS * curveVelocityMmS) / (2.0 * accelMmSS));
  const int curvePrePadSegments =
      static_cast<int>(std::ceil((brakeDistanceMm + 40.0) / std::max(planStepMm, 1e-6)));
  const int curvePostPadSegments =
      static_cast<int>(std::ceil(30.0 / std::max(planStepMm, 1e-6)));
  if (!speedPlanSegmentTypes.empty() && (curvePrePadSegments > 0 || curvePostPadSegments > 0)) {
    std::vector<int> padded = speedPlanSegmentTypes;
    const int n = static_cast<int>(speedPlanSegmentTypes.size());
    for (int i = 0; i < n; ++i) {
      if (speedPlanSegmentTypes[static_cast<size_t>(i)] == 0) {
        continue;
      }
      for (int d = -curvePrePadSegments; d <= curvePostPadSegments; ++d) {
        int j = i + d;
        while (j < 0) {
          j += n;
        }
        while (j >= n) {
          j -= n;
        }
        padded[static_cast<size_t>(j)] = 1;
      }
    }
    speedPlanSegmentTypes.swap(padded);
  }

  std::vector<double> velocityLimitMmS(speedPlanDistanceMm.size(), maxVelocityMmS);
  for (size_t i = 0; i < speedPlanDistanceMm.size(); ++i) {
    const bool leftCurve = (i > 0 && speedPlanSegmentTypes[i - 1] != 0);
    const bool rightCurve = (i < speedPlanSegmentTypes.size() && speedPlanSegmentTypes[i] != 0);
    if (leftCurve || rightCurve) {
      velocityLimitMmS[i] = curveVelocityMmS;
    }
  }

  speedPlanVelocityMmS.assign(speedPlanDistanceMm.size(), 0.0);
  speedPlanVelocityMmS[0] = std::min(curveVelocityMmS, velocityLimitMmS[0]);
  for (size_t i = 1; i < speedPlanDistanceMm.size(); ++i) {
    const double ds = std::max(speedPlanDistanceMm[i] - speedPlanDistanceMm[i - 1], 0.0);
    const double reachable = std::sqrt(std::max(
        0.0, speedPlanVelocityMmS[i - 1] * speedPlanVelocityMmS[i - 1] + 2.0 * accelMmSS * ds));
    speedPlanVelocityMmS[i] = std::min(velocityLimitMmS[i], reachable);
  }

  const double endVelocityTargetMmS = speedPlanVelocityMmS[0];
  speedPlanVelocityMmS.back() =
      std::min(speedPlanVelocityMmS.back(), std::min(velocityLimitMmS.back(), endVelocityTargetMmS));
  for (size_t i = speedPlanDistanceMm.size() - 1; i > 0; --i) {
    const double ds = std::max(speedPlanDistanceMm[i] - speedPlanDistanceMm[i - 1], 0.0);
    const double reachable = std::sqrt(std::max(
        0.0, speedPlanVelocityMmS[i] * speedPlanVelocityMmS[i] + 2.0 * accelMmSS * ds));
    speedPlanVelocityMmS[i - 1] =
        std::min(velocityLimitMmS[i - 1], std::min(speedPlanVelocityMmS[i - 1], reachable));
  }
  for (size_t i = 1; i < speedPlanDistanceMm.size(); ++i) {
    const double ds = std::max(speedPlanDistanceMm[i] - speedPlanDistanceMm[i - 1], 0.0);
    const double reachable = std::sqrt(std::max(
        0.0, speedPlanVelocityMmS[i - 1] * speedPlanVelocityMmS[i - 1] + 2.0 * accelMmSS * ds));
    speedPlanVelocityMmS[i] =
        std::min(velocityLimitMmS[i], std::min(speedPlanVelocityMmS[i], reachable));
  }

  speedPlanReady = true;
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
  msg << "\"desiredVelocityInput\":" << desiredVelocityInputMmS << ',';
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
  const CoursePoint sensorPose = sensorPointFromPose(state.pose, params.sensorBaseMm);
  msg << "\"poseSensor\":{";
  msg << "\"x\":" << sensorPose.xMm << ',';
  msg << "\"y\":" << sensorPose.yMm;
  msg << "},";
  msg << "\"course\":{";
  msg << "\"straightLength\":" << params.courseStraightLengthMm << ',';
  msg << "\"curveRadius\":" << params.courseCurveRadiusMm << ',';
  msg << "\"length\":" << course.lengthMm() << ',';
  msg << "\"polyline\":[";
  const auto& coursePoints = course.points();
  for (size_t i = 0; i < coursePoints.size(); ++i) {
    if (i != 0) {
      msg << ',';
    }
    msg << "{\"x\":" << coursePoints[i].xMm << ",\"y\":" << coursePoints[i].yMm << '}';
  }
  msg << "]";
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
  msg << "\"course_resample_step_mm\":" << params.courseResampleStepMm << ',';
  msg << "\"course_file\":\"" << params.courseFile << "\",";
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
  msg << "],";
  msg << "\"speedPlanReady\":" << (speedPlanReady ? "true" : "false") << ',';
  msg << "\"speedPlanDistanceMm\":[";
  for (size_t i = 0; i < speedPlanDistanceMm.size(); ++i) {
    if (i != 0) {
      msg << ',';
    }
    msg << speedPlanDistanceMm[i];
  }
  msg << "],";
  msg << "\"speedPlanVelocityMmS\":[";
  for (size_t i = 0; i < speedPlanVelocityMmS.size(); ++i) {
    if (i != 0) {
      msg << ',';
    }
    msg << speedPlanVelocityMmS[i];
  }
  msg << "],";
  msg << "\"speedPlanSegmentTypes\":[";
  for (size_t i = 0; i < speedPlanSegmentTypes.size(); ++i) {
    if (i != 0) {
      msg << ',';
    }
    msg << speedPlanSegmentTypes[i];
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
  const CoursePoint sensorPose = sensorPointFromPose(state.pose, params.sensorBaseMm);
  std::cout << "Sensor x,y [mm]      : " << sensorPose.xMm << ", " << sensorPose.yMm << '\n';
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
