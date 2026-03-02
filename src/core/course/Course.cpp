#include "core/course/Course.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>

namespace {
constexpr double kPi = 3.14159265358979323846;

double dist2(const CoursePoint& a, const CoursePoint& b) {
  const double dx = a.xMm - b.xMm;
  const double dy = a.yMm - b.yMm;
  return dx * dx + dy * dy;
}

double wrappedDistanceMm(double a, double b, double lengthMm) {
  const double d = std::abs(a - b);
  if (lengthMm <= 1e-9) {
    return d;
  }
  return std::min(d, lengthMm - d);
}
}  // namespace

Course Course::fromParams(const SimParams& params) {
  Course course;
  std::vector<CoursePoint> loaded;
  if (!params.courseFile.empty()) {
    loaded = loadPointsFromFile(params.courseFile);
    if (loaded.empty()) {
      std::cerr << "failed to load course_file: " << params.courseFile
                << " (fallback to default oval)\n";
    } else {
      std::cerr << "loaded course_file: " << params.courseFile
                << " points=" << loaded.size() << '\n';
    }
  }

  if (loaded.empty()) {
    loaded = makeDefaultOvalPoints(params.courseStraightLengthMm, params.courseCurveRadiusMm);
  }

  course.points_ = finalizeClosedPolyline(loaded);
  const double resampleStepMm = std::max(params.courseResampleStepMm, 1.0);
  course.points_ = resamplePolyline(course.points_, resampleStepMm);
  if (course.points_.size() < 2) {
    course.points_ = finalizeClosedPolyline(makeDefaultOvalPoints(5000.0, 300.0));
    course.points_ = resamplePolyline(course.points_, resampleStepMm);
  }
  std::cerr << "course polyline resampled: step_mm=" << resampleStepMm
            << ", points=" << course.points_.size() << '\n';

  course.cumulativeMm_.assign(course.points_.size(), 0.0);
  for (size_t i = 1; i < course.points_.size(); ++i) {
    const double ds = std::hypot(
        course.points_[i].xMm - course.points_[i - 1].xMm,
        course.points_[i].yMm - course.points_[i - 1].yMm);
    course.cumulativeMm_[i] = course.cumulativeMm_[i - 1] + ds;
  }
  course.lengthMm_ = course.cumulativeMm_.back();
  return course;
}

double Course::lengthMm() const {
  return lengthMm_;
}

const std::vector<CoursePoint>& Course::points() const {
  return points_;
}

Course::Projection Course::project(const CoursePoint& query,
                                   const CoursePoint& headingHint,
                                   double sHintMm,
                                   double sHintWeight) const {
  Projection best{};
  if (points_.size() < 2 || lengthMm_ <= 1e-9) {
    return best;
  }

  const double headingNorm = std::hypot(headingHint.xMm, headingHint.yMm);
  const bool hasHeadingHint = headingNorm > 1e-9;
  const double hx = hasHeadingHint ? (headingHint.xMm / headingNorm) : 0.0;
  const double hy = hasHeadingHint ? (headingHint.yMm / headingNorm) : 0.0;

  double bestScore = 1e30;
  bool hasForwardCompatible = false;

  for (size_t i = 0; i + 1 < points_.size(); ++i) {
    const CoursePoint& a = points_[i];
    const CoursePoint& b = points_[i + 1];
    const double abx = b.xMm - a.xMm;
    const double aby = b.yMm - a.yMm;
    const double ab2 = abx * abx + aby * aby;
    if (ab2 <= 1e-12) {
      continue;
    }

    const double apx = query.xMm - a.xMm;
    const double apy = query.yMm - a.yMm;
    const double t = std::clamp((apx * abx + apy * aby) / ab2, 0.0, 1.0);
    CoursePoint q{
        a.xMm + t * abx,
        a.yMm + t * aby,
    };
    const double segLen = std::sqrt(ab2);
    const CoursePoint tangent{
        abx / segLen,
        aby / segLen,
    };

    const double d2 = dist2(query, q);
    const double headingAlign = tangent.xMm * hx + tangent.yMm * hy;
    const bool forwardCompatible = !hasHeadingHint || (headingAlign >= -0.15);
    if (!forwardCompatible && hasForwardCompatible) {
      continue;
    }

    const double headingPenalty =
        hasHeadingHint ? ((headingAlign >= 0.0) ? 0.0 : (1.0 - headingAlign) * 100.0) : 0.0;
    const double sCandidate = cumulativeMm_[i] + t * segLen;
    const double sPenalty = (sHintMm >= 0.0)
                                ? wrappedDistanceMm(sCandidate, sHintMm, lengthMm_) * std::max(sHintWeight, 0.0)
                                : 0.0;
    const double score = d2 + headingPenalty * headingPenalty + sPenalty * sPenalty;
    if (!hasForwardCompatible || forwardCompatible) {
      if (!hasForwardCompatible || score < bestScore) {
        bestScore = score;
        best.point = q;
        best.tangent = tangent;
        best.sMm = sCandidate;
      }
      if (forwardCompatible) {
        hasForwardCompatible = true;
      }
    }
  }

  if (best.sMm >= lengthMm_) {
    best.sMm = std::fmod(best.sMm, lengthMm_);
  }
  if (best.sMm < 0.0) {
    best.sMm += lengthMm_;
  }
  return best;
}

std::vector<CoursePoint> Course::makeDefaultOvalPoints(double straightLengthMm, double radiusMm) {
  const double straight = std::max(straightLengthMm, 1.0);
  const double radius = std::max(radiusMm, 1.0);
  const double halfStraight = 0.5 * straight;
  const int straightSegments = std::max(8, static_cast<int>(std::round(straight / 120.0)));
  const int arcSegments = 48;

  std::vector<CoursePoint> pts;
  pts.reserve(static_cast<size_t>(straightSegments * 2 + arcSegments * 2 + 4));
  pts.push_back(CoursePoint{-halfStraight, 0.0});

  for (int i = 1; i <= straightSegments; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(straightSegments);
    pts.push_back(CoursePoint{-halfStraight + t * straight, 0.0});
  }
  for (int i = 1; i <= arcSegments; ++i) {
    const double t =
        -0.5 * kPi + (kPi * static_cast<double>(i) / static_cast<double>(arcSegments));
    pts.push_back(CoursePoint{
        halfStraight + radius * std::cos(t),
        radius + radius * std::sin(t),
    });
  }
  for (int i = 1; i <= straightSegments; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(straightSegments);
    pts.push_back(CoursePoint{halfStraight - t * straight, 2.0 * radius});
  }
  for (int i = 1; i <= arcSegments; ++i) {
    const double t =
        0.5 * kPi + (kPi * static_cast<double>(i) / static_cast<double>(arcSegments));
    pts.push_back(CoursePoint{
        -halfStraight + radius * std::cos(t),
        radius + radius * std::sin(t),
    });
  }
  return pts;
}

std::vector<CoursePoint> Course::loadPointsFromFile(const std::string& path) {
  std::ifstream ifs(path);
  if (!ifs) {
    return {};
  }

  std::vector<CoursePoint> pts;
  std::string line;
  while (std::getline(ifs, line)) {
    const auto hashPos = line.find('#');
    if (hashPos != std::string::npos) {
      line = line.substr(0, hashPos);
    }
    std::istringstream iss(line);
    double x = 0.0;
    double y = 0.0;
    if (!(iss >> x >> y)) {
      continue;
    }
    pts.push_back(CoursePoint{x, y});
  }
  return pts;
}

std::vector<CoursePoint> Course::finalizeClosedPolyline(const std::vector<CoursePoint>& input) {
  std::vector<CoursePoint> pts;
  pts.reserve(input.size() + 1);
  for (const CoursePoint& p : input) {
    if (!pts.empty()) {
      const double ds2 = dist2(pts.back(), p);
      if (ds2 <= 1e-6) {
        continue;
      }
    }
    pts.push_back(p);
  }
  if (pts.size() < 3) {
    return {};
  }
  if (dist2(pts.front(), pts.back()) > 1e-6) {
    pts.push_back(pts.front());
  }
  return pts;
}

std::vector<CoursePoint> Course::resamplePolyline(const std::vector<CoursePoint>& input, double stepMm) {
  if (input.size() < 2) {
    return input;
  }
  const double dsTarget = std::max(stepMm, 1.0);
  std::vector<CoursePoint> out;
  out.reserve(input.size() * 2);
  out.push_back(input.front());
  for (size_t i = 0; i + 1 < input.size(); ++i) {
    const CoursePoint& a = input[i];
    const CoursePoint& b = input[i + 1];
    const double dx = b.xMm - a.xMm;
    const double dy = b.yMm - a.yMm;
    const double segLen = std::hypot(dx, dy);
    if (segLen <= 1e-9) {
      continue;
    }
    const int segments = std::max(1, static_cast<int>(std::ceil(segLen / dsTarget)));
    for (int k = 1; k <= segments; ++k) {
      const double t = static_cast<double>(k) / static_cast<double>(segments);
      out.push_back(CoursePoint{
          a.xMm + t * dx,
          a.yMm + t * dy,
      });
    }
  }
  if (out.size() >= 2 && dist2(out.front(), out.back()) > 1e-6) {
    out.push_back(out.front());
  }
  return out;
}
