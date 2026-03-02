#pragma once

#include "core/common/Types.h"

#include <vector>

struct CoursePoint {
  double xMm{0.0};
  double yMm{0.0};
};

class Course {
 public:
  struct Projection {
    CoursePoint point{};
    CoursePoint tangent{};
    double sMm{0.0};
  };

  static Course fromParams(const SimParams& params);

  double lengthMm() const;
  const std::vector<CoursePoint>& points() const;
  Projection project(const CoursePoint& query,
                    const CoursePoint& headingHint,
                    double sHintMm = -1.0,
                    double sHintWeight = 1.0) const;

 private:
  static std::vector<CoursePoint> makeDefaultOvalPoints(double straightLengthMm, double radiusMm);
  static std::vector<CoursePoint> loadPointsFromFile(const std::string& path);
  static std::vector<CoursePoint> finalizeClosedPolyline(const std::vector<CoursePoint>& input);
  static std::vector<CoursePoint> resamplePolyline(const std::vector<CoursePoint>& input, double stepMm);

  std::vector<CoursePoint> points_;
  std::vector<double> cumulativeMm_;
  double lengthMm_{0.0};
};
