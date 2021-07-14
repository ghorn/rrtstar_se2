#pragma once

namespace Se2 {

  enum class Axis {
      kX,
      kY,
      kTheta
  };

  struct Point {
    double x;
    double y;
    double theta;
  };

  Axis IncrementAxis(const Axis axis);

  double RelevantCoord(const Axis axis, const Point point);

  double Midpoint(const Axis axis, const Point lb, const Point ub);

  Point SetValue(const Point point, const Axis axis, const double value);
  
}  // namespace Se2
