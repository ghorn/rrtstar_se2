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
  
}  // namespace Se2
