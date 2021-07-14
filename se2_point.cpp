#include "se2_point.hpp"

#include <iostream>

namespace Se2 {

  Axis IncrementAxis(const Axis axis) {
    switch (axis) {
    case Axis::kX:
      {
        return Axis::kY;
      }
    case Axis::kY:
      {
        return Axis::kTheta;
      }
    case Axis::kTheta:
      {
        return Axis::kX;
      }
    default:
      {
        std::cerr << "Illegal Se2Axis value " << static_cast<int32_t>(axis) << std::endl;
        std::exit(1);
      }
    }
  }

  double RelevantCoord(const Axis axis, const Point point) {
    switch (axis) {
    case Axis::kX:
      {
        return point.x;
      }
    case Axis::kY:
      {
        return point.y;
      }
    case Axis::kTheta:
      {
        return point.theta;
      }
    default:
      {
        std::cerr << "Illegal Se2Axis value " << static_cast<int32_t>(axis) << std::endl;
        std::exit(1);
      }
    }
  }

} // namespace Se2
