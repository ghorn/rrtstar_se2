#include "r3_point.hpp"

#include <cassert>  // M_PI
#include <cmath>  // M_PI
#include <iostream>

namespace rrts {
namespace R3 {

  double DistanceSquared(const Point &p0, const Point &p1) {
    const double dx = p1.x - p0.x;
    const double dy = p1.y - p0.y;
    const double dz = p1.z - p0.z;
    return dx*dx + dy*dy + dz*dz;
  }

} // namespace R3
} // namespace rrts
