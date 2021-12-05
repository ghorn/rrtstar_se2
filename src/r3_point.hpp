#pragma once

#include <glm/glm.hpp>

namespace rrts {
namespace R3 {

  using Point = glm::dvec3;

  double Midpoint(const int32_t axis, const Point &lb, const Point &ub);

  double DistanceSquared(const Point &p0, const Point &p1);

}  // namespace R3
}  // namespace rrts
