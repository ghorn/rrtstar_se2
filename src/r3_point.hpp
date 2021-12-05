#pragma once

#include <glm/glm.hpp>

namespace rrts {
namespace R3 {

  using Point = glm::dvec3;

  double DistanceSquared(const Point &p0, const Point &p1);

}  // namespace R3
}  // namespace rrts
