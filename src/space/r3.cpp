#include "src/space/r3.hpp"

#include <algorithm>              // for any_of
#include <cassert>                // for assert
#include <cmath>                  // for isnormal, sqrt
#include <cstddef>                // for size_t
#include <cstdint>                // for int32_t
#include <glm/gtx/intersect.hpp>  // for intersectLineSphere
#include <glm/gtx/norm.hpp>       // for distance2

namespace rrts::space::r3 {
Point R3::Sample() {
  const double x = lb_.x + (ub_.x - lb_.x) * uniform_distribution_(rng_engine_);
  const double y = lb_.y + (ub_.y - lb_.y) * uniform_distribution_(rng_engine_);
  const double z = lb_.z + (ub_.z - lb_.z) * uniform_distribution_(rng_engine_);
  return {x, y, z};
}

bool R3::PointInSpheres(const Point &p) const {
  auto point_in_sphere = [&p](const Sphere &s) {
    return glm::distance2(p, s.center) <= s.radius * s.radius;
  };
  return std::any_of(sphere_obstacles_.cbegin(), sphere_obstacles_.cend(), point_in_sphere);
}

Point R3::SampleFree() {
  for (;;) {
    Point p = Sample();
    if (!PointInSpheres(p)) {
      return p;
    }
  }
}

double R3::MuXfree() const {
  const double dx = ub_.x - lb_.x;
  const double dy = ub_.y - lb_.y;
  const double dz = ub_.z - lb_.z;
  double volume = dx * dy * dz;
  //// subtract sphere obstacles
  // for (const Sphere &s : sphere_obstacles_) {
  //  const double r = s.radius;
  //  volume -= 4/3*M_PI*r*r*r;
  //}
  return volume;
}

Line R3::FormBridge(const Point &v0, const Point &v1) const {
  Line line{};
  line.p0 = v0;
  line.p1 = v1;
  line.dist = glm::distance(glm::dvec3(v0), glm::dvec3(v1));

  return line;
}

std::tuple<Point, Line> R3::Steer(const Point &v0, const Point &v1, double eta) const {
  const double dx = v1.x - v0.x;
  const double dy = v1.y - v0.y;
  const double dz = v1.z - v0.z;

  const double dist = sqrt(dx * dx + dy * dy + dz * dz);

  // if distance is shorter than eta, then go all the way to the second point
  if (dist <= eta) {
    return std::make_tuple(v1, FormBridge(v0, v1));
  }

  // otherwise we have to shorten the distance to eta
  const double scale_factor = eta / dist;

  Point vret = v0;
  vret.x += dx * scale_factor;
  vret.y += dy * scale_factor;
  vret.z += dz * scale_factor;

  assert(std::isnormal(vret.x));  // NOLINT
  assert(std::isnormal(vret.y));  // NOLINT
  assert(std::isnormal(vret.z));  // NOLINT

  // fprintf(stderr, "%8.4f %8.4f %8.4f\n", vret.x, vret.y, vret.z);
  return std::make_tuple(vret, FormBridge(v0, vret));
}

bool R3::CollisionFree(const Line &line) const {
  // intersect with a couple dummy spheres
  const glm::dvec3 &gx0 = line.p0;
  const glm::dvec3 &gx1 = line.p1;
  for (const Sphere &s : sphere_obstacles_) {
    glm::dvec3 intersect_pos1;
    glm::dvec3 intersect_normal1;
    glm::dvec3 intersect_pos2;
    glm::dvec3 intersect_normal2;
    bool intersect =
        glm::intersectLineSphere<glm::dvec3>(gx0, gx1, s.center, s.radius, intersect_pos1,
                                             intersect_normal1, intersect_pos2, intersect_normal2);
    if (intersect) {
      return false;
    }
  }
  return true;
}

std::array<BoundingBoxIntervals, 3> R3::BoundingBox(const Point &p, double max_distance) const {
  std::array<BoundingBoxIntervals, 3> bbs{};

  for (size_t k = 0; k < 3; k++) {
    double coord = p[static_cast<int32_t>(k)];
    bbs[k].centroid = coord;
    bbs[k].intervals.push_back(BoundingBoxInterval{coord - max_distance, coord + max_distance});
  }

  return bbs;
}

}  // namespace rrts::space::r3
