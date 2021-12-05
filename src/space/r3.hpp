#pragma once

#include <iostream>
#include <vector>
#include <set>
#include <glm/glm.hpp>

#include "src/space/space_base.hpp"

namespace rrts {
namespace space {
namespace r3 {

  using glm::dvec3;
  struct Point : public dvec3 {
    using dvec3::dvec3;
    Point(const glm::dvec3 &v) : dvec3(v) {};
    double DistanceSquared(const Point &other) const {
      const double dx = x - other.x;
      const double dy = y - other.y;
      const double dz = z - other.z;
      return dx*dx + dy*dy + dz*dz;
    }
  };

  // simple obstacle
  struct Sphere {
    glm::dvec3 center;
    double radius;
  };

  struct Line {
    Point p0;
    Point p1;
    double dist;
  };

  class R3 : public SpaceBase<Point, Line> {
  public:
    R3(Point lb, Point ub, std::vector<Sphere> sphere_obstacles) :
      lb_(lb), ub_(ub), sphere_obstacles_(sphere_obstacles) {};
    ~R3() = default;

    double mu_Xfree() const;
    Point SampleFree();
    Point Steer(const Point&, const Point&, const double eta) const;

    // bridges
    bool CollisionFree(const Line&) const;
    double BridgeCost(const Line &line) const {return line.dist;};
    Line FormBridge(const Point &v0, const Point &v1) const;

    Point lb_;
    Point ub_;
    std::vector<Sphere> sphere_obstacles_;
  private:
    Point Sample();
    bool PointInSphere(const Point &p);
  };

}  // namespace r3
}  // namespace space
}  // namespace rrts
