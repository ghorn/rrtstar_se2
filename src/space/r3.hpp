#pragma once

#include <glm/glm.hpp>
#include <iostream>
#include <set>
#include <utility>
#include <vector>

#include "src/space/space_base.hpp"

namespace rrts::space::r3 {

using glm::dvec3;
struct Point : public dvec3 {
  using dvec3::dvec3;
  explicit Point(const glm::dvec3 &v) : dvec3(v){};
  [[nodiscard]] double DistanceSquared(const Point &other) const {
    const double dx = x - other.x;
    const double dy = y - other.y;
    const double dz = z - other.z;
    return dx * dx + dy * dy + dz * dz;
  }
};

// simple obstacle
struct Sphere {
  glm::dvec3 center;
  double radius;
};

struct Line {
  Point p0{};
  Point p1{};
  double dist{0};
};

class R3 : public SpaceBase<Point, Line, 3> {
 public:
  R3(Point lb, Point ub, std::vector<Sphere> sphere_obstacles)
      : lb_(lb), ub_(ub), sphere_obstacles_(std::move(sphere_obstacles)){};
  ~R3() override = default;

  [[nodiscard]] double mu_Xfree() const override;
  Point SampleFree() override;
  [[nodiscard]] Point Steer(const Point &v0, const Point &v1, double eta) const override;

  // bridges
  [[nodiscard]] bool CollisionFree(const Line &line) const override;
  [[nodiscard]] double BridgeCost(const Line &line) const override { return line.dist; };
  [[nodiscard]] Line FormBridge(const Point &v0, const Point &v1) const override;
  [[nodiscard]] std::array<bool, 3> Periodic() const override { return {false, false, false}; }

 private:
  Point Sample();
  [[nodiscard]] bool PointInSpheres(const Point &p) const;

  Point lb_;
  Point ub_;
  std::vector<Sphere> sphere_obstacles_;
  std::mt19937_64 rng_engine_{};  // NOLINT(cert-msc32-c,cert-msc51-cpp)
  std::uniform_real_distribution<double> uniform_distribution_{};
};

}  // namespace rrts::space::r3
