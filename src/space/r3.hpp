#pragma once

#include <glm/glm.hpp>
#include <iostream>
#include <set>
#include <utility>
#include <vector>

#include "src/space/space_base.hpp"
#include "src/tree/node.hpp"  // BoundingBox

namespace rrts::space::r3 {

using BoundingBoxInterval = rrts::tree::BoundingBoxInterval;

using glm::dvec3;
struct Point : public dvec3 {
  using dvec3::dvec3;
  explicit Point(const glm::dvec3 &v) : dvec3(v){};
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

  [[nodiscard]] double MuXfree() const override;
  Point SampleFree() override;
  [[nodiscard]] Point Steer(const Point &v0, const Point &v1, double eta) const override;

  // bridges
  [[nodiscard]] bool CollisionFree(const Line &line) const override;
  [[nodiscard]] double BridgeCost(const Line &line) const override { return line.dist; };
  [[nodiscard]] Line FormBridge(const Point &v0, const Point &v1) const override;

  // efficient search
  [[nodiscard]] std::array<BoundingBoxIntervals, 3> BoundingBox(const Point &p,
                                                  double max_distance) const override;

  [[nodiscard]] const Point &Lb() const override { return lb_; };
  [[nodiscard]] const Point &Ub() const override { return ub_; };

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
