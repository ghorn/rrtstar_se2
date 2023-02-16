#pragma once

#include <array>        // for array
#include <glm/glm.hpp>  // for dvec3, vec, vec<>::vec
#include <random>       // for mt19937_64, uniform_real_distribution
#include <tuple>        // for tuple
#include <utility>      // for move
#include <vector>       // for vector

#include "src/space/space_base.hpp"  // for BoundingBoxIntervals, SpaceBase, Trajectory
#include "src/tree/tree_base.hpp"    // for BoundingBoxInterval
// #include "src/tree/node.hpp"  // BoundingBox

namespace rrts::space::r3 {

using BoundingBoxInterval = rrts::tree::BoundingBoxInterval;

using glm::dvec3;
struct R3Point : public dvec3 {
  using dvec3::dvec3;
  explicit R3Point(const glm::dvec3 &v) : dvec3(v){};
  std::string Render() const {
    return std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(z);
  }
};

// simple obstacle
struct Sphere {
  glm::dvec3 center;
  double radius;
};

class Line : public Trajectory<R3Point> {
 public:
  ~Line() override = default;
  [[nodiscard]] double TrajectoryCost() const override { return dist; }

  R3Point p0{};
  R3Point p1{};
  double dist{0};
};

class R3 : public SpaceBase<R3Point, Line, 3> {
 public:
  R3(R3Point lb, R3Point ub, std::vector<Sphere> sphere_obstacles)
      : lb_(lb), ub_(ub), sphere_obstacles_(std::move(sphere_obstacles)){};
  ~R3() override = default;

  [[nodiscard]] double MuXfree() const override;
  R3Point SampleFree() override;
  [[nodiscard]] std::tuple<R3Point, Line> Steer(const R3Point &v0, const R3Point &v1,
                                                double eta) const override;

  // bridges
  [[nodiscard]] bool CollisionFree(const Line &line) const override;
  [[nodiscard]] Line FormBridge(const R3Point &v0, const R3Point &v1) const override;

  // efficient search
  [[nodiscard]] std::array<BoundingBoxIntervals, 3> BoundingBox(const R3Point &p,
                                                                double max_distance) const override;

  [[nodiscard]] const R3Point &Lb() const override { return lb_; };
  [[nodiscard]] const R3Point &Ub() const override { return ub_; };

 private:
  R3Point Sample();
  [[nodiscard]] bool PointInSpheres(const R3Point &p) const;

  R3Point lb_;
  R3Point ub_;
  std::vector<Sphere> sphere_obstacles_;
  std::mt19937_64 rng_engine_{};  // NOLINT(cert-msc32-c,cert-msc51-cpp)
  std::uniform_real_distribution<double> uniform_distribution_{};
};

}  // namespace rrts::space::r3
