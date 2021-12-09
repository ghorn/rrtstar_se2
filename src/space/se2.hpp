#pragma once

#include <glm/glm.hpp>
#include <iostream>
#include <set>
#include <utility>
#include <vector>

#include "src/assert.hpp"
#include "src/space/dubins/dubins.hpp"
#include "src/space/space_base.hpp"
#include "src/tree/node.hpp"  // BoundingBox

namespace rrts::space::se2 {

using Se2Coord = dubins::Se2Coord;
using DubinsPath = dubins::DubinsPath;
using BoundingBoxInterval = rrts::tree::BoundingBoxInterval;
using BoundingBoxIntervals = rrts::tree::BoundingBoxIntervals;

// simple obstacle
struct Sphere {
  glm::dvec2 center;
  double radius;
};

using BoundingBoxReflector =
    std::function<std::vector<BoundingBoxInterval>(int32_t axis, BoundingBoxInterval bb)>;

class Se2 : public SpaceBase<Se2Coord, DubinsPath, 3> {
 public:
  Se2(double rho, const glm::dvec2 &lb, const glm::dvec2 &ub, std::vector<Sphere> sphere_obstacles)
      : rho_(rho), lb_{lb, -M_PI}, ub_{ub, M_PI}, sphere_obstacles_(std::move(sphere_obstacles)){};
  ~Se2() override = default;

  [[nodiscard]] double MuXfree() const override;
  Se2Coord SampleFree() override;
  [[nodiscard]] Se2Coord Steer(const Se2Coord &v0, const Se2Coord &v1, double eta) const override;

  // bridges
  [[nodiscard]] bool CollisionFree(const DubinsPath &path) const override;
  [[nodiscard]] double BridgeCost(const DubinsPath &line) const override {
    return line.TotalLength();
  };
  [[nodiscard]] DubinsPath FormBridge(const Se2Coord &v0, const Se2Coord &v1) const override;

  // efficient search
  [[nodiscard]] std::array<BoundingBoxIntervals, 3> BoundingBox(const Se2Coord &p,
                                                  double max_distance) const override;

  [[nodiscard]] const Se2Coord &Lb() const override { return lb_; };
  [[nodiscard]] const Se2Coord &Ub() const override { return ub_; };

 private:
  Se2Coord Sample();
  [[nodiscard]] bool Se2CoordInSpheres(const Se2Coord &p) const;

  double rho_;
  Se2Coord lb_;
  Se2Coord ub_;
  std::vector<Sphere> sphere_obstacles_;
  std::mt19937_64 rng_engine_{};  // NOLINT(cert-msc32-c,cert-msc51-cpp)
  std::uniform_real_distribution<double> uniform_distribution_{};
};

}  // namespace rrts::space::se2
