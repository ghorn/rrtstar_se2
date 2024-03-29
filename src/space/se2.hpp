#pragma once

#include <array>                       // for array
#include <cmath>                       // for M_PI
#include <cstdint>                     // for int32_t
#include <functional>                  // for function
#include <glm/ext/vector_double2.hpp>  // for dvec2
#include <random>                      // for mt19937_64, uniform_real_distribution
#include <tuple>                       // for tuple
#include <utility>                     // for move
#include <vector>                      // for vector

#include "src/space/dubins/dubins.hpp"  // for DubinsPath, Se2Coord
#include "src/space/space_base.hpp"     // for SpaceBase
#include "src/tree/tree_base.hpp"       // for BoundingBoxInterval, BoundingBoxIntervals
// #include "src/tree/node.hpp"  // BoundingBox

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
  [[nodiscard]] std::tuple<Se2Coord, DubinsPath> Steer(const Se2Coord &v0, const Se2Coord &v1,
                                                       double eta) const override;

  // bridges
  [[nodiscard]] bool CollisionFree(const DubinsPath &path) const override;
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
