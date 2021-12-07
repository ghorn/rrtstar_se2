#pragma once

#include <glm/glm.hpp>
#include <iostream>
#include <set>
#include <utility>
#include <vector>

#include "src/space/space_base.hpp"

namespace rrts::space::se2 {

// Same as computing x - y but guaranteed to be in [-pi, pi].
double AngleDifference(double x, double y);

using glm::dvec3;
struct Se2Coord : public dvec3 {
  using dvec3::dvec3;
  explicit Se2Coord(const glm::dvec3 &v) : dvec3(v){};
  [[nodiscard]] double DistanceSquared(const Se2Coord &other) const {
    const double dx = x - other.x;
    const double dy = y - other.y;
    const double dz = AngleDifference(z, other.z);
    return dx * dx + dy * dy + dz * dz;
  }
};

//  // simple obstacle
//  struct Sphere {
//    glm::dvec3 center;
//    double radius;
//  };

struct DubinsPath {
  int dummy{};
  Se2Coord p0{};
  Se2Coord p1{};
  double dist{0};
};

class Se2 : public SpaceBase<Se2Coord, DubinsPath, 3> {
 public:
  Se2(Se2Coord lb, Se2Coord ub) : lb_(lb), ub_(ub){};
  ~Se2() override = default;

  [[nodiscard]] double MuXfree() const override;
  Se2Coord SampleFree() override;
  [[nodiscard]] Se2Coord Steer(const Se2Coord &v0, const Se2Coord &v1, double eta) const override;

  // bridges
  [[nodiscard]] bool CollisionFree(const DubinsPath &line) const override;
  [[nodiscard]] double BridgeCost(const DubinsPath &line) const override { return line.dist; };
  [[nodiscard]] DubinsPath FormBridge(const Se2Coord &v0, const Se2Coord &v1) const override;
  [[nodiscard]] std::array<bool, 3> Periodic() const override { return {false, false, true}; }

 private:
  Se2Coord Sample();
  //[[nodiscard]] bool PointInSpheres(const Se2Coord &p) const;

  Se2Coord lb_;
  Se2Coord ub_;
  // std::vector<Sphere> sphere_obstacles_;
  std::mt19937_64 rng_engine_{};  // NOLINT(cert-msc32-c,cert-msc51-cpp)
  std::uniform_real_distribution<double> uniform_distribution_{};
};

}  // namespace rrts::space::se2
