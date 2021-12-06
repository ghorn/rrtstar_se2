#pragma once

#include <array>
#include <iostream>
#include <random>
#include <set>
#include <vector>

namespace rrts::space {


  template <typename Point, typename Bridge, size_t D>
  class SpaceBase {
  public:
    virtual ~SpaceBase() = default;

    virtual Point SampleFree() = 0;
    virtual Point Steer(const Point&, const Point&, double eta) const = 0;

    [[nodiscard]] virtual double mu_Xfree() const = 0;

    // bridges
    virtual bool CollisionFree(const Bridge&) const = 0;
    virtual double BridgeCost(const Bridge &bridge) const = 0;
    virtual Bridge FormBridge(const Point &v0, const Point &v1) const = 0;

    // periodic
    virtual std::array<bool, D> Periodic() const = 0;
  };

}  // namespace rrts::space
