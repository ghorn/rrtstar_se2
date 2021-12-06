#pragma once

#include <iostream>
#include <random>
#include <set>
#include <vector>

namespace rrts::space {


  template <typename Point, typename Line>
  class SpaceBase {
  public:
    SpaceBase() : rng_engine(), uniform_distribution() {};
    virtual ~SpaceBase() = default;

    virtual Point SampleFree() = 0;
    virtual Point Steer(const Point&, const Point&, double eta) const = 0;

    [[nodiscard]] virtual double mu_Xfree() const = 0;

    // bridges
    virtual bool CollisionFree(const Line&) const = 0;
    virtual double BridgeCost(const Line &line) const = 0;
    virtual Line FormBridge(const Point &v0, const Point &v1) const = 0;

  protected:
    std::mt19937_64 rng_engine;
    std::uniform_real_distribution<double> uniform_distribution;
  };

}  // namespace rrts::space
