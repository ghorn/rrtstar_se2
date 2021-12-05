#pragma once

#include <iostream>
#include <vector>
#include <set>
#include <random>

namespace rrts {
namespace space {


  template <typename Point, typename Line>
  class SpaceBase {
  public:
    SpaceBase(){};
    virtual ~SpaceBase() = default;

    virtual Point SampleFree() = 0;
    virtual Point Steer(const Point&, const Point&, const double eta) const = 0;

    virtual double mu_Xfree() const = 0;

    // bridges
    virtual bool CollisionFree(const Line&) const = 0;
    virtual double BridgeCost(const Line &line) const = 0;
    virtual Line FormBridge(const Point &v0, const Point &v1) const = 0;

  protected:
    std::mt19937_64 rng_engine;
    std::uniform_real_distribution<double> uniform_distribution;
  };

}  // namespace space
}  // namespace rrts
