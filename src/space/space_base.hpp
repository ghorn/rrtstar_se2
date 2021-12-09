#pragma once

#include <array>
#include <iostream>
#include <random>
#include <set>
#include <vector>

#include "src/tree/tree_base.hpp"  // BoundingBoxIntervals

namespace rrts::space {

template <typename Point>
class Trajectory {
 public:
  virtual ~Trajectory() = default;
  // This should be memoized, it will get called quite a bit.
  [[nodiscard]] virtual double TrajectoryCost() const = 0;
};

using BoundingBoxIntervals = rrts::tree::BoundingBoxIntervals;

template <typename Point, typename Bridge, size_t D>
class SpaceBase {
 public:
  virtual ~SpaceBase() = default;

  virtual Point SampleFree() = 0;
  virtual std::tuple<Point, Bridge> Steer(const Point &, const Point &, double eta) const = 0;

  [[nodiscard]] virtual double MuXfree() const = 0;
  [[nodiscard]] virtual const Point &Lb() const = 0;
  [[nodiscard]] virtual const Point &Ub() const = 0;

  // bridges
  virtual bool CollisionFree(const Bridge &) const = 0;
  // TODO(greg): should this be a convenience function? doesn't steer take its place?
  virtual Bridge FormBridge(const Point &v0, const Point &v1) const = 0;

  // efficient K-D tree searching
  virtual std::array<BoundingBoxIntervals, 3> BoundingBox(const Point &p,
                                                          double max_distance) const = 0;
};

}  // namespace rrts::space
