#pragma once

#include <array>
#include <functional>
#include <iostream>
#include <tuple>
#include <vector>

#include "src/tagged.hpp"

namespace rrts::tree {

struct BoundingBoxInterval {
  double lb;
  double ub;
};

// need a centroid for a bounding box
struct BoundingBoxIntervals {
  double centroid;
  std::vector<BoundingBoxInterval> intervals;
};

template <size_t D>
void DescribeBoundingBox(const std::array<BoundingBoxIntervals, D> &bbs) {
  std::cerr << "centroid:" << std::endl;
  for (size_t k = 0; k < D; k++) {
    std::cerr << "  [" << k << "]: " << bbs[k].centroid << std::endl;
  }
  std::cerr << "intervals:" << std::endl;
  for (size_t k = 0; k < D; k++) {
    std::cerr << "  [" << k << "]:";
    for (const BoundingBoxInterval &interval : bbs[k].intervals) {
      std::cerr << "  {" << interval.lb << ", " << interval.ub << "}";
    }
    std::cerr << std::endl;
  }
}

template <size_t D>
using BoundingBoxesFunction =
    std::function<std::array<BoundingBoxIntervals, D>(double best_distance)>;

template <typename Point, typename Bridge>
using DistanceFunction = std::function<Bridge(const Point &)>;

template <typename Point, typename Bridge, size_t D>
struct TreeBase {
  virtual ~TreeBase() = default;

  virtual void Insert(const Tagged<Point> &new_point) = 0;
  virtual std::tuple<Tagged<Point>, Bridge> Nearest(
      const DistanceFunction<Point, Bridge> &distance_fun,
      const BoundingBoxesFunction<D> &bb_fun) const = 0;
  virtual std::vector<std::tuple<Tagged<Point>, Bridge> > Near(
      const DistanceFunction<Point, Bridge> &distance_fun,
      std::array<BoundingBoxIntervals, D> bounding_boxes, double radius) const = 0;
  [[nodiscard]] virtual double Cardinality() const = 0;
};
}  // namespace rrts::tree
