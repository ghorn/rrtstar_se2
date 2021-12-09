#pragma once

#include <array>
#include <functional>
#include <iostream>
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

template <typename Point, size_t D>
using BoundingBoxesFunction =
    std::function<std::array<BoundingBoxIntervals, D>(double best_distance)>;

template <typename Point>
using DistanceFunction = std::function<double(const Point &)>;

template <typename Point, size_t D>
struct TreeBase {
  virtual ~TreeBase() = default;

  virtual void Insert(const Tagged<Point> &new_point) = 0;
  virtual Tagged<Point> Nearest(const DistanceFunction<Point> &distance_fun,
                                const BoundingBoxesFunction<Point, D> &bb_fun) const = 0;
  virtual std::vector<Tagged<Point>> Near(const DistanceFunction<Point> &distance_fun,
                                          std::array<BoundingBoxIntervals, D> bounding_boxes,
                                          double radius) const = 0;
  [[nodiscard]] virtual double Cardinality() const = 0;
};
}  // namespace rrts::tree
