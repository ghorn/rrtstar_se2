#pragma once

#include "src/assert.hpp"
#include "src/tagged.hpp"
#include "src/tree/tree_base.hpp"

namespace rrts::tree {

template <typename Point, size_t D>
class Naive : public TreeBase<Point, D> {
 public:
  Naive(const Point & /*unused*/, const Point & /*unused*/) : points_{} {};

  void Insert(const Tagged<Point> &new_point) { points_.push_back(new_point); }

  Tagged<Point> Nearest(const DistanceFunction<Point> &compute_distance,
                        const BoundingBoxesFunction<Point, D> &bb_fun
                        __attribute__((unused))) const {
    ASSERT(!points_.empty());

    Tagged<Point> nearest_point = points_.at(0);
    double nearest_distance = compute_distance(nearest_point.point);

    for (size_t k = 1; k < points_.size(); k++) {
      Tagged<Point> x = points_.at(k);
      double distance = compute_distance(x.point);
      if (distance < nearest_distance) {
        nearest_distance = distance;
        nearest_point = x;
      }
    }

    return nearest_point;
  }

  std::vector<Tagged<Point>> Near(const DistanceFunction<Point> &compute_distance,
                                  std::array<BoundingBoxIntervals, D> bounding_boxes
                                  __attribute__((unused)),
                                  double radius) const {
    std::vector<Tagged<Point>> near_points;
    for (const Tagged<Point> &x : points_) {
      if (compute_distance(x.point) <= radius) {
        near_points.push_back(x);
      }
    }
    return near_points;
  }

  [[nodiscard]] double Cardinality() const {
    // TODO(greg): this might be wrong
    return static_cast<double>(points_.size());
  };

  // private:
  std::vector<Tagged<Point>> points_;
};

}  // namespace rrts::tree
