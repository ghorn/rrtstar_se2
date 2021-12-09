#pragma once

#include "src/assert.hpp"
#include "src/tagged.hpp"
#include "src/tree/tree_base.hpp"

namespace rrts::tree {

template <typename Point, typename Bridge, size_t D>
class Naive : public TreeBase<Point, Bridge, D> {
 public:
  Naive(const Point & /*unused*/, const Point & /*unused*/) : points_{} {};

  void Insert(const Tagged<Point> &new_point) { points_.push_back(new_point); }

  std::tuple<Tagged<Point>, Bridge> Nearest(const DistanceFunction<Point, Bridge> &compute_distance,
                                            const BoundingBoxesFunction<D> &bb_fun
                                            __attribute__((unused))) const {
    ASSERT(!points_.empty());

    Tagged<Point> nearest_point = points_.at(0);
    Bridge bridge0 = compute_distance(nearest_point.Point());
    double lowest_cost = bridge0.TrajectoryCost();
    std::tuple<Tagged<Point>, Bridge> nearest_point_and_bridge(nearest_point, bridge0);

    for (size_t k = 1; k < points_.size(); k++) {
      Tagged<Point> x = points_.at(k);
      Bridge bridge = compute_distance(x.Point());
      if (bridge.TrajectoryCost() < lowest_cost) {
        lowest_cost = bridge.TrajectoryCost();
        nearest_point_and_bridge = std::make_tuple(x, bridge);
      }
    }

    return nearest_point_and_bridge;
  }

  std::vector<std::tuple<Tagged<Point>, Bridge>> Near(
      const DistanceFunction<Point, Bridge> &compute_distance,
      std::array<BoundingBoxIntervals, D> bounding_boxes __attribute__((unused)),
      double radius) const {
    std::vector<std::tuple<Tagged<Point>, Bridge>> near_points;
    for (const Tagged<Point> &x : points_) {
      Bridge bridge = compute_distance(x.Point());
      if (bridge.TrajectoryCost() <= radius) {
        near_points.push_back(std::make_tuple(x, bridge));
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
