#pragma once

#include <cmath>  // M_PI
#include <glm/glm.hpp>
#include <iostream>
#include <memory>
#include <string>
#include <variant>
#include <vector>

#include "src/assert.hpp"
#include "src/tagged.hpp"
#include "src/tree/node.hpp"
#include "src/tree/tree_base.hpp"

namespace rrts::tree {

// Just a Node with some meta-information and helper functions.
template <typename Point, typename Bridge, size_t D>
struct Fast : public TreeBase<Point, Bridge, D> {
  Fast(Point lb, Point ub) : num_nodes_(0), lb_(lb), ub_(ub), root_(Empty{}){};

  void Draw() const { root_.Draw(""); }
  void Insert(const Tagged<Point> &new_point) override {
    num_nodes_++;
    root_.InsertPoint(new_point, 0, lb_, ub_);
  }

 private:
  int64_t num_nodes_;
  Point lb_;
  Point ub_;

 public:
  std::tuple<Tagged<Point>, Bridge> Nearest(
      const DistanceFunction<Point, Bridge> &distance_fun,
      const BoundingBoxesFunction<D> &compute_bbs) const override {
    std::optional<std::tuple<Tagged<Point>, Bridge> > closest_point{};

    // Need an initial bounding box. Note that this could be std::optional and we could compute
    // on the first point we hit, but the bounding box also has a "centroid" which helps choose
    // which side of the split to explore first, which is a huge speedup that would be defered.
    // TODO(greg): test the speedup and eliminate this constant
    double initial_distance_for_bounding_box = 1e22;
    std::array<BoundingBoxIntervals, D> bbs = compute_bbs(initial_distance_for_bounding_box);

    root_.Nearest(closest_point, bbs, 0, lb_, ub_, distance_fun, compute_bbs);
    ASSERT(closest_point.has_value());

    return *closest_point;
  }

  std::vector<std::tuple<Tagged<Point>, Bridge> > Near(
      const DistanceFunction<Point, Bridge> &distance_fun,
      std::array<BoundingBoxIntervals, D> bounding_boxes, double radius) const override {
    std::vector<std::tuple<Tagged<Point>, Bridge> > close_points;

    typename Node<Point, Bridge, D>::SearchParams search_params;
    search_params.radius = radius;
    search_params.distance_function = distance_fun;
    search_params.bounding_boxes = bounding_boxes;

    root_.Near(&close_points, search_params, 0, lb_, ub_);
    return close_points;
  }

  [[nodiscard]] double Cardinality() const override {
    // TODO(greg): this might be wrong
    return static_cast<double>(num_nodes_);
  }

 private:
  Node<Point, Bridge, D> root_;

};  // struct Fast

}  // namespace rrts::tree
