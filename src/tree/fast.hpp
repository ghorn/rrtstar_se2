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
template <typename Point, size_t D>
struct Fast : public TreeBase<Point, D> {
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
  Tagged<Point> Nearest(const DistanceFunction<Point> &distance_fun,
                        const BoundingBoxesFunction<Point, D> &compute_bbs) const override {
    // Test point will be overwritten because closest point distance is so high.
    Tagged<Point> closest_point = {2222222222222222, {}};
    double closest_point_distance = 1e9;
    std::array<BoundingBoxIntervals, D> bbs = compute_bbs(closest_point_distance);

    root_.Nearest(&closest_point, &closest_point_distance, &bbs, 0, lb_, ub_, distance_fun,
                  compute_bbs);

    return closest_point;
  }

  std::vector<Tagged<Point>> Near(const DistanceFunction<Point> &distance_fun,
                                  std::array<BoundingBoxIntervals, D> bounding_boxes,
                                  double radius) const override {
    std::vector<Tagged<Point>> close_points;

    typename Node<Point, D>::SearchParams search_params;
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
  Node<Point, D> root_;

};  // struct Fast

}  // namespace rrts::tree
