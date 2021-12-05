#pragma once

#include <cmath> // M_PI
#include <glm/glm.hpp>
#include <iostream>
#include <memory>
#include <string>
#include <variant>
#include <vector>

#include "src/tagged.hpp"
#include "src/tree/node.hpp"
#include "src/tree/tree_base.hpp"

namespace rrts::tree {

// Just a Node with some meta-information and helper functions.
template <typename Point>
struct Fast : public TreeBase<Point> {
  Fast(Point lb, Point ub) : lb_(lb), ub_(ub), root_(Empty{}) {};

  int64_t num_nodes_ = 0;

  void Draw() const {root_.Draw("");}
  void Insert(const Tagged<Point> &new_point) {
    //std::cerr << "inserting point " << new_point.index << std::endl;
    num_nodes_++;
    root_.InsertPoint_(new_point, 0, lb_, ub_);
  }

  Point lb_;
  Point ub_;

  Tagged<Point> Nearest(const Point &test_point) const {
    // Test point will be overwritten because closest point distance is so high.
    // This is potentially violated if bounds are anything besides (0, 1).
    Tagged<Point> closest_point = {2222222222222222, test_point};
    double closest_point_distance = 1e9;
    double closest_point_distance_squared = 1e18;

    root_.Nearest_(&closest_point,
                   &closest_point_distance,
                   &closest_point_distance_squared,
                   test_point,
                   0,
                   lb_,
                   ub_);

    return closest_point;
  }
  std::vector<Tagged<Point> > Near(const Point &test_point,
                                   const double radius) const {
    std::vector<Tagged<Point>> close_points;

    typename Node<Point>::SearchParams search_params;
    search_params.radius = radius;
    search_params.radius_squared = radius*radius;
    search_params.test_point = test_point;

    search_params.bounding_box_lb = test_point;
    search_params.bounding_box_lb.x -= radius;
    search_params.bounding_box_lb.y -= radius;
    search_params.bounding_box_lb.z -= radius;

    search_params.bounding_box_ub = test_point;
    search_params.bounding_box_ub.x += radius;
    search_params.bounding_box_ub.y += radius;
    search_params.bounding_box_ub.z += radius;

    root_.Near_(&close_points, search_params, 0, lb_, ub_);
    return close_points;
  }

  [[nodiscard]] double Cardinality() const {
    // TODO(greg): this might be wrong
    return static_cast<double>(num_nodes_);
  }
private:
  Node<Point> root_;

};  // struct Fast

} // namespace rrts
