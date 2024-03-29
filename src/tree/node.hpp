#pragma once

#include <algorithm>  // for any_of
#include <array>      // for array
#include <cassert>    // for assert
#include <cstddef>    // for size_t
#include <cstdint>    // for int32_t
#include <iostream>   // for endl, cerr, ostream, basic_ostream
#include <memory>     // for make_unique, unique_ptr
#include <optional>   // for optional
#include <string>     // for operator+, operator<<, string
#include <tuple>      // for get, make_tuple
#include <utility>    // for move, forward
#include <variant>    // for tuple, visit, variant
#include <vector>     // for vector

#include "src/tagged.hpp"          // for Tagged
#include "src/tree/tree_base.hpp"  // for BoundingBoxIntervals, DistanceFunction, BoundingBoxInt...

namespace rrts::tree {

// helper type for the visitor
template <class... Ts>
struct Overloaded : Ts... {  // NOLINT(fuchsia-trailing-return)
  using Ts::operator()...;
};
// explicit deduction guide (not needed as of C++20)
template <class... Ts>
Overloaded(Ts...) -> Overloaded<Ts...>;  // NOLINT(fuchsia-trailing-return)

//    lb0             ub0
//     |---------------|
//             lb1           ub1
//              |-------------|
static inline bool IntervalIntersects(const double tree_lb, const double tree_ub,
                                      const BoundingBoxIntervals &bbs) {
  auto one_interval_intersects = [tree_lb, tree_ub](const BoundingBoxInterval &bb) {
    assert(tree_lb < tree_ub);  // NOLINT
    assert(bb.lb < bb.ub);      // NOLINT
    return tree_lb <= bb.ub && bb.lb <= tree_ub;
  };

  return std::any_of(bbs.intervals.cbegin(), bbs.intervals.cend(), one_interval_intersects);
}

// No data points.
struct Empty {
  // TODO(greg): std::monostate
};

// K-D tree with all points on leaves, not splits. Splits happen on half planes every time
// so that the tree is approximately balanced if the space is sampled uniformly.
template <typename Point, typename Bridge, size_t D>
struct Node {
  // A single data point.
  struct Leaf {
    explicit Leaf(Tagged<Point> point) : point_(point){};
    Tagged<Point> point_;
  };

  // Two or more data points. Not necessarily balanced - they could both be in the
  // left branch with nothing in the right branch, for instance.
  struct Split {
    // Forwarding constructor which will accept arguments of any type which can be implicitly
    // converted to an `Node`
    template <typename L, typename R>
    Split(L &&l, R &&r)
        : left_{std::make_unique<Node>(Node{std::forward<L>(l)})},
          right_{std::make_unique<Node>(Node{std::forward<R>(r)})} {}

    std::unique_ptr<Node> left_;
    std::unique_ptr<Node> right_;

    void SearchNearestLeft(std::optional<std::tuple<Tagged<Point>, Bridge> > &closest_point,
                           std::array<BoundingBoxIntervals, D> &bbs, const int32_t axis,
                           const Point &tree_lb, const Point &tree_ub,
                           const DistanceFunction<Point, Bridge> &distance_function,
                           const BoundingBoxesFunction<D> &compute_bbs) const {
      const double tree_axis_lb = tree_lb[axis];
      const double tree_axis_ub = tree_ub[axis];
      const double tree_axis_mid = 0.5 * (tree_axis_lb + tree_axis_ub);

      const BoundingBoxIntervals &bb_axis = bbs[static_cast<size_t>(axis)];
      if (IntervalIntersects(tree_axis_lb, tree_axis_mid, bb_axis)) {
        Point left_branch_ub = tree_ub;
        left_branch_ub[axis] = tree_axis_mid;
        left_->Nearest(closest_point, bbs, NextAxis(axis), tree_lb, left_branch_ub,
                       distance_function, compute_bbs);
      }
    }
    void SearchNearestRight(std::optional<std::tuple<Tagged<Point>, Bridge> > &closest_point,
                            std::array<BoundingBoxIntervals, D> &bbs, const int32_t axis,
                            const Point &tree_lb, const Point &tree_ub,
                            const DistanceFunction<Point, Bridge> &distance_function,
                            const BoundingBoxesFunction<D> &compute_bbs) const {
      const double tree_axis_lb = tree_lb[axis];
      const double tree_axis_ub = tree_ub[axis];
      const double tree_axis_mid = 0.5 * (tree_axis_lb + tree_axis_ub);

      const BoundingBoxIntervals &bb_axis = bbs[static_cast<size_t>(axis)];
      if (IntervalIntersects(tree_axis_mid, tree_axis_ub, bb_axis)) {
        Point right_branch_lb = tree_lb;
        right_branch_lb[axis] = tree_axis_mid;
        right_->Nearest(closest_point, bbs, NextAxis(axis), right_branch_lb, tree_ub,
                        distance_function, compute_bbs);
      }
    }
  };  // struct Split

  explicit Node(const Empty &e) : value_{e} {}
  explicit Node(const Leaf &l) : value_{l} {}
  explicit Node(Split &&s) : value_{std::move(s)} {}
  Node &operator=(const Empty &e) {  // NOLINT(fuchsia-overloaded-operator)
    value_ = e;
    return *this;
  };
  Node &operator=(const Leaf &l) {  // NOLINT(fuchsia-overloaded-operator)
    value_ = l;
    return *this;
  };
  Node &operator=(Split &&s) {  // NOLINT(fuchsia-overloaded-operator)
    value_ = std::move(s);
    return *this;
  };

  // Insert a new point into the node.
  void InsertPoint(const Tagged<Point> new_point, const int32_t axis, const Point tree_lb,
                   const Point tree_ub) {
    for (int32_t k = 0; k < static_cast<int32_t>(D); k++) {
      assert(tree_lb[k] <= new_point.Point()[k]);
      assert(new_point.Point()[k] <= tree_ub[k]);
    }

    std::visit(
        Overloaded{
            // Inserting a point in Empty promotes it to Leaf.
            [this, new_point](const Empty /*unused*/) {
              // std::cerr << "Empty turning to Leaf " << new_point.Index() << std::endl;
              // value_ = std::variant<Empty, Leaf, Split>(Leaf(new_point));
              value_ = Leaf(new_point);
            },
            // Point inserted into Leaf must turn into Split.
            [this, new_point, tree_lb, tree_ub, axis](const Leaf leaf) {
              // std::cerr << "Leaf " << leaf.point_.Index() << " splitting." << std::endl;
              const Tagged<Point> &old_point = leaf.point_;  // convenience/readability
              const double mid = 0.5 * (tree_lb[axis] + tree_ub[axis]);
              const double new_coord = new_point.Point()[axis];
              const double old_coord = old_point.Point()[axis];
              const bool new_point_left = new_coord <= mid;
              const bool old_point_left = old_coord <= mid;
              const bool new_point_right = !new_point_left;
              const bool old_point_right = !old_point_left;
              // First point left, second point right.
              if (old_point_left && new_point_right) {
                // std::cerr << "old point left, new point right" << std::endl;
                value_ = Split{Leaf{old_point}, Leaf{new_point}};
                // Draw("");
                return;
              }
              // Second point left, first point right.
              if (old_point_right && new_point_left) {
                // std::cerr << "old point right, new point left" << std::endl;
                value_ = Split{Leaf{new_point}, Leaf{old_point}};
                // Draw("");
                return;
              }
              // Both points left.
              if (old_point_left && new_point_left) {
                // std::cerr << "old point left, new point left" << std::endl;
                Node left = Node(Leaf(leaf.point_));
                Point left_branch_ub = tree_ub;
                left_branch_ub[axis] = mid;  // split axis in half
                left.InsertPoint(new_point, NextAxis(axis), tree_lb, left_branch_ub);
                value_ = Split{std::move(left), Empty{}};
                // Draw("");
                return;
              }
              // Both points right.
              if (old_point_right && new_point_right) {
                // std::cerr << "old point right, new point right" << std::endl;
                Node right = Node(Leaf(leaf.point_));
                Point right_branch_lb = tree_lb;
                right_branch_lb[axis] = mid;  // split axis in half
                right.InsertPoint(new_point, NextAxis(axis), right_branch_lb, tree_ub);
                value_ = Split{Empty{}, std::move(right)};
                // Draw("");
                return;
              }
            },
            // Point inserted to Split will be inserted into either left or right child, depending
            // on its coordinate.
            [new_point, axis, tree_lb, tree_ub](const Split &split) {
              // std::cerr << "Split splitting." << std::endl;
              const double mid = 0.5 * (tree_lb[axis] + tree_ub[axis]);
              const double new_coord = new_point.Point()[axis];
              const bool new_point_left = new_coord <= mid;
              if (new_point_left) {
                // new point left
                Point left_branch_ub = tree_ub;
                left_branch_ub[axis] = mid;  // split axis in half
                split.left_->InsertPoint(new_point, NextAxis(axis), tree_lb, left_branch_ub);
                return;
              }
              // new point right
              Point right_branch_lb = tree_lb;
              right_branch_lb[axis] = mid;  // split axis in half
              split.right_->InsertPoint(new_point, NextAxis(axis), right_branch_lb, tree_ub);
            }},
        value_);
  }

  struct SearchParams {
    double radius{0};
    DistanceFunction<Point, Bridge> distance_function{};
    std::array<BoundingBoxIntervals, D> bounding_boxes{};
  };
  void Near(std::vector<std::tuple<Tagged<Point>, Bridge> > *const close_points,
            const SearchParams &params, const int32_t axis, const Point tree_lb,
            const Point tree_ub) const {
    std::visit(
        Overloaded{
            // There are no close points inside an empty node.
            [](const Empty /*unused*/) {},
            // If we're at a leaf, it's worth testing.
            [&params, close_points](const Leaf leaf) {
              Bridge test_bridge = params.distance_function(leaf.point_.Point());
              if (test_bridge.TrajectoryCost() <= params.radius) {
                close_points->push_back(std::make_tuple(leaf.point_, test_bridge));
              }
            },
            // If we're at a Split, only test branches which intersect the sphere's bounding box.
            //
            //                 tree_lb       tre_mid         tree_ub
            // Tree branches:    |---------------|--------------|
            //
            //                                       bb_lb                bb_ub
            // Bounding box:                           |----------------------|
            [&params, close_points, axis, &tree_lb, &tree_ub](const Split &split) {
              const BoundingBoxIntervals &bb_axis =
                  params.bounding_boxes.at(static_cast<size_t>(axis));
              const double tree_axis_lb = tree_lb[axis];
              const double tree_axis_ub = tree_ub[axis];
              const double tree_axis_mid = 0.5 * (tree_axis_lb + tree_axis_ub);

              bool should_search_left = false;
              bool should_search_right = false;

              {
                // Normal search, no angle wrapping.
                // Left branch intersects bounding box:
                if (IntervalIntersects(tree_axis_lb, tree_axis_mid, bb_axis)) {
                  should_search_left = true;
                }
                // Right branch intersects bounding box:
                if (IntervalIntersects(tree_axis_mid, tree_axis_ub, bb_axis)) {
                  should_search_right = true;
                }
              }

              // Execute the recursive searches.
              if (should_search_left) {
                Point left_branch_ub = tree_ub;
                left_branch_ub[axis] = tree_axis_mid;
                split.left_->Near(close_points, params, NextAxis(axis), tree_lb, left_branch_ub);
              }
              if (should_search_right) {
                Point right_branch_lb = tree_lb;
                right_branch_lb[axis] = tree_axis_mid;
                split.right_->Near(close_points, params, NextAxis(axis), right_branch_lb, tree_ub);
              }
            }},
        value_);
  }

  void Nearest(std::optional<std::tuple<Tagged<Point>, Bridge> > &closest_point,
               std::array<BoundingBoxIntervals, D> &bbs, const int32_t axis, const Point &tree_lb,
               const Point &tree_ub, const DistanceFunction<Point, Bridge> &distance_function,
               const BoundingBoxesFunction<D> &compute_bbs) const {
    std::visit(
        Overloaded{
            // There are no close points inside an empty node.
            [](const Empty /*unused*/) { return; },
            // If we're at a leaf, it's worth testing.
            [&closest_point, &bbs, &distance_function, &compute_bbs](const Leaf leaf) {
              const Bridge test_bridge = distance_function(leaf.point_.Point());

              // if we haven't hit a node yet, then this is automatically our closest node
              if (!closest_point) {
                closest_point = std::make_tuple(leaf.point_, test_bridge);
                bbs = compute_bbs(test_bridge.TrajectoryCost());
                return;
              }

              // if we already have a closest point, then we have to compare it
              if (test_bridge.TrajectoryCost() < std::get<1>(*closest_point).TrajectoryCost()) {
                closest_point = std::make_tuple(leaf.point_, test_bridge);
                bbs = compute_bbs(test_bridge.TrajectoryCost());
              }
            },
            // If we're at a Split, only test branches which intersect the sphere's bounding box.
            //
            //                 tree_lb       tre_mid         tree_ub
            // Tree branches:    |---------------|--------------|
            //
            //                                       bb_lb                bb_ub
            // Bounding box:                           |----------------------|
            [axis, &tree_lb, &tree_ub, &closest_point, &bbs, &distance_function,
             &compute_bbs](const Split &split) {
              // The order of searching matters. If we go left to right always,
              // it'll be a breadth first search at worst.
              // We should first search the branch that is more likely to contain the close point
              // so that the bounding box can be shrunk as rapidly as possible
              const double tree_axis_mid = 0.5 * (tree_lb[axis] + tree_ub[axis]);

              const BoundingBoxIntervals &intervals = bbs[static_cast<size_t>(axis)];
              if (intervals.centroid < tree_axis_mid) {
                split.SearchNearestLeft(closest_point, bbs, axis, tree_lb, tree_ub,
                                        distance_function, compute_bbs);
                split.SearchNearestRight(closest_point, bbs, axis, tree_lb, tree_ub,
                                         distance_function, compute_bbs);
              } else {
                split.SearchNearestRight(closest_point, bbs, axis, tree_lb, tree_ub,
                                         distance_function, compute_bbs);
                split.SearchNearestLeft(closest_point, bbs, axis, tree_lb, tree_ub,
                                        distance_function, compute_bbs);
              }
            }},
        value_);
  }

  void Draw(const std::string &prefix) const {
    std::visit(
        Overloaded{// There are no close points inside an empty node.
                   [&prefix](const Empty /*unused*/) { std::cerr << prefix + " x" << std::endl; },
                   // If we're at a leaf, it's worth testing.
                   [&prefix](const Leaf leaf) {
                     std::cerr << prefix + " " << leaf.point_.Index() << std::endl;
                   },
                   [&prefix](const Split &split) {
                     std::cerr << prefix << std::endl;
                     std::cerr << prefix + "\\" << std::endl;
                     split.left_->Draw(prefix + " L");
                     split.right_->Draw(prefix + " R");
                   }},
        value_);
  }

  static int32_t NextAxis(int32_t axis) {
    int32_t next_axis = axis + 1;
    if (next_axis >= static_cast<int32_t>(D)) {
      next_axis = 0;
    }
    return next_axis;
  }

 private:
  // The node data.
  std::variant<Empty, Leaf, Split> value_;
};
}  // namespace rrts::tree
