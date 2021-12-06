#pragma once

#include <cmath> // M_PI
#include <glm/glm.hpp>
#include <iostream>
#include <memory>
#include <string>
#include <variant>
#include <vector>

#include "src/tagged.hpp"
#include "src/tree/tree_base.hpp"

namespace rrts::tree {

// helper type for the visitor
template<class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
// explicit deduction guide (not needed as of C++20)
template<class... Ts> overloaded(Ts...) -> overloaded<Ts...>;

//    lb0             ub0
//     |---------------|
//             lb1           ub1
//              |-------------|
static inline bool IntervalIntersects(const double lb0,
                                      const double ub0,
                                      const double lb1,
                                      const double ub1) {
  assert(lb0 < ub0);
  assert(lb1 < ub1);
  return lb0 <= ub1 && lb1 <= ub0;
}

// No data points.
struct Empty {
  // TODO(greg): std::monostate
};
  
// 3-dimensional binary tree. Not a K-D tree, I forget the real name for it.
// Each branch in the tree divides a spatial dimension in half. Each layer of the tree
// rotates the axis being divided.
template <typename Point>
struct Node {

  // A single data point.
  struct Leaf {
    Leaf(Tagged<Point> point) : point_(point) {};
    Tagged<Point> point_;
  };

  // Two or more data points. Not necessarily balanced - they could both be in the
  // left branch with nothing in the right branch, for instance.
  struct Split {
    // Forwarding constructor which will accept arguments of any type which can be implicitly
    // converted to an `Node`
    template <typename L, typename R>
    Split(L &&l, R &&r) :
        left_{std::make_unique<Node>(Node{std::forward<L>(l)})},
        right_{std::make_unique<Node>(Node{std::forward<R>(r)})} {}

    // // Forwarding constructor which will accept arguments of any type which can be implicitly
    // // converted to an `Node`
    // template <typename L, typename R>
    // Split(L &&l, R &&r) :
    //   left_{std::make_unique<Node>(std::forward<L>(l))},
    //   right_{std::make_unique<Node>(std::forward<R>(r))} {}

    //Split(Node &&left, Node &&right) :
    //  left_{std::make_unique<Node>(std::move(left))},
    //  right_{std::make_unique<Node>(std::move(right))} {}

    //Split(std::unique_ptr<Node> left, std::unique_ptr<Node> right) :
    //  left_{std::move(left)},
    //  right_{std::move(right)} {}
    std::unique_ptr<Node> left_;
    std::unique_ptr<Node> right_;

    void SearchNearestLeft(Tagged<Point> * const closest_point,
                           double * const closest_point_distance,
                           double * const closest_point_distance_squared,
                           const Point test_point,
                           const int32_t axis,
                           const Point tree_lb,
                           const Point tree_ub) const {
      const double tree_axis_lb = tree_lb[axis];
      const double tree_axis_ub = tree_ub[axis];
      const double tree_axis_mid = 0.5*(tree_axis_lb + tree_axis_ub);

      double bb_axis_lb = test_point[axis] - *closest_point_distance;
      double bb_axis_ub = test_point[axis] + *closest_point_distance;
      if (IntervalIntersects(tree_axis_lb, tree_axis_mid, bb_axis_lb, bb_axis_ub)) {
        Point left_branch_ub = tree_ub;
        left_branch_ub[axis] = tree_axis_mid;
        left_->Nearest_(closest_point,
                        closest_point_distance,
                        closest_point_distance_squared,
                        test_point,
                        NextAxis(axis),
                        tree_lb,
                        left_branch_ub);
      }
    }
    void SearchNearestRight(Tagged<Point> * const closest_point,
                            double * const closest_point_distance,
                            double * const closest_point_distance_squared,
                            const Point test_point,
                            const int32_t axis,
                            const Point tree_lb,
                            const Point tree_ub) const {
      const double tree_axis_lb = tree_lb[axis];
      const double tree_axis_ub = tree_ub[axis];
      const double tree_axis_mid = 0.5*(tree_axis_lb + tree_axis_ub);

      double bb_axis_lb = test_point[axis] - *closest_point_distance;
      double bb_axis_ub = test_point[axis] + *closest_point_distance;
      if (IntervalIntersects(tree_axis_mid, tree_axis_ub, bb_axis_lb, bb_axis_ub)) {
        Point right_branch_lb = tree_lb;
        right_branch_lb[axis] = tree_axis_mid;
        right_->Nearest_(closest_point,
                         closest_point_distance,
                         closest_point_distance_squared,
                         test_point,
                         NextAxis(axis),
                         right_branch_lb,
                         tree_ub);
      }
    }
  };  // struct Split

  //Node(Empty empty) : value_(empty){}
  //Node(Leaf leaf) : value_(leaf){}
  //Node(Split split) :value_(std::move(split)){}
  explicit Node(const Empty &e) : value_{e} {}
  explicit Node(const Leaf &l) : value_{l} {}
  explicit Node(Split &&s) : value_{std::move(s)} {}
  Node &operator=(const Empty &e) { value_ = e; return *this; };
  Node &operator=(const Leaf &l) { value_ = l; return *this; };
  Node &operator=(Split &&s) { value_ = std::move(s); return *this; };
  //Node(const Empty &e) : value_{e} {}
  //Node(const Leaf &l) : value_{l} {}
  //Node(Split &&s) : value_{std::move(s)} {}
  //Node &operator=(const Empty &e) { value_ = e; return *this; };
  //Node &operator=(const Leaf &l) { value_ = l; return *this; };
  //Node &operator=(Split &&s) { value_ = std::move(s); return *this; };

  // Insert a new point into the node.
  void InsertPoint_(const Tagged<Point> new_point,
                    const int32_t axis,
                    const Point tree_lb,
                    const Point tree_ub) {
      assert(tree_lb.x <= new_point.point.x);
      assert(tree_lb.y <= new_point.point.y);
      assert(tree_lb.z <= new_point.point.z);
      assert(new_point.point.x <= tree_ub.x);
      assert(new_point.point.y <= tree_ub.y);
      assert(new_point.point.z <= tree_ub.z);

      std::visit(overloaded {
          // Inserting a point in Empty promotes it to Leaf.
          [this, new_point](const Empty /*unused*/) {
            //std::cerr << "Empty turning to Leaf " << new_point.index << std::endl;
            //value_ = std::variant<Empty, Leaf, Split>(Leaf(new_point));
            value_ = Leaf(new_point);
          },
          // Point inserted into Leaf must turn into Split.
          [this, new_point, tree_lb, tree_ub, axis](const Leaf leaf) {
            //std::cerr << "Leaf " << leaf.point_.index << " splitting." << std::endl;
            const Tagged<Point> &old_point = leaf.point_; // convenience/readability
            const double mid = 0.5*(tree_lb[axis] + tree_ub[axis]);
            const double new_coord = new_point.point[axis];
            const double old_coord = old_point.point[axis];
            const bool new_point_left = new_coord <= mid;
            const bool old_point_left = old_coord <= mid;
            const bool new_point_right = !new_point_left;
            const bool old_point_right = !old_point_left;
            // First point left, second point right.
            if (old_point_left && new_point_right) {
              //std::cerr << "old point left, new point right" << std::endl;
              value_ = Split{Leaf{old_point}, Leaf{new_point}};
              //Draw("");
              return;
            }
            // Second point left, first point right.
            if (old_point_right && new_point_left) {
              //std::cerr << "old point right, new point left" << std::endl;
              value_ = Split{Leaf{new_point}, Leaf{old_point}};
              //Draw("");
              return;
            }
            // Both points left.
            if (old_point_left && new_point_left) {
              //std::cerr << "old point left, new point left" << std::endl;
              Node left = Node(Leaf(leaf.point_));
              Point left_branch_ub = tree_ub;
              left_branch_ub[axis] = mid; // split axis in half
              left.InsertPoint_(new_point, NextAxis(axis), tree_lb, left_branch_ub);
              value_ = Split{std::move(left), Empty{}};
              //Draw("");
              return;
            }
            // Both points right.
            if (old_point_right && new_point_right) {
              //std::cerr << "old point right, new point right" << std::endl;
              Node right = Node(Leaf(leaf.point_));
              Point right_branch_lb = tree_lb;
              right_branch_lb[axis] = mid; // split axis in half
              right.InsertPoint_(new_point, NextAxis(axis), right_branch_lb, tree_ub);
              value_ = Split{Empty{}, std::move(right)};
              //Draw("");
              return;
            }
          },
          // Point inserted to Split will be inserted into either left or right child, depending on its coordinate.
          [this, new_point, axis, tree_lb, tree_ub](const Split &split) {
            //std::cerr << "Split splitting." << std::endl;
            const double mid = 0.5*(tree_lb[axis] + tree_ub[axis]);
            const double new_coord = new_point.point[axis];
            const bool new_point_left = new_coord <= mid;
            if (new_point_left) {
              // new point left
              Point left_branch_ub = tree_ub;
              left_branch_ub[axis] = mid; // split axis in half
              split.left_->InsertPoint_(new_point, NextAxis(axis), tree_lb, left_branch_ub);
              return;
            } else {
              // new point right
              Point right_branch_lb = tree_lb;
              right_branch_lb[axis] = mid; // split axis in half
              split.right_->InsertPoint_(new_point, NextAxis(axis), right_branch_lb, tree_ub);
              return;
            }
          }
        }, value_);
  }

  struct SearchParams{
    SearchParams() : radius(0), radius_squared(0), test_point{}, bounding_box_lb{}, bounding_box_ub{} {};
    double radius;
    double radius_squared;
    Point test_point;
    Point bounding_box_lb;
    Point bounding_box_ub;
  };
  void Near_(std::vector<Tagged<Point>> * const close_points,
             const SearchParams &params,
             const int32_t axis,
             const Point tree_lb,
             const Point tree_ub) const {
      std::visit(overloaded {
          // There are no close points inside an empty node.
          [](const Empty /*unused*/) {
          },
          // If we're at a leaf, it's worth testing.
          [&params, close_points](const Leaf leaf) {
            if (leaf.point_.point.DistanceSquared(params.test_point) <= params.radius_squared) {
              close_points->push_back(leaf.point_);
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
            const double bb_axis_lb = params.bounding_box_lb[axis];
            const double bb_axis_ub = params.bounding_box_ub[axis];
            const double tree_axis_lb = tree_lb[axis];
            const double tree_axis_ub = tree_ub[axis];
            const double tree_axis_mid = 0.5*(tree_axis_lb + tree_axis_ub);

            bool should_search_left = false;
            bool should_search_right = false;

            {
              // Normal search, no angle wrapping.
              // Left branch intersects bounding box:
              if (IntervalIntersects(tree_axis_lb, tree_axis_mid, bb_axis_lb, bb_axis_ub)) {
                should_search_left = true;
              }
              // Right branch intersects bounding box:
              if (IntervalIntersects(tree_axis_mid, tree_axis_ub, bb_axis_lb, bb_axis_ub)) {
                should_search_right = true;
              }
            }

            // Execute the recursive searches.
            if (should_search_left) {
              Point left_branch_ub = tree_ub;
              left_branch_ub[axis] = tree_axis_mid;
              split.left_->Near_(close_points,
                                 params,
                                 NextAxis(axis),
                                 tree_lb,
                                 left_branch_ub);
            }
            if (should_search_right) {
              Point right_branch_lb = tree_lb;
              right_branch_lb[axis] = tree_axis_mid;
              split.right_->Near_(close_points,
                                  params,
                                  NextAxis(axis),
                                  right_branch_lb,
                                  tree_ub);
            }
          }
        }, value_);
  }

  void Nearest_(Tagged<Point> * const closest_point,
                double * const closest_point_distance,
                double * const closest_point_distance_squared,
                const Point test_point,
                const int32_t axis,
                const Point tree_lb,
                const Point tree_ub) const {
      std::visit(overloaded {
          // There are no close points inside an empty node.
          [](const Empty /*unused*/) {
            return;
          },
          // If we're at a leaf, it's worth testing.
          [&test_point, closest_point, closest_point_distance, closest_point_distance_squared](const Leaf leaf) {
            const double test_distance_squared = leaf.point_.point.DistanceSquared(test_point);
            if (test_distance_squared < *closest_point_distance_squared) {
              *closest_point = leaf.point_;
              *closest_point_distance_squared = test_distance_squared;
              *closest_point_distance = sqrt(test_distance_squared);
            }
          },
          // If we're at a Split, only test branches which intersect the sphere's bounding box.
          //
          //                 tree_lb       tre_mid         tree_ub
          // Tree branches:    |---------------|--------------|
          //
          //                                       bb_lb                bb_ub
          // Bounding box:                           |----------------------|
          [this, &test_point, axis, &tree_lb, &tree_ub, closest_point, closest_point_distance, closest_point_distance_squared](const Split &split) {
            // The order of searching matters. If we go left to right always,
            // it'll be a breadth first search at worst.
            // We should first search the branch that is more likely to contain the close point
            // so that the bounding box can be shrunk as rapidly as possible
            const double tree_axis_mid = 0.5*(tree_lb[axis] + tree_ub[axis]);
            if (test_point[axis] < tree_axis_mid) {
              split.SearchNearestLeft(closest_point, closest_point_distance, closest_point_distance_squared,
                                      test_point, axis, tree_lb, tree_ub);
              split.SearchNearestRight(closest_point, closest_point_distance, closest_point_distance_squared,
                                       test_point, axis, tree_lb, tree_ub);
            } else {
              split.SearchNearestRight(closest_point, closest_point_distance, closest_point_distance_squared,
                                       test_point, axis, tree_lb, tree_ub);
              split.SearchNearestLeft(closest_point, closest_point_distance, closest_point_distance_squared,
                                      test_point, axis, tree_lb, tree_ub);
            }
          }
        }, value_);
    }

    void Draw(const std::string &prefix) const {
      std::visit(overloaded {
          // There are no close points inside an empty node.
          [&prefix](const Empty /*unused*/) {
            std::cerr << prefix + " x" << std::endl;
          },
          // If we're at a leaf, it's worth testing.
          [&prefix](const Leaf leaf) {
            std::cerr << prefix + " " << leaf.point_.index << std::endl;
          },
          [&prefix](const Split &split) {
            std::cerr << prefix << std::endl;
            std::cerr << prefix + "\\" << std::endl;
            split.left_->Draw(prefix  + " L");
            split.right_->Draw(prefix + " R");
          }
        }, value_);
  }

  static const int32_t tree_dimension_ = 3; // TODO(greg): don't hardcode!!!
  static int32_t NextAxis(int32_t axis) {
    int32_t next_axis = axis + 1;
    if (next_axis >= tree_dimension_) {
      next_axis = 0;
    }
    return next_axis;
  }
private:
  // The node data.
  std::variant<Empty, Leaf, Split> value_;

};
} // namespace rrts::tree
