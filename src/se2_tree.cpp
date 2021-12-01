#include "se2_tree.hpp"

#include <cassert>
#include <cmath> // M_PI
#include <iostream>

namespace Se2 {

// helper type for the visitor
template<class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
// explicit deduction guide (not needed as of C++20)
template<class... Ts> overloaded(Ts...) -> overloaded<Ts...>;

void Node::InsertPoint_(const Point new_point,
                        const Axis axis,
                        const Point tree_lb,
                        const Point tree_ub) {
  assert(tree_lb.x <= new_point.x);
  assert(tree_lb.y <= new_point.y);
  assert(tree_lb.theta <= new_point.theta);
  assert(new_point.x <= tree_ub.x);
  assert(new_point.y <= tree_ub.y);
  assert(new_point.theta <= tree_ub.theta);
  std::visit(overloaded {
      // Inserting a point in Empty promotes it to Leaf.
      [this, new_point](const Empty) {
        std::cerr << "Empty turning to Leaf." << std::endl;
        //value_ = std::variant<Empty, Leaf, Split>(Leaf(new_point));
        value_ = Leaf(new_point);
      },
      // Point inserted into Leaf must turn into Split.
      [this, new_point, tree_lb, tree_ub, axis](const Leaf leaf) {
        std::cerr << "Leaf splitting." << std::endl;
        const Point &old_point = leaf.point_; // convenience/readability
        const double mid = Midpoint(axis, tree_lb, tree_ub);
        const double new_coord = RelevantCoord(axis, new_point);
        const double old_coord = RelevantCoord(axis, old_point);
        const bool new_point_left = new_coord <= mid;
        const bool old_point_left = old_coord <= mid;
        const bool new_point_right = !new_point_left;
        const bool old_point_right = !old_point_left;
        // First point left, second point right.
        if (old_point_left && new_point_right) {
          value_ = Split{Leaf{old_point}, Leaf{new_point}};
          return;
        }
        // Second point left, first point right.
        if (old_point_right && new_point_left) {
          value_ = Split{Leaf{new_point}, Leaf{old_point}};
          return;
        }
        // Both points left.
        if (old_point_left && new_point_left) {
          Node left = Node(Leaf(leaf.point_));
          const Point left_branch_ub = SetValue(tree_ub, axis, mid); // split axis in half
          left.InsertPoint_(new_point, IncrementAxis(axis), tree_lb, left_branch_ub);
          value_ = Split{std::move(leaf), Empty{}};
          return;
        }
        // Both points right.
        if (old_point_right && new_point_right) {
          Node right = Node(Leaf(leaf.point_));
          const Point right_branch_lb = SetValue(tree_lb, axis, mid); // split axis in half
          right.InsertPoint_(new_point, IncrementAxis(axis), right_branch_lb, tree_ub);
          value_ = Split{Empty{}, std::move(leaf)};
          return;
        }
      },
      // Point inserted to Split will be inserted into either left or right child, depending on its coordinate.
      [new_point, axis, tree_lb, tree_ub](const Split &split) {
        std::cerr << "Split splitting." << std::endl;
        const double mid = Midpoint(axis, tree_lb, tree_ub);
        const double new_coord = RelevantCoord(axis, new_point);
        const bool new_point_left = new_coord <= mid;
        const bool new_point_right = !new_point_left;
        if (new_point_left) {
          const Point left_branch_ub = SetValue(tree_ub, axis, mid); // split axis in half
          split.left_->InsertPoint_(new_point, IncrementAxis(axis), tree_lb, left_branch_ub);
          return;
        }
        if (new_point_right) {
          const Point right_branch_lb = SetValue(tree_lb, axis, mid); // split axis in half
          split.right_->InsertPoint_(new_point, IncrementAxis(axis), right_branch_lb, tree_ub);
          return;
        }
      }
    }, value_);
}

static inline bool IntervalIntersects(const double lb0,
                                      const double ub0,
                                      const double lb1,
                                      const double ub1) {
  assert(ub0 > lb0);
  assert(ub1 > lb1);
  return ub1 >= lb0 && lb1 <= ub0;
}

void Node::PointsWithinRadiusOf_(std::vector<Point> * const close_points,
                                 const SearchParams &params,
                                 const Axis axis,
                                 const Point tree_lb,
                                 const Point tree_ub) {
  // These are going to fail. Keep them in until I run the program as a sanity check.
  assert(tree_lb.x <= params.test_point.x);
  assert(tree_lb.y <= params.test_point.y);
  assert(tree_lb.theta <= params.test_point.theta);
  assert(params.test_point.x <= tree_ub.x);
  assert(params.test_point.y <= tree_ub.y);
  assert(params.test_point.theta <= tree_ub.theta);

  std::visit(overloaded {
      // There are no close points inside an empty node.
      [](const Empty) {
      },
      // If we're at a leaf, it's worth testing.
      [&params, close_points](const Leaf leaf) {
        if (DistanceSquared(leaf.point_, params.test_point) <= params.radius_squared) {
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
        const double bb_axis_lb = RelevantCoord(axis, params.bounding_box_lb);
        const double bb_axis_ub = RelevantCoord(axis, params.bounding_box_ub);
        const double tree_axis_lb = RelevantCoord(axis, tree_lb);
        const double tree_axis_ub = RelevantCoord(axis, tree_ub);
        const double tree_axis_mid = 0.5*(tree_axis_lb + tree_axis_ub);

        bool should_search_left = false;
        bool should_search_right = false;

        if (axis != Axis::kTheta) {
          // Normal search, no angle wrapping.
          // Left branch intersects bounding box:
          if (IntervalIntersects(tree_axis_lb, tree_axis_mid, bb_axis_lb, bb_axis_ub)) {
            should_search_left = true;
          }
          // Right branch intersects bounding box:
          if (IntervalIntersects(tree_axis_mid, tree_axis_ub, bb_axis_lb, bb_axis_ub)) {
            should_search_right = true;
          }
        } else {
          // The axis is Axis::kTheta - must consider angle wrapping.
          if (params.radius >= M_PI) {
            // If the radius is greater than pi, just search both branches. If we don't do this,
            // we could get aliasing issues.
            should_search_left = true;
            should_search_right = true;
          } else {
            // The domain is between -pi and pi.
            // If a bound is outside that range, wrap it into two interval searches.
            //
            // There are three possibilities.
            //
            // 1. Bounding box goes below -pi.
            // 2. Bounding box goes above pi.
            // 3. No overflow.
            //
            // It can't go both above and below, because we already handled radius >= pi.
            // It can't go more than a factor of pi below, for the same reason.
            //
            if (bb_axis_lb < -M_PI) {
              // Possibility 1.
              const double bb_interval_1_lb = -M_PI;
              const double bb_interval_1_ub = bb_axis_ub;

              const double bb_interval_2_lb = bb_axis_lb + 2*M_PI;
              const double bb_interval_2_ub = M_PI;
              // Left branch intersects bounding box:
              if (IntervalIntersects(tree_axis_lb, tree_axis_mid, bb_interval_1_lb, bb_interval_1_ub)
                  || IntervalIntersects(tree_axis_lb, tree_axis_mid, bb_interval_2_lb, bb_interval_2_ub)) {
                should_search_left = true;
              }
              // Right branch intersects bounding box:
              if (IntervalIntersects(tree_axis_mid, tree_axis_ub, bb_interval_1_lb, bb_interval_1_ub)
                  || IntervalIntersects(tree_axis_mid, tree_axis_ub, bb_interval_2_lb, bb_interval_2_ub)) {
                should_search_right = true;
              }
            } else if (bb_axis_ub > M_PI) {
              // Possibility 2.
              const double bb_interval_1_lb = bb_axis_lb;
              const double bb_interval_1_ub = M_PI;

              const double bb_interval_2_lb = -M_PI;
              const double bb_interval_2_ub = bb_axis_ub - 2*M_PI;
              // Left branch intersects bounding box:
              if (IntervalIntersects(tree_axis_lb, tree_axis_mid, bb_interval_1_lb, bb_interval_1_ub)
                  || IntervalIntersects(tree_axis_lb, tree_axis_mid, bb_interval_2_lb, bb_interval_2_ub)) {
                should_search_left = true;
              }
              // Right branch intersects bounding box:
              if (IntervalIntersects(tree_axis_mid, tree_axis_ub, bb_interval_1_lb, bb_interval_1_ub)
                  || IntervalIntersects(tree_axis_mid, tree_axis_ub, bb_interval_2_lb, bb_interval_2_ub)) {
                should_search_right = true;
              }
            } else {
              // Possibility 3 - no overflow
              // Left branch intersects bounding box:
              if (IntervalIntersects(tree_axis_lb, tree_axis_mid, bb_axis_lb, bb_axis_ub)) {
                should_search_left = true;
              }
              // Right branch intersects bounding box:
              if (IntervalIntersects(tree_axis_mid, tree_axis_ub, bb_axis_lb, bb_axis_ub)) {
                should_search_right = true;
              }
            } // if (bb_axis_lb < -M_PI) {....} else if (bb_axis_lb > M_PI) {
          } // if (params.radius >= M_PI) {....} else {
        } // if (axis != Axis::kTheta) {....} else {

        // Execute the recursive searches.
        if (should_search_left) {
          const Point left_branch_ub = SetValue(tree_ub, axis, tree_axis_mid);
          split.left_->PointsWithinRadiusOf_(close_points,
                                             params,
                                             IncrementAxis(axis),
                                             tree_lb,
                                             left_branch_ub);
        }
        if (should_search_right) {
          const Point right_branch_lb = SetValue(tree_lb, axis, tree_axis_mid);
          split.right_->PointsWithinRadiusOf_(close_points,
                                              params,
                                              IncrementAxis(axis),
                                              right_branch_lb,
                                              tree_ub);
        }
      }
    }, value_);
}


//Tree::Tree(const double lb, const double ub) : lb_(lb), ub_(ub) {
//
//}

} // namespace Se2
