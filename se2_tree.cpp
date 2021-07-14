#include "se2_tree.hpp"

#include <iostream>

Se2Axis IncrementAxis(const Se2Axis axis) {
  switch (axis) {
  case Se2Axis::kX:
    {
      return Se2Axis::kY;
    }
  case Se2Axis::kY:
    {
      return Se2Axis::kTheta;
    }
  case Se2Axis::kTheta:
    {
      return Se2Axis::kX;
    }
  default:
    {
      std::cerr << "Illegal Se2Axis value " << static_cast<int32_t>(axis) << std::endl;
      std::exit(1);
    }
  }
}

double RelevantCoord(const Se2Axis axis, const Se2Point point) {
  switch (axis) {
  case Se2Axis::kX:
    {
      return point.x;
    }
  case Se2Axis::kY:
    {
      return point.y;
    }
  case Se2Axis::kTheta:
    {
      return point.theta;
    }
  default:
    {
      std::cerr << "Illegal Se2Axis value " << static_cast<int32_t>(axis) << std::endl;
      std::exit(1);
    }
  }
}
// helper type for the visitor
template<class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
// explicit deduction guide (not needed as of C++20)
template<class... Ts> overloaded(Ts...) -> overloaded<Ts...>;

void Se2Node::InsertPoint_(const Se2Point new_point,
                           const Se2Axis axis,
                           const double lb,
                           const double ub) {
  std::visit(overloaded {
      // Inserting a point in Empty promotes it to Leaf.
      [this, new_point](const Empty) {
        //value_ = std::variant<Empty, Leaf, Split>(Leaf(new_point));
        value_ = Leaf(new_point);
      },
      // Point inserted into Leaf must turn into Split.
      [this, new_point, lb, ub, axis](const Leaf leaf) {
        const Se2Point &old_point = leaf.point_; // convenience/readability
        const double mid = 0.5 * (lb + ub);
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
          Se2Node left = Se2Node(Leaf(leaf.point_));
          left.InsertPoint_(new_point, IncrementAxis(axis), lb, mid); // TODO(greg): WRONG bounds! make them 3-dim
          value_ = Split{std::move(leaf), Empty{}};
          return;
        }
        // Both points right.
        if (old_point_right && new_point_right) {
          Se2Node right = Se2Node(Leaf(leaf.point_));
          right.InsertPoint_(new_point, IncrementAxis(axis), mid, ub); // TODO(greg): WRONG bounds! make them 3-dim
          value_ = Split{Empty{}, std::move(leaf)};
          return;
        }
      },
      // Point inserted to Split will be inserted into either left or right child, depending on its coordinate.
      [new_point, axis, lb, ub](const Split &split) {
        const double mid = 0.5 * (lb + ub);
        const double new_coord = RelevantCoord(axis, new_point);
        const bool new_point_left = new_coord <= mid;
        const bool new_point_right = !new_point_left;
        if (new_point_left) {
          split.left_->InsertPoint_(new_point, IncrementAxis(axis), lb, mid); // TODO(greg): WRONG bounds! make them 3-dim
          return;
        }
        if (new_point_right) {
          split.right_->InsertPoint_(new_point, IncrementAxis(axis), mid, ub); // TODO(greg): WRONG bounds! make them 3-dim
          return;
        }
      }
    }, value_);
}

//Se2Tree::Se2Tree(const double lb, const double ub) : lb_(lb), ub_(ub) {
//
//}
