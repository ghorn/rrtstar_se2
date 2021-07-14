#pragma once

#include <variant>
#include <memory>
#include <utility>

#include "se2_point.hpp"

namespace Se2 {

// 3-dimensional binary tree. Not a K-D tree, I forget the real name for it.
// Each branch in the tree divides a spatial dimension in half. Each layer of the tree
// rotates the axis being divided.
struct Node {
  // No data points.
  struct Empty {
    // TODO(greg): std::monostate
  };

  // A single data point.
  struct Leaf {
    Leaf(Point point) : point_(point) {};
    Point point_;
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
  };

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
  void InsertPoint_(const Point new_point,
                    const Axis axis,
                    const Point lb,
                    const Point ub);

private:
  // The node data.
  std::variant<Empty, Leaf, Split> value_;
};


//// Just an Se2::Node with some meta-information. This will be the user-facing interface.
//// Required meta-information is the upper and lower bounds of the data space.
//// Future optional meta-information could be number of nodes, max depth, etc.
//struct Tree {
//  Tree(const double global_lb, const double global_ub) : global_lb_(global_lb), global_ub_(global_ub) {};
//  ~Tree();
//
//private:
//  const double global_lb_;
//  const double global_ub_;
//
//  Node node;
//};
} // namespace Se2
