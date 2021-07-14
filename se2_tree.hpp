#pragma once

#include <variant>
#include <memory>
#include <utility>

enum class Se2Axis {
  kX,
  kY,
  kTheta
};

Se2Axis IncrementAxis(const Se2Axis axis);

struct Se2Point {
  double x;
  double y;
  double theta;
};

// 3-dimensional binary tree. Not a K-D tree, I forget the real name for it.
// Each branch in the tree divides a spatial dimension in half. Each layer of the tree
// rotates the axis being divided.
struct Se2Node {
  // No data points.
  struct Empty {
    // TODO(greg): std::monostate
  };

  // A single data point.
  struct Leaf {
    Leaf(Se2Point point) : point_(point) {};
    ~Leaf();
    Se2Point point_;
  };

  // Two or more data points. Not necessarily balanced - they could both be in the
  // left branch with nothing in the right branch, for instance.
  struct Split {
    // Forwarding constructor which will accept arguments of any type which can be implicitly
    // converted to an `Se2Node`
    template <typename L, typename R>
    Split(L &&l, R &&r) :
        left_{std::make_unique<Se2Node>(Se2Node{std::forward<L>(l)})},
        right_{std::make_unique<Se2Node>(Se2Node{std::forward<R>(r)})} {}

    // // Forwarding constructor which will accept arguments of any type which can be implicitly
    // // converted to an `Se2Node`
    // template <typename L, typename R>
    // Split(L &&l, R &&r) :
    //   left_{std::make_unique<Se2Node>(std::forward<L>(l))},
    //   right_{std::make_unique<Se2Node>(std::forward<R>(r))} {}

    //Split(Se2Node &&left, Se2Node &&right) :
    //  left_{std::make_unique<Se2Node>(std::move(left))},
    //  right_{std::make_unique<Se2Node>(std::move(right))} {}

    //Split(std::unique_ptr<Se2Node> left, std::unique_ptr<Se2Node> right) :
    //  left_{std::move(left)},
    //  right_{std::move(right)} {}
    std::unique_ptr<Se2Node> left_;
    std::unique_ptr<Se2Node> right_;
  };

  //Se2Node(Empty empty) : value_(empty){}
  //Se2Node(Leaf leaf) : value_(leaf){}
  //Se2Node(Split split) :value_(std::move(split)){}
  explicit Se2Node(const Empty &e) : value_{e} {}
  explicit Se2Node(const Leaf &l) : value_{l} {}
  explicit Se2Node(Split &&s) : value_{std::move(s)} {}
  Se2Node &operator=(const Empty &e) { value_ = e; return *this; };
  Se2Node &operator=(const Leaf &l) { value_ = l; return *this; };
  Se2Node &operator=(Split &&s) { value_ = std::move(s); return *this; };
  //Se2Node(const Empty &e) : value_{e} {}
  //Se2Node(const Leaf &l) : value_{l} {}
  //Se2Node(Split &&s) : value_{std::move(s)} {}
  //Se2Node &operator=(const Empty &e) { value_ = e; return *this; };
  //Se2Node &operator=(const Leaf &l) { value_ = l; return *this; };
  //Se2Node &operator=(Split &&s) { value_ = std::move(s); return *this; };

  // The node data.
  std::variant<Empty, Leaf, Split> value_;

  // Insert a new point into the node.
  void InsertPoint_(const Se2Point new_point,
                    const Se2Axis axis,
                    const double lb,
                    const double ub);
};


//// Just an Se2Node with some meta-information. This will be the user-facing interface.
//// Required meta-information is the upper and lower bounds of the data space.
//// Future optional meta-information could be number of nodes, max depth, etc.
//struct Se2Tree {
//  Se2Tree(const double global_lb, const double global_ub) : global_lb_(global_lb), global_ub_(global_ub) {};
//  ~Se2Tree();
//
//private:
//  const double global_lb_;
//  const double global_ub_;
//
//  Se2Node node;
//};
