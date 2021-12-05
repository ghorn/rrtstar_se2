#pragma once

#include <cmath> // M_PI
#include <variant>
#include <memory>
#include <string>
#include <iostream>
#include <vector>
#include <glm/glm.hpp>

#include "src/r3_point.hpp"
#include "src/tree/base.hpp"
#include "src/tagged.hpp"

namespace rrts {
namespace tree {

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
    Leaf(Tagged<R3::Point> point) : point_(point) {};
    Tagged<R3::Point> point_;
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
  void InsertPoint_(const Tagged<R3::Point> new_point,
                    const int32_t axis,
                    const R3::Point tree_lb,
                    const R3::Point tree_ub);

  struct SearchParams {
    double radius;
    double radius_squared;
    R3::Point test_point;
    R3::Point bounding_box_lb;
    R3::Point bounding_box_ub;
  };
  void Near_(std::vector<Tagged<R3::Point>> * const close_points,
             const SearchParams &params,
             const int32_t axis,
             const R3::Point tree_lb,
             const R3::Point tree_ub) const;

  void Nearest_(Tagged<R3::Point> * const closest_point,
                double * const closest_point_distance,
                double * const closest_point_distance_squared,
                const R3::Point test_point,
                const int32_t axis,
                const R3::Point tree_lb,
                const R3::Point tree_ub) const;
  static const int32_t tree_dimension_ = 3; // TODO(greg): don't hardcode!!!
  void Draw(const std::string &prefix) const;
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

  void SearchNearestLeft(const Node::Split &split,
                         Tagged<R3::Point> * const closest_point,
                         double * const closest_point_distance,
                         double * const closest_point_distance_squared,
                         const R3::Point test_point,
                         const int32_t axis,
                         const R3::Point tree_lb,
                         const R3::Point tree_ub) const;
  void SearchNearestRight(const Node::Split &split,
                          Tagged<R3::Point> * const closest_point,
                          double * const closest_point_distance,
                          double * const closest_point_distance_squared,
                          const R3::Point test_point,
                          const int32_t axis,
                          const R3::Point tree_lb,
                          const R3::Point tree_ub) const;
};


// Just an R3::Node with some meta-information. This will be the user-facing interface.
// Required meta-information is the upper and lower bounds of the data space.
// Future optional meta-information could be number of nodes, max depth, etc.
struct Fast {
  Fast(R3::Point lb, R3::Point ub) : lb_(lb), ub_(ub), root_(Node::Empty{}) {};
  ~Fast(){};

  int64_t num_nodes_ = 0;

  void Draw() const {root_.Draw("");}
  void Insert(const Tagged<R3::Point> new_point) {
    //std::cerr << "inserting point " << new_point.index << std::endl;
    num_nodes_++;
    root_.InsertPoint_(new_point, 0, lb_, ub_);
  }

  const R3::Point lb_;
  const R3::Point ub_;

  Tagged<R3::Point> Nearest(const R3::Point test_point) const {
    // Test point will be overwritten because closest point distance is so high.
    // This is potentially violated if bounds are anything besides (0, 1).
    Tagged<R3::Point> closest_point = {2222222222222222, test_point};
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
  std::vector<Tagged<R3::Point> > Near(const R3::Point &test_point,
                                   const double radius) const {
    std::vector<Tagged<R3::Point>> close_points;

    Node::SearchParams search_params;
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

  double Cardinality() const {
    // TODO(greg): this might be wrong
    return static_cast<double>(num_nodes_);
  };
private:
  Node root_;
};
} // namespace fast
} // namespace rrts
