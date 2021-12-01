#pragma once

#include <cmath> // M_PI
#include <variant>
#include <memory>
#include <vector>

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
                    const Point tree_lb,
                    const Point tree_ub);

  struct SearchParams {
    double radius;
    double radius_squared;
    Point test_point;
    Point bounding_box_lb;
    Point bounding_box_ub;
  };
  void PointsWithinRadiusOf_(std::vector<Point> * const close_points,
                             const SearchParams &params,
                             const Axis axis,
                             const Point tree_lb,
                             const Point tree_ub);
  
  void Nearest_(Point * const closest_point,
                double * const closest_point_distance,
                double * const closest_point_distance_squared,
                const Point test_point,
                const Axis axis,
                const Point tree_lb,
                const Point tree_ub);

private:
  // The node data.
  std::variant<Empty, Leaf, Split> value_;
};


// Just an Se2::Node with some meta-information. This will be the user-facing interface.
// Required meta-information is the upper and lower bounds of the data space.
// Future optional meta-information could be number of nodes, max depth, etc.
struct Tree {
  Tree(Se2::Point x_init) : root_(x_init) {};
  ~Tree(){};

  int64_t num_nodes_ = 1;
  void Insert(const Point new_point) {
    num_nodes_++;
    root_.InsertPoint_(new_point, Axis::kX, lb_, ub_);
  }

  // TODO(greg): different bounds than 0?
  const Se2::Point lb_ = {0, 0, -M_PI};
  const Se2::Point ub_ = {1, 1,  M_PI};

  Se2::Point Nearest(const Point test_point) {
    // Test point will be overwritten because closest point distance is so high.
    // This is potentially violated if bounds are anything besides (0, 1).
    Se2::Point closest_point = test_point;
    double closest_point_distance = 1e9;
    double closest_point_distance_squared = 1e18;

    root_.Nearest_(&closest_point,
                   &closest_point_distance,
                   &closest_point_distance_squared,
                   test_point,
                   Axis::kX,
                   lb_,
                   ub_);

    return closest_point;
  }

  std::vector<Point> PointsWithinRadiusOf(const Se2::Point &test_point,
                                          const double radius) {
    std::vector<Point> close_points;
    
    Node::SearchParams search_params;
    search_params.radius = radius;
    search_params.radius_squared = radius*radius;
    search_params.test_point = test_point;
    
    search_params.bounding_box_lb = test_point;
    search_params.bounding_box_lb.x -= radius;
    search_params.bounding_box_lb.y -= radius;
    search_params.bounding_box_lb.theta -= radius;
    
    search_params.bounding_box_ub = test_point;
    search_params.bounding_box_ub.x += radius;
    search_params.bounding_box_ub.y += radius;
    search_params.bounding_box_ub.theta += radius;
    
    root_.PointsWithinRadiusOf_(&close_points,
                                search_params,
                                Axis::kX,
                                lb_,
                                ub_);
    return close_points;
  }
  
private:
  Node root_;
};
} // namespace Se2
