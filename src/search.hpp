#pragma once

#include <algorithm>           // for max
#include <cassert>             // for assert
#include <cmath>               // for pow, fmin, log
#include <cstddef>             // for size_t
#include <ext/alloc_traits.h>  // for __alloc_traits<>::value_type
#include <functional>          // for function
#include <set>                 // for set, operator!=, _Rb_tree_const_iterator, _Rb_tree_const...
#include <tuple>               // for get, tuple
#include <vector>              // for vector

#include "src/space/n_ball.hpp"  // for VolumeOfNBall
#include "src/tagged.hpp"        // for Tagged

namespace rrts {

template <class Point, class Bridge>
struct Edge {
  Edge(size_t parent_index, const Bridge &bridge) : parent_index_(parent_index), bridge_(bridge){};
  size_t parent_index_;
  Bridge bridge_;
};

enum class StepResult {
  kSuccess,
  kNotObstacleFree,
};

// Point is the state.
// Bridge is the trajectory between two states.
// D is the dimension of the point.
template <class Point, class Bridge, int D, class Tree, class Space>
class Search {
 public:
  Search(Point x_init, Space space, double eta)
      : space_(space), tree_(space_.Lb(), space_.Ub()), eta_(eta) {
    tree_.Insert(Tagged{0, x_init});
    children_map_.emplace_back(std::set<size_t>{});
  }

 private:
  // index of edge (plus one) is node, Edge type contains parent index.
  std::vector<Edge<Point, Bridge> > edges_{};
  std::vector<std::set<size_t> > children_map_{};

  Space space_;
  Tree tree_;

  // zeta_d is volume of the unit ball in d dimensions.
  static constexpr double kZetaD = VolumeOfNBall(D, 1.0);
  // The proof says gamma_rrtstar should be strictly greater than this number.
  // I set it equal because mu_Xfree is conservatively large.
  // TODO(greg): study if increasing it further helps
  static constexpr double kD = static_cast<double>(D);
  double gamma_rrts_ = pow(2 * (1 + 1 / kD), 1 / kD) * pow(space_.MuXfree() / kZetaD, 1 / kD);

  double eta_;

 public:
  const std::vector<Edge<Point, Bridge> > &Edges() const { return edges_; };
  [[nodiscard]] double Cardinality() const { return tree_.Cardinality(); };

  // Compute cost to go of a node by tracing parents back and adding up all the bridges.
  // O[n]
  double CostToGo(size_t index) {
    double cost = 0;

    while (index != 0) {
      const Edge<Point, Bridge> &edge = edges_.at(index - 1);
      cost += edge.bridge_.TrajectoryCost();
      index = edge.parent_index_;
    }

    return cost;
  }

  // Compute costs to go of all nodes in one pass.
  // O[n].
  std::vector<double> ComputeCostsToGo() {
    std::vector<double> costs_to_go(edges_.size() + 1, 0.0);

    std::function<void(size_t, double)> compute_child_costs =
        [this, &costs_to_go, &compute_child_costs](size_t index, double parent_cost) {
          // iterate through children of this node
          for (const size_t child_index : children_map_.at(index)) {
            // for each child, update it's cost to go
            const Edge<Point, Bridge> &edge = edges_.at(child_index - 1);
            double child_cost_to_go = parent_cost + edge.bridge_.TrajectoryCost();
            costs_to_go.at(child_index) = child_cost_to_go;
            // for each child, update all of its children recursively
            compute_child_costs(child_index, child_cost_to_go);
          }
        };

    compute_child_costs(0, 0.0);
    return costs_to_go;
  }

  StepResult Step() {
    // ********** Sample new point to add to tree. ***********
    // L3 from paper
    const Point x_rand = space_.SampleFree();

    // L4 from paper
    // const Tagged<Point> x_nearest = tree_.Nearest(x_rand);
    // TODO(greg): return bridge to avoid recomputing it
    const std::tuple<Tagged<Point>, Bridge> xb_nearest = tree_.Nearest(
        [&x_rand, this](const Point &p) { return space_.FormBridge(p, x_rand); },
        [&x_rand, this](double distance) { return space_.BoundingBox(x_rand, distance); });
    Tagged<Point> x_nearest = std::get<0>(xb_nearest);

    // L5 from paper
    const std::tuple<Point, Bridge> xb_new =
        space_.Steer(x_nearest.Point(), x_rand,
                     eta_);  // TODO(greg): is this redundant with FormBridge in Nearest?
    const Point &x_new = std::get<0>(xb_new);
    const Bridge &nearest_to_new_bridge = std::get<1>(xb_new);

    // L6 from paper
    if (!space_.CollisionFree(nearest_to_new_bridge)) {
      return StepResult::kNotObstacleFree;
    }

    // L7 from paper
    const double card_v = tree_.Cardinality();
    const double radius = fmin(gamma_rrts_ * pow(log(card_v) / card_v, 1 / kD), eta_);
    std::vector<std::tuple<Tagged<Point>, Bridge> > xb_nears =
        tree_.Near([&x_new, this](const Point &p) { return space_.FormBridge(p, x_new); },
                   space_.BoundingBox(x_new, radius), radius);

    // ************** Add new node to graph. ****************
    // L8 from paper
    assert(edges_.size() + 1 == children_map_.size());
    size_t new_index = edges_.size() + 1;  // 0th node has no bridge, offset by one
    tree_.Insert(Tagged<Point>{new_index, x_new});

    // ********** Find best node to connect new node t. ***********
    // L9 from paper
    Tagged<Point> x_min = x_nearest;
    Bridge b_min = nearest_to_new_bridge;
    double c_min = CostToGo(x_min.Index()) + b_min.TrajectoryCost();

    // L10-12 in paper
    for (const std::tuple<Tagged<Point>, Bridge> xb_near : xb_nears) {
      Tagged<Point> x_near = std::get<0>(xb_near);
      const Bridge &near_to_new_bridge = std::get<1>(xb_near);
      double c = CostToGo(x_near.Index()) + near_to_new_bridge.TrajectoryCost();
      if (space_.CollisionFree(near_to_new_bridge) && c < c_min) {
        x_min = x_near;
        c_min = c;
        b_min = near_to_new_bridge;
      }
    }

    // L13 in paper
    // Insert edge for new node.
    Edge<Point, Bridge> new_edge(x_min.Index(), b_min);
    edges_.push_back(new_edge);
    children_map_.emplace_back(std::set<size_t>{});     // new node initially has no children
    children_map_.at(x_min.Index()).insert(new_index);  // new node is a child of x_min

    // ******* Rewire tree: see if any near node is better off going to new node. ******
    // L14-16 in paper
    for (const std::tuple<Tagged<Point>, Bridge> xb_near : xb_nears) {
      const Tagged<Point> &x_near = std::get<0>(xb_near);
      if (x_near.Index() != 0 &&
          x_near.Index() !=
              x_min.Index()) {  // don't rewire root node, it has no parents, don't rewire z_min
        Bridge new_to_near_bridge = space_.FormBridge(
            x_new, x_near.Point());  // don't try to reuse the above bridge, it goes the wrong way.
                                     // though in reversable cases like R3 and dubins it could
                                     // simply be reversed instead of recomputed......
        const double c = c_min + new_to_near_bridge.TrajectoryCost();
        if (space_.CollisionFree(new_to_near_bridge) && c < CostToGo(x_near.Index())) {
          // Update parent map.
          size_t old_parent_index = edges_.at(x_near.Index() - 1).parent_index_;
          children_map_.at(old_parent_index).erase(x_near.Index());
          children_map_.at(new_index).insert(x_near.Index());

          // Update edge.
          edges_.at(x_near.Index() - 1) = Edge<Point, Bridge>{new_index, new_to_near_bridge};
        }
      }
    }

    return StepResult::kSuccess;
  }
};
}  // namespace rrts
