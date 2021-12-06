#pragma once

#include <cassert>

#include <glm/glm.hpp>
#include <random>

#include "src/space/n_ball.hpp"
#include "src/tagged.hpp"

namespace rrts {

  template <class Point, class Bridge>
  struct Edge {
    Edge(size_t parent_index, const Bridge &bridge) : parent_index_(parent_index), bridge_(bridge) {};
    //double cost_to_go;
    size_t parent_index_;
    //size_t index;
    //Point point;
    Bridge bridge_;
    //double bridge_cost;
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
    Search(Point x_init, Point lb, Point ub, Space space, double eta) : edges_{}, cost_to_go_{}, tree_(lb, ub), space_(space), eta_(eta) {
      cost_to_go_.push_back(0);
      tree_.Insert({0, x_init});
    }

    // index of edge (plus one) is node, Edge type contains parent index.
    std::vector<Edge<Point, Bridge> > edges_;
    std::vector<double> cost_to_go_;

    Tree tree_;
    Space space_;

      // zeta_d is volume of the unit ball in d dimensions.
    static constexpr double zeta_d = VolumeOfNBall(D, 1.0);
    // The proof says gamma_rrtstar should be strictly greater than this number.
    // I set it equal because mu_Xfree is conservatively large.
    // TODO(greg): study if increasing it further helps
    static constexpr double d = static_cast<double>(D);
    double gamma_rrts = pow(2 * (1 + 1 / d), 1/d) * pow(space_.mu_Xfree() / zeta_d, 1 / d);

  private:
    double eta_;

  public:
    StepResult Step() {
      // ********** Sample new point to add to tree. ***********
      // L3 from paper
      const Point x_rand = space_.SampleFree();

      // L4 from paper
      const Tagged<Point> x_nearest = tree_.Nearest(x_rand);

      // L5 from paper
      const Point x_new = space_.Steer(x_nearest.point, x_rand, eta_);

      // L6 from paper
      Bridge nearest_to_new_bridge = space_.FormBridge(x_nearest.point, x_new);
      if (!space_.CollisionFree(nearest_to_new_bridge)) {
        return StepResult::kNotObstacleFree;
      }

      // L7 from paper
      const double cardV = tree_.Cardinality();
      const double radius = fmin(gamma_rrts * pow(log(cardV) / cardV, 1/d), eta_);
      std::vector<Tagged<Point> > X_near = tree_.Near(x_new, radius);

      // ************** Add new node to graph. ****************
      // L8 from paper
      assert(edges_.size() + 1 == cost_to_go_.size());
      size_t new_index = edges_.size() + 1; // 0th node has no bridge, offset by one
      tree_.Insert(Tagged<Point>{new_index, x_new});

      // ********** Find best node to connect new node t. ***********
      // L9 from paper
      Tagged<Point> x_min = x_nearest;
      Bridge b_min = nearest_to_new_bridge;
      double c_min = cost_to_go_.at(x_min.index) + space_.BridgeCost(b_min);

      // L10-12 in paper
      for (const Tagged<Point> x_near : X_near) {
        Bridge near_to_new_bridge = space_.FormBridge(x_near.point, x_new);
        double c = cost_to_go_.at(x_near.index) + space_.BridgeCost(near_to_new_bridge);
        if (space_.CollisionFree(near_to_new_bridge) && c < c_min) {
          x_min = x_near;
          c_min = c;
          b_min = near_to_new_bridge;
        }
      }

      // L13 in paper
      // Insert edge for new node.
      Edge<Point, Bridge> new_edge(x_min.index, b_min);
      edges_.push_back(new_edge);
      cost_to_go_.push_back(c_min);  // TODO(greg): Can/should cost_to_go and edge be combined?

      // ******* Rewire tree: see if any near node is better off going to new node. ******
      // L14-16 in paper
      for (const Tagged<Point> x_near : X_near) {
        if (x_near.index != 0) { // don't rewire root node, it has no parents
          Bridge new_to_near_bridge = space_.FormBridge(x_new, x_near.point);
          const double c = c_min + space_.BridgeCost(new_to_near_bridge);
          if (space_.CollisionFree(new_to_near_bridge) && c < cost_to_go_.at(x_near.index)) {
            // Change parent.
            cost_to_go_.at(x_near.index) = c;
            edges_.at(x_near.index - 1) = Edge<Point, Bridge>{new_index, new_to_near_bridge};
          }
        }
      }

      return StepResult::kSuccess;
    }
  };
}  // namespace rrts
