#pragma once

#include <cassert>

#include <random>
#include <glm/glm.hpp>

namespace rrts {

  double VolumeOfNBall(int n, double radius);

  template <class Point>
  struct Tagged {
    size_t index;
    Point point;
  };

  template <class Point, class Bridge>
  struct Edge {
    //double cost_to_go;
    size_t parent_index;
    //size_t index;
    //Point point;
    Bridge bridge;
    //double bridge_cost;
  };


  enum class StepResult {
    kSuccess,
    kNotObstacleFree,
  };

  // Point is the state.
  // Bridge is the trajectory between two states.
  // D is the dimension of the point.
  template <class Point, class Bridge, int D>
  class Search {
  public:
    Search() {
      cost_to_go_.push_back(0);
    }
    virtual ~Search() = default;

    StepResult Step() {
      // ********** Sample new point to add to tree. ***********
      // L3 from paper
      const Point x_rand = SampleFree();

      // L4 from paper
      const Tagged<Point> x_nearest = Nearest(x_rand);

      // L5 from paper
      const Point x_new = Steer(x_nearest.point, x_rand);

      // L6 from paper
      Bridge nearest_to_new_bridge = FormBridge(x_nearest.point, x_new);
      if (!CollisionFree(nearest_to_new_bridge)) {
        return StepResult::kNotObstacleFree;
      }

      // L7 from paper
      const double radius = fmin(gamma_rrts() * pow(log(CardV()) / CardV(), 1/d), eta_);
      std::vector<Tagged<Point> > X_near = Near(x_new, radius);

      // ************** Add new node to graph. ****************
      // L8 from paper
      assert(edges_.size() + 1 == cost_to_go_.size());
      size_t new_index = edges_.size() + 1; // 0th node has no bridge, offset by one
      InsertPoint(Tagged<Point>{new_index, x_new});

      // ********** Find best node to connect new node t. ***********
      // L9 from paper
      Tagged<Point> x_min = x_nearest;
      Bridge b_min = nearest_to_new_bridge;
      double c_min = cost_to_go_.at(x_min.index) + BridgeCost(b_min);

      // L10-12 in paper
      for (const Tagged<Point> x_near : X_near) {
        Bridge near_to_new_bridge = FormBridge(x_near.point, x_new);
        double c = cost_to_go_.at(x_near.index) + BridgeCost(near_to_new_bridge);
        if (CollisionFree(near_to_new_bridge) && c < c_min) {
          x_min = x_near;
          c_min = c;
          b_min = near_to_new_bridge;
        }
      }

      // L13 in paper
      // Insert edge for new node.
      Edge<Point, Bridge> new_edge;
      new_edge.bridge = b_min;
      new_edge.parent_index = x_min.index;
      edges_.push_back(new_edge);
      cost_to_go_.push_back(c_min);  // TODO(greg): Can/should cost_to_go and edge be combined?

      // ******* Rewire tree: see if any near node is better off going to new node. ******
      // L14-16 in paper
      for (const Tagged<Point> x_near : X_near) {
        if (x_near.index != 0) { // don't rewire root node, it has no parents
          Bridge new_to_near_bridge = FormBridge(x_new, x_near.point);
          const double c = c_min + BridgeCost(new_to_near_bridge);
          if (CollisionFree(new_to_near_bridge) && c < cost_to_go_.at(x_near.index)) {
            // Change parent.
            cost_to_go_.at(x_near.index) = c;
            edges_.at(x_near.index - 1) = Edge<Point, Bridge>{new_index, new_to_near_bridge};
          }
        }
      }

      return StepResult::kSuccess;
    }
  
    double eta_ = 0.1;
    // index of edge (plus one) is node, Edge type contains parent index.
    std::vector<Edge<Point, Bridge> > edges_;
    std::vector<double> cost_to_go_;
    
  private:
    //Se2::Tree tree_;
    // zeta_d is volume of the unit ball in d dimensions.
    const double zeta_d = VolumeOfNBall(D, 1.0);
    const double d = static_cast<double>(D);
    virtual double mu_Xfree() = 0; //  = 2*M_PI; // TODO(greg): adapt for obstacles and for non-(0,1),(0,1),(-pi,pi)
    // The proof says gamma_rrtstar should be strictly greater than this number.
    // I set it equal because mu_Xfree is conservatively large.
    double gamma_rrts() {
      // compute on first invocation to avoid using virtual mu_XFree function in constructor.
      static double ret = pow(2 * (1 + 1 / d), 1/d) * pow(mu_Xfree() / zeta_d, 1 / d);
      return ret;
    }

    // Bridging point to point
    virtual double BridgeCost(const Bridge &) = 0;
    virtual Bridge FormBridge(const Point&, const Point&) = 0;
    virtual bool CollisionFree(const Bridge&) = 0;

    virtual double CardV() = 0;
    virtual Point SampleFree() = 0;
    virtual std::vector<Tagged<Point> > Near(const Point&, double) = 0;
    virtual Tagged<Point> Nearest(const Point&) = 0;
    virtual Point Steer(const Point&, const Point&) = 0;
    virtual void InsertPoint(const Tagged<Point>&) = 0;

  public:
    std::mt19937_64 rng_engine;
    std::uniform_real_distribution<double> uniform_distribution;
  };
}
