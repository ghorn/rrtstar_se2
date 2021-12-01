#pragma once

#include <cassert>

#include <random>
#include <glm/glm.hpp>

#include "src/se2_point.hpp"
#include "src/se2_tree.hpp"
//#include "src/r2_point.hpp"
//#include "src/r2_tree.hpp"

namespace rrts {

//template <class Point, class Line, class Tree>
//class Space {
//  
//}
  struct Sphere {
    glm::vec3 center;
    double radius;
  };
  
class Algorithm {
public:
  Algorithm(Se2::Point initial_point, double eta) : tree_(initial_point), eta_(eta) {
    sphere_obstacles.push_back({{0.5, 0.5, 0}, 0.5});
    sphere_obstacles.push_back({{0.25, 0.75, 0}, 0.3});
  };
  ~Algorithm(){};

  enum class StepResult {
    kSuccess,
    kNotObstacleFree,
  };

  StepResult Step();
  std::vector<Sphere> sphere_obstacles;

private:
  Se2::Tree tree_;
  const double eta_;
  // zeta_d is volume of the unit ball in d dimensions.
  const double zeta_d = VolumeOfNBall(d_, 1.0);
  const int d_ = 3;
  const double d = static_cast<double>(d_);
  double mu_Xfree = 2*M_PI; // TODO(greg): adapt for obstacles and for non-(0,1),(0,1),(-pi,pi)
  // The proof says gamma_rrtstar should be strictly greater than this number.
  // I set it equal because mu_Xfree is conservatively large.
  const double gamma_rrts = pow(2 * (1 + 1 / d), 1/d) * pow(mu_Xfree / zeta_d, 1 / d);

  Se2::Point SampleFree();
  std::vector<Se2::Point> Near(Se2::Point);
  Se2::Point Steer(Se2::Point, Se2::Point);
  bool CollisionFree(Se2::Point, Se2::Point);
  void InsertPoint(Se2::Point);
  void Increment();

  std::mt19937_64 rng_engine;
  std::uniform_real_distribution<double> uniform_distribution;

  static double VolumeOfNBall(int n, double radius);
};

}
