#include "src/se2_tree.hpp"

#include <random>

int main() {
  Se2::Node tree{Se2::Node::Empty{}};
  const Se2::Point lb = {0, 0, -M_PI};
  const Se2::Point ub = {1, 1, M_PI};
  std::mt19937_64 rng_engine;
  std::uniform_real_distribution<double> uniform_distribution;

  for (int k=0; k<100; k++) {
    const double x = uniform_distribution(rng_engine);
    const double y = uniform_distribution(rng_engine);
    const double theta = uniform_distribution(rng_engine);
    Se2::Point point {x, y, theta};
    tree.InsertPoint_(point, Se2::Axis::kX, lb, ub);
  }
}
