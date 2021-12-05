#include <cstdlib>             // for EXIT_SUCCESS
#include <iostream>            // for operator<<, basic_ostream, cerr, endl, ostream, cha...

#include "src/rrt_star.hpp"
#include "src/r3_search.hpp"

using Line = rrts::R3::Line;
using Point = rrts::R3::Point;

int run_it() {
  rrts::R3::Point lb = {0, -2, -1};
  rrts::R3::Point ub = {5,  2,  1};
  rrts::R3::Point x_init{0.01, 0, 0};
  std::vector<rrts::R3::Sphere> sphere_obstacles;
  sphere_obstacles.push_back({{4.0, 0.5, 0}, 1});
  sphere_obstacles.push_back({{2.0, -0.4, 0}, 1.15});
  rrts::R3::R3Search search(x_init, lb, ub, sphere_obstacles);
  search.eta_ = 0.15;

  int64_t count = 0;
  while (count<1000) {
    if (search.Step() == rrts::StepResult::kSuccess) {
      count++;
    }
  }
  std::cerr << "finished" << std::endl;
  return EXIT_SUCCESS;
}

int main(int argc __attribute__((unused)), char *argv[] __attribute__((unused))) {
  try {
    return run_it();
  } catch (const std::exception &e) {
    std::cerr << e.what();
  }
}
