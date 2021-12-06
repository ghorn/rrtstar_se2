#include <cstdlib>             // for EXIT_SUCCESS
#include <iostream>            // for operator<<, basic_ostream, cerr, endl, ostream, cha...

#include "src/rrt_star.hpp"
#include "src/space/r3.hpp"
#include "src/tree/fast.hpp"
#include "src/tree/naive.hpp"

using namespace rrts;
using Line = space::r3::Line;
using Point = space::r3::Point;
using Sphere = space::r3::Sphere;

int run_it() {
  Point lb = {0, -2, -1};
  Point ub = {5,  2,  1};
  Point x_init{0.01, 0, 0};
  std::vector<Sphere> sphere_obstacles;
  sphere_obstacles.push_back({{4.0, 0.5, 0}, 1});
  sphere_obstacles.push_back({{2.0, -0.4, 0}, 1.15});
  space::r3::R3 r3_space(lb, ub, sphere_obstacles);
  Search<Point, Line, 3, tree::Fast<Point, 3>, space::r3::R3> search(x_init, lb, ub, r3_space, 0.15);

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
