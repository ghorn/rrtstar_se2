#include <algorithm>  // for max
#include <cstdint>    // for int64_t
#include <cstdlib>    // for EXIT_SUCCESS
#include <iostream>   // for operator<<, cerr, endl, ostream, basic_ostream
#include <tuple>      // for tuple
#include <vector>     // for vector

#include "src/search.hpp"     // for Search, StepResult, StepResult::kSuccess
#include "src/space/r3.hpp"   // for Sphere, Line, Point, R3
#include "src/tree/fast.hpp"  // for Fast

using Line = rrts::space::r3::Line;
using Point = rrts::space::r3::Point;
using Sphere = rrts::space::r3::Sphere;
using Tree = rrts::tree::Fast<Point, Line, 3>;
using Space = rrts::space::r3::R3;

int RunIt() {
  Point lb = {0, -2, -1};
  Point ub = {5, 2, 1};
  Point x_init{0.01, 0, 0};
  std::vector<Sphere> sphere_obstacles;
  sphere_obstacles.push_back({{4.0, 0.5, 0}, 1});
  sphere_obstacles.push_back({{2.0, -0.4, 0}, 1.15});
  Space r3_space(lb, ub, sphere_obstacles);
  rrts::Search<Point, Line, 3, Tree, Space> search(x_init, r3_space, 0.15);

  int64_t count = 0;
  while (count < 1000) {
    if (search.Step() == rrts::StepResult::kSuccess) {
      count++;
    }
  }
  std::cerr << "finished" << std::endl;
  return EXIT_SUCCESS;
}

int main(int argc __attribute__((unused)), char *argv[] __attribute__((unused))) {
  try {
    return RunIt();
  } catch (const std::exception &e) {
    std::cerr << e.what();
  }
}
