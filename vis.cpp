#include <bits/exception.h>  // for exception
#include <sys/types.h>       // for key_t, uint

#include <algorithm>           // for copy, max
#include <chrono>              // for operator""s, chrono_literals
#include <cstdio>              // for fprintf, stderr
#include <cstdlib>             // for EXIT_SUCCESS
#include <eigen3/Eigen/Dense>  // for Matrix, DenseCoeffsBase
#include <functional>          // for function
#include <iostream>            // for operator<<, basic_ostream, cerr, endl, ostream, cha...
#include <mutex>               // for mutex, lock_guard
#include <optional>            // for optional, nullopt
#include <queue>               // for queue
#include <thread>              // for sleep_for, thread
#include <vector>              // for vector

//#include "bb3d/assert.hpp"            // for ASSERT
#include "bb3d/opengl_context.hpp"    // for Window

#include "src/rrt_star.hpp"

int run_it(char *argv0) {
  Se2::Point x_init{0.1, 0.1, 30*3.14/180};
  const double eta = 0.2;
  rrts::Algorithm algorithm(x_init, eta);

  for (int k=0; k<10; k++) {
    algorithm.Step();
  }

  std::exit(EXIT_SUCCESS);
  // Boilerplate
  bb3d::Window window(argv0);

//// problem
//ProblemVisualization visualization;
//visualization.Update<NU_OBJ, NV_OBJ, NU_VIS, NU_VIS>(Backboard<NX, NY>::Initialize());
//
//// it's theadn' time
//SharedData shared_data;
//std::thread thread_object([&shared_data]() { Optimize(shared_data); });
//
//std::function<void(key_t)> handle_keypress = [&visualization](key_t key) {
//  visualization.HandleKeyPress(key);
//};
//
//std::function<void()> update_visualization = [&visualization, &shared_data]() {
//  // drain the queue
//  std::optional<Eigen::Matrix<double, NX, NY>> dvs = std::nullopt;
//  {
//    const std::lock_guard<std::mutex> lock(shared_data.queue_mutex);
//    while (shared_data.dvs_queue.size() > 1) {
//      shared_data.dvs_queue.pop();
//    }
//    if (!shared_data.dvs_queue.empty()) {
//      dvs = shared_data.dvs_queue.front();
//      shared_data.dvs_queue.pop();
//    }
//  }
//  if (dvs) {
//    visualization.Update<NU_OBJ, NV_OBJ, NU_VIS, NU_VIS>(
//        Backboard<NX, NY>::ToControlPoints(*dvs));
//  }
//};
//
//std::function<void(const glm::mat4 &, const glm::mat4 &)> draw_visualization =
//    [&visualization](const glm::mat4 &view, const glm::mat4 &proj) {
//      visualization.Draw(view, proj);
//    };
//
//window.Run(handle_keypress, update_visualization, draw_visualization);

  return EXIT_SUCCESS;
}

int main(int argc __attribute__((unused)), char *argv[]) {
  try {
    run_it(argv[0]);
  } catch (const std::exception &e) {
    std::cerr << e.what();
  }
}
