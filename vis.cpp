#include <bits/exception.h>  // for exception
#include <sys/types.h>       // for key_t, uint

#include <algorithm>           // for copy, max
#include <chrono>              // for operator""s, chrono_literals
#include <cstdio>              // for fprintf, stderr
#include <cstdlib>             // for EXIT_SUCCESS
#include <functional>          // for function
#include <iostream>            // for operator<<, basic_ostream, cerr, endl, ostream, cha...
#include <mutex>               // for mutex, lock_guard
#include <optional>            // for optional, nullopt
#include <queue>               // for queue
#include <thread>              // for sleep_for, thread
#include <vector>              // for vector

#include "bb3d/opengl_context.hpp"    // for Window
#include "bb3d/shader/lines.hpp"
#include "bb3d/shader/colorlines.hpp"

#include "src/rrt_star.hpp"
#include "src/r3_search.hpp"

using Line = rrts::R3::Line;
using Point = rrts::R3::Point;

static void UpdateBridgeLines(bb3d::ColorLines &lines,
                              const std::vector<double> &cost_to_go,
                              const std::vector<rrts::Edge<Point, Line> > &edges) {
  double max_cost_to_go = 0;
  for (double c : cost_to_go) {
    max_cost_to_go = std::max(max_cost_to_go, c);
  }

  std::vector<std::vector<bb3d::ColoredVec3> > bridges;
  size_t node_index = 1;
  for (const rrts::Edge<Point, Line> &edge: edges) {
    bb3d::ColoredVec3 cv0;
    bb3d::ColoredVec3 cv1;
    cv0.position = edge.bridge.p0;
    cv1.position = edge.bridge.p1;
    const double ctg0 = cost_to_go.at(edge.parent_index) / max_cost_to_go;
    const double ctg1 = cost_to_go.at(node_index) / max_cost_to_go;
    cv0.color = {ctg0, 0, 1 - ctg0, 1};
    cv1.color = {ctg1, 0, 1 - ctg1, 1};
    bridges.push_back({cv0, cv1});
    node_index++;
  }
  lines.Update(bridges);
}

static void UpdatePoints(bb3d::Lines &lines,
                         const std::vector<rrts::Tagged<Point> > &tagged_points) {
  std::vector<glm::vec3> points;
  for (const rrts::Tagged<Point> &x: tagged_points) {
    points.push_back(x.point);
  }
  std::vector<std::vector<glm::vec3> > point_strip{points};
  lines.Update(point_strip);
}

int run_it(char *argv0) {
  rrts::R3::Point lb = {0, -2, -1};
  rrts::R3::Point ub = {5,  2,  1};
  rrts::R3::Point x_init{0.01, 0, 0};
  std::vector<rrts::R3::Sphere> sphere_obstacles;
  sphere_obstacles.push_back({{4.0, 0.5, 0}, 1});
  sphere_obstacles.push_back({{2.0, -0.5, 0}, 1.25});
  rrts::R3::R3Search search(x_init, lb, ub, sphere_obstacles);
  search.eta_ = 0.2;

  bb3d::Window window(argv0);
  window.SetCameraFocus({0, 0, 0});
  window.SetCameraAzimuthDeg(-45);

  bb3d::Lines points;
  points.SetPointSize(2);
  bb3d::ColorLines bridge_lines;
  bb3d::Lines sphere_lines;
  bb3d::Lines bounding_box_lines;

  // sphere lines
  {
    std::vector<std::vector<glm::vec3> > sphere_axes;
    for (rrts::R3::Sphere &sphere : search.sphere_obstacles_) {
      glm::vec3 dx = {sphere.radius, 0, 0};
      glm::vec3 dy = {0, sphere.radius, 0};
      glm::vec3 dz = {0, 0, sphere.radius};
      const glm::vec3 &center = sphere.center;
      sphere_axes.push_back({center + dx, center - dx});
      sphere_axes.push_back({center + dy, center - dy});
      sphere_axes.push_back({center + dz, center - dz});
    }
    sphere_lines.Update(sphere_axes);
  }

  // bounding box lines
  {
    std::vector<std::vector<glm::vec3> > bb_lines;
    bb_lines.push_back({{lb.x, lb.y, lb.z},
                        {ub.x, lb.y, lb.z}});
    bb_lines.push_back({{lb.x, lb.y, ub.z},
                        {ub.x, lb.y, ub.z}});
    bb_lines.push_back({{lb.x, ub.y, lb.z},
                        {ub.x, ub.y, lb.z}});
    bb_lines.push_back({{lb.x, ub.y, ub.z},
                        {ub.x, ub.y, ub.z}});

    bb_lines.push_back({{lb.x, lb.y, lb.z},
                        {lb.x, ub.y, lb.z}});
    bb_lines.push_back({{lb.x, lb.y, ub.z},
                        {lb.x, ub.y, ub.z}});
    bb_lines.push_back({{ub.x, lb.y, lb.z},
                        {ub.x, ub.y, lb.z}});
    bb_lines.push_back({{ub.x, lb.y, ub.z},
                        {ub.x, ub.y, ub.z}});

    bb_lines.push_back({{lb.x, lb.y, lb.z},
                        {lb.x, lb.y, ub.z}});
    bb_lines.push_back({{lb.x, ub.y, lb.z},
                        {lb.x, ub.y, ub.z}});
    bb_lines.push_back({{ub.x, lb.y, lb.z},
                        {ub.x, lb.y, ub.z}});
    bb_lines.push_back({{ub.x, ub.y, lb.z},
                        {ub.x, ub.y, ub.z}});
    bounding_box_lines.Update(bb_lines);
  }

  bool pause = false;
  std::function<void(key_t)> handle_keypress = [&window, &pause](key_t key) {
    if (key == 80) {
      std::cout << "pausing" << std::endl;
      pause = !pause;
    };
  };

  int64_t count = 0;
  std::mutex step_mutex;
  std::function<void()> update_visualization = [&window, &search, &count, &bridge_lines, &points, &step_mutex]() {
    const std::lock_guard<std::mutex> lock(step_mutex);
    UpdateBridgeLines(bridge_lines, search.cost_to_go_, search.edges_);
    UpdatePoints(points, search.tree_.points_);
  };

  std::function<void(const glm::mat4 &, const glm::mat4 &)> draw_visualization =
    [&sphere_lines, &bridge_lines, &bounding_box_lines, &points](const glm::mat4 &view, const glm::mat4 &proj) {
      glm::vec4 sphere_color = {0, 1, 0, 0.5};
      glm::vec4 point_color = {1, 1, 1, 0.3};
      glm::vec4 bb_color = {1, 1, 1, 0.5};
      sphere_lines.Draw(view, proj, sphere_color, GL_LINES);
      bridge_lines.Draw(view, proj, GL_LINES);
      points.Draw(view, proj, point_color, GL_POINTS);
      bounding_box_lines.Draw(view, proj, bb_color, GL_LINES);
    };

  // it's theadn' time
  std::thread thread_object([&count, &pause, &step_mutex, &search]() {
    while (count<10000) {
      if (!pause) {
        const std::lock_guard<std::mutex> lock(step_mutex);
        if (search.Step() == rrts::StepResult::kSuccess) {
          count++;
        }
      }
      using namespace std::chrono_literals;
      std::this_thread::sleep_for(1ms);
    }
    std::cerr << "finished" << std::endl;
  });

  window.Run(handle_keypress, update_visualization, draw_visualization);

  return EXIT_SUCCESS;
}

int main(int argc __attribute__((unused)), char *argv[]) {
  try {
    run_it(argv[0]);
  } catch (const std::exception &e) {
    std::cerr << e.what();
  }
}
