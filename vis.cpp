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
#include <sstream>

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
  // Find max cost to go in order to scale lines
  double max_cost_to_go = 0;
  for (double c : cost_to_go) {
    max_cost_to_go = std::max(max_cost_to_go, c);
  }

  // Draw all bridges
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

static bool InGoalRegion(const glm::vec3 &p) {
  if (p.x < 4.5F) {
    return false;
  }
  if (p.y < -1.75F) {
    return false;
  }
  if (p.y > -1.25F) {
    return false;
  }
  if (p.z > 0.25F) {
    return false;
  }
  if (p.z < -0.25F) {
    return false;
  }
  return true;
}

static double UpdateGoalLine(bb3d::Lines &goal_line,
                             const std::vector<double> &cost_to_go,
                             const std::vector<rrts::Edge<Point, Line> > &edges) {
  double min_cost_to_go = 0;
  size_t winner_index = 0;
  bool got_winner = false;
  size_t index = 1;
  // best cost to go in a goal region
  for (const rrts::Edge<Point, Line> &edge : edges) {
    if (InGoalRegion(edge.bridge.p1) && (cost_to_go.at(index) < min_cost_to_go || !got_winner)) {
      winner_index = index;
      got_winner = true;
      min_cost_to_go = cost_to_go.at(index);
    }
    index++;
  }

  // trace back route from winner
  if (got_winner) {
    std::vector<glm::vec3> winning_route;
    size_t head = winner_index;

    int num_links = 0;
    while (head != 0) {
      num_links++;
      rrts::Edge<Point, Line> edge = edges.at(head - 1);
      glm::vec3 p = edge.bridge.p1;
      p.z -= 0.02F;
      //std::cerr << edge.bridge.p1.x << " " << edge.bridge.p1.y << " " << edge.bridge.p1.z << " " << std::endl;
      winning_route.push_back(p);
      head = edge.parent_index;

      if (head == 0) {
        glm::vec3 pf = edge.bridge.p0;
        pf.z -= 0.02F;
        winning_route.push_back(pf);
      }
    }
    std::vector<std::vector<glm::vec3> > segments;
    segments.push_back(winning_route);

    goal_line.Update(segments);

    return min_cost_to_go;
  }

  return -1;
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
  sphere_obstacles.push_back({{2.0, -0.4, 0}, 1.15});
  rrts::R3::R3Search search(x_init, lb, ub, sphere_obstacles);
  search.eta_ = 0.15;

  bb3d::Window window(argv0);
  window.SetCameraFocus({0, 0, 0});
  window.SetCameraAzimuthDeg(-45);

  bb3d::Lines points;
  points.SetPointSize(2);
  bb3d::ColorLines bridge_lines;
  bb3d::Lines goal_line;
  bb3d::Lines sphere_lines;
  bb3d::Lines bounding_box_lines;
  bb3d::Freetype textbox(18);

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
  std::mutex search_mutex;
  double min_cost_to_go = -1;
  std::function<void()> update_visualization = [&window, &search, &count, &bridge_lines, &goal_line, &points, &search_mutex, &min_cost_to_go]() {
    const std::lock_guard<std::mutex> lock(search_mutex);
    UpdateBridgeLines(bridge_lines, search.cost_to_go_, search.edges_);
    min_cost_to_go = UpdateGoalLine(goal_line, search.cost_to_go_, search.edges_);
    UpdatePoints(points, search.naive_tree_.points_);
  };

  std::function<void(const glm::mat4 &, const glm::mat4 &)> draw_visualization =
    [&sphere_lines, &bridge_lines, &bounding_box_lines, &goal_line, &points, &window, &textbox, &search, &search_mutex, &min_cost_to_go](
        const glm::mat4 &view, const glm::mat4 &proj) {
      glm::vec4 sphere_color = {0, 1, 0, 0.5};
      glm::vec4 point_color = {1, 1, 1, 0.3};
      glm::vec4 bb_color = {1, 1, 1, 0.5};
      glm::vec4 goal_color = {1, 1, 1, 1};
      sphere_lines.Draw(view, proj, sphere_color, GL_LINES);
      bridge_lines.Draw(view, proj, GL_LINES);
      goal_line.Draw(view, proj, goal_color, GL_LINE_STRIP);
      points.Draw(view, proj, point_color, GL_POINTS);
      bounding_box_lines.Draw(view, proj, bb_color, GL_LINES);

      // some text
      std::stringstream message;
      {
        const std::lock_guard<std::mutex> lock(search_mutex);
        message << search.Cardinality() << " nodes in tree";
        if (min_cost_to_go > 0) {
          message << " optimal distance is " << min_cost_to_go;
        }
      }
      window.RenderText(textbox, message.str(), 25.0F, 25.0F, glm::vec3(1, 1, 1));

    };

  // it's theadn' time
  std::thread thread_object([&count, &pause, &search_mutex, &search]() {
    using namespace std::chrono_literals;
    std::this_thread::sleep_for(800ms);
    while (count<50000) {
      if (!pause) {
        const std::lock_guard<std::mutex> lock(search_mutex);
        for (int yolo=0; yolo<100; yolo++) {
          if (search.Step() == rrts::StepResult::kSuccess) {
            count++;
          }
        }
      }
      std::this_thread::sleep_for(1us);
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
