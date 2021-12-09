#include <GL/glew.h>         // for GL_LINES, GL_LINE_STRIP, GL_POINTS
#include <bits/exception.h>  // for exception
#include <sys/types.h>       // for key_t

#include <algorithm>           // for max
#include <chrono>              // for operator""ms, operator""us, chrono_literals
#include <cmath>               // for pow, M_PI
#include <cstdio>              // for fprintf, size_t, stderr
#include <cstdlib>             // for EXIT_SUCCESS
#include <ext/alloc_traits.h>  // for __alloc_traits<>::value_type
#include <functional>          // for function
#include <glm/glm.hpp>         // for vec<>::(anonymous), vec3, operator+, operator-, dvec3
#include <iostream>            // for operator<<, basic_ostream, endl, ostream, basic_os...
#include <mutex>               // for mutex, lock_guard
#include <random>              // for mt19937_64, uniform_real_distribution, uniform_int...
#include <sstream>             // for stringstream
#include <thread>              // for sleep_for, thread
#include <tuple>               // for tuple
#include <vector>              // for vector

#include "bb3d/opengl_context.hpp"     // for Window
#include "bb3d/shader/colorlines.hpp"  // for ColoredVec3, ColorLines
#include "bb3d/shader/freetype.hpp"    // for Freetype
#include "bb3d/shader/lines.hpp"       // for Lines
#include "src/search.hpp"              // for Edge, Search, StepResult, StepResult::kSuccess
#include "src/space/r3.hpp"            // for Sphere, Point, Line, R3
#include "src/tree/fast.hpp"           // for Fast

using Line = rrts::space::r3::Line;
using Bridge = Line;
using Point = rrts::space::r3::Point;
using Sphere = rrts::space::r3::Sphere;
// using Tree = rrts::tree::Naive<Point, Bridge, 3>;  // comment in for testing Fast tree
using Tree = rrts::tree::Fast<Point, Bridge, 3>;
using Space = rrts::space::r3::R3;

struct Problem {
  Problem(const Point &lb, const Point &ub, const Sphere &goal_region,
          const std::vector<Sphere> &obstacles)
      : lb_(lb),
        ub_(ub),
        goal_region_(goal_region),
        obstacles_(obstacles),
        r3_space_(lb, ub, obstacles),
        search_(x_init, r3_space_, 0.55){};
  Point x_init = {0, 0, 0};
  Point lb_;
  Point ub_;
  Sphere goal_region_;
  std::vector<Sphere> obstacles_;
  Space r3_space_;
  rrts::Search<Point, Line, 3, Tree, Space> search_;

  void Describe() {
    fprintf(stderr, "lb: % 7.2f % 7.2f % 7.2f\n", lb_.x, lb_.y, lb_.z);
    fprintf(stderr, "ub: % 7.2f % 7.2f % 7.2f\n", ub_.x, ub_.y, ub_.z);
    int k = 0;
    for (const Sphere &s : obstacles_) {
      fprintf(stderr, "obstacle %d: radius %7.2f, center % 7.2f % 7.2f % 7.2f\n", k, s.radius,
              s.center.x, s.center.y, s.center.z);
      k++;
    }
  }

  void UpdateSphereLines(bb3d::ColorLines &sphere_lines) const {
    const glm::vec4 obstacle_color = {1, 1, 0, 1};
    const glm::vec4 goal_color = {0, 1, 1, 1};
    std::vector<std::vector<bb3d::ColoredVec3> > sphere_axes;

    std::function<void(const Sphere &, const glm::vec4 &)> push_sphere =
        [&sphere_axes](const Sphere &sphere, glm::vec4 color) {
          glm::vec3 dx = {sphere.radius, 0, 0};
          glm::vec3 dy = {0, sphere.radius, 0};
          glm::vec3 dz = {0, 0, sphere.radius};
          const glm::vec3 &center = sphere.center;
          sphere_axes.push_back({{{center + dx, color}, {center - dx, color}}});
          sphere_axes.push_back({{{center + dy, color}, {center - dy, color}}});
          sphere_axes.push_back({{{center + dz, color}, {center - dz, color}}});
        };

    for (const Sphere &sphere : obstacles_) {
      push_sphere(sphere, obstacle_color);
    }
    push_sphere(goal_region_, goal_color);
    sphere_lines.Update(sphere_axes);
  }

  void UpdateBoundingBoxLines(bb3d::Lines &bounding_box_lines) const {
    std::vector<std::vector<glm::vec3> > bb_lines;
    bb_lines.push_back({{lb_.x, lb_.y, lb_.z}, {ub_.x, lb_.y, lb_.z}});
    bb_lines.push_back({{lb_.x, lb_.y, ub_.z}, {ub_.x, lb_.y, ub_.z}});
    bb_lines.push_back({{lb_.x, ub_.y, lb_.z}, {ub_.x, ub_.y, lb_.z}});
    bb_lines.push_back({{lb_.x, ub_.y, ub_.z}, {ub_.x, ub_.y, ub_.z}});

    bb_lines.push_back({{lb_.x, lb_.y, lb_.z}, {lb_.x, ub_.y, lb_.z}});
    bb_lines.push_back({{lb_.x, lb_.y, ub_.z}, {lb_.x, ub_.y, ub_.z}});
    bb_lines.push_back({{ub_.x, lb_.y, lb_.z}, {ub_.x, ub_.y, lb_.z}});
    bb_lines.push_back({{ub_.x, lb_.y, ub_.z}, {ub_.x, ub_.y, ub_.z}});

    bb_lines.push_back({{lb_.x, lb_.y, lb_.z}, {lb_.x, lb_.y, ub_.z}});
    bb_lines.push_back({{lb_.x, ub_.y, lb_.z}, {lb_.x, ub_.y, ub_.z}});
    bb_lines.push_back({{ub_.x, lb_.y, lb_.z}, {ub_.x, lb_.y, ub_.z}});
    bb_lines.push_back({{ub_.x, ub_.y, lb_.z}, {ub_.x, ub_.y, ub_.z}});
    bounding_box_lines.Update(bb_lines);
  }

  void UpdateBridgeLines(bb3d::ColorLines &lines, const std::vector<double> &cost_to_go) const {
    const std::vector<rrts::Edge<Point, Line> > &edges = search_.Edges();

    // Find max cost to go in order to scale lines
    double max_cost_to_go = 0;
    for (double c : cost_to_go) {
      max_cost_to_go = std::max(max_cost_to_go, c);
    }

    // Draw all bridges
    std::vector<std::vector<bb3d::ColoredVec3> > bridges;
    size_t node_index = 1;
    for (const rrts::Edge<Point, Line> &edge : edges) {
      bb3d::ColoredVec3 cv0{};
      bb3d::ColoredVec3 cv1{};
      cv0.position = static_cast<glm::dvec3>(edge.bridge_.p0);
      cv1.position = static_cast<glm::dvec3>(edge.bridge_.p1);
      const double ctg0 = cost_to_go.at(edge.parent_index_) / max_cost_to_go;
      const double ctg1 = cost_to_go.at(node_index) / max_cost_to_go;
      cv0.color = {ctg0, 0, 1 - ctg0, 0.6};
      cv1.color = {ctg1, 0, 1 - ctg1, 0.6};
      bridges.push_back({cv0, cv1});
      node_index++;
    }
    lines.Update(bridges);
  }

  [[nodiscard]] bool InGoalRegion(const Point &p) const {
    double dist = glm::distance(glm::dvec3(p), goal_region_.center);
    return dist <= goal_region_.radius;
  }

  double UpdateGoalLine(bb3d::Lines &goal_line, const std::vector<double> &cost_to_go) const {
    const std::vector<rrts::Edge<Point, Line> > &edges = search_.Edges();

    double min_cost_to_go = 0;
    size_t winner_index = 0;
    bool got_winner = false;
    size_t index = 1;
    // best cost to go in a goal region
    for (const rrts::Edge<Point, Line> &edge : edges) {
      if (InGoalRegion(edge.bridge_.p1) && (cost_to_go.at(index) < min_cost_to_go || !got_winner)) {
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
        glm::vec3 p = static_cast<glm::dvec3>(edge.bridge_.p1);
        p.z -= 0.05F;
        // std::cerr << edge.bridge.p1.x << " " << edge.bridge.p1.y << " " << edge.bridge.p1.z << "
        // " << std::endl;
        winning_route.push_back(p);
        head = edge.parent_index_;

        if (head == 0) {
          glm::vec3 pf = static_cast<glm::dvec3>(edge.bridge_.p0);
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
};

static Problem RandomProblem(std::mt19937_64 &rng_engine) {
  std::uniform_real_distribution<double> uniform_distribution;
  std::function<double(void)> uniform = [&uniform_distribution, &rng_engine]() {
    return uniform_distribution(rng_engine);
  };
  std::function<Point(void)> uniform_p = [&uniform]() {
    return Point{uniform(), uniform(), uniform()};
  };

  const double dx = 3 + 2 * uniform();
  const double dy = 3 + 2 * uniform();
  const double dz = 3 + 2 * uniform();
  const Point lb = {0, -dy / 2, -dz / 2};
  const Point ub = {dx, dy / 2, dz / 2};

  const Point goal = {ub[0], dy * (uniform() - 0.5), dz * (uniform() - 0.5)};
  const double goal_radius = 0.5;
  Sphere goal_region = {goal, goal_radius};

  // obstacles
  const int max_num_obstacles = 10;
  const size_t n =
      static_cast<size_t>(std::uniform_int_distribution<>(1, max_num_obstacles + 1)(rng_engine));
  const double total_volume = dx * dy * dz;
  const double obstacle_fraction = 0.6;
  // upper bound (no overlap)
  const double volume_per_obstacle = total_volume * obstacle_fraction / static_cast<double>(n);
  const double radius_per_obstacle = pow(3 * volume_per_obstacle / (4 * M_PI), 1.0 / 3.0);
  std::vector<Sphere> obstacles;
  while (obstacles.size() < n) {
    const double obstacle_radius = radius_per_obstacle * (0.7 + 0.7 * uniform());
    const Point p = {dx * uniform(), dy * (uniform() - 0.5), dz * (uniform() - 0.5)};
    const bool avoids_goal =
        glm::distance(glm::dvec3(p), glm::dvec3(goal)) > obstacle_radius + goal_radius;
    const bool avoids_start = glm::length(glm::dvec3(p)) > obstacle_radius + 0.5;
    if (avoids_goal && avoids_start) {
      Sphere obstacle = {p, obstacle_radius};
      obstacles.push_back(obstacle);
    }
  }

  return Problem(lb, ub, goal_region, obstacles);
}

// static void UpdatePoints(bb3d::Lines &lines,
//                         const std::vector<Tagged<Point> > &tagged_points) {
//  std::vector<glm::vec3> points;
//  for (const Tagged<Point> &x: tagged_points) {
//    points.push_back(x.point);
//  }
//  std::vector<std::vector<glm::vec3> > point_strip{points};
//  lines.Update(point_strip);
//}

int Run(char *argv0) {
  std::mt19937_64 rng_engine;  // NOLINT
  Problem problem = RandomProblem(rng_engine);

  problem.Describe();

  bb3d::Window window(argv0);
  window.SetCameraFocus({0, 0, 0});
  window.SetCameraAzimuthDeg(-45);

  bb3d::Lines points;
  points.SetPointSize(2);
  bb3d::ColorLines bridge_lines;
  bb3d::Lines goal_line;
  bb3d::ColorLines sphere_lines;
  bb3d::Lines bounding_box_lines;
  bb3d::Freetype textbox(18);

  bool pause = false;
  std::function<void(key_t)> handle_keypress = [&window, &pause](key_t key) {
    if (key == 80) {
      pause = !pause;
      std::cout << "pausing " << pause << std::endl;
    };
  };

  std::mutex search_mutex;
  double min_cost_to_go = -1;
  std::function<void()> update_visualization = [&window, &problem, &bridge_lines, &goal_line,
                                                &points, &search_mutex, &min_cost_to_go,
                                                &sphere_lines, &bounding_box_lines]() {
    const std::lock_guard<std::mutex> lock(search_mutex);

    static float azimuth_deg = 0.f;
    azimuth_deg += 0.5f;
    while (azimuth_deg > 180.f) {
      azimuth_deg -= 360.f;
    }
    window.SetCameraAzimuthDeg(azimuth_deg);
    window.SetCameraFocus(0.5 * (problem.lb_ + problem.ub_));

    const std::vector<double> cost_to_go = problem.search_.ComputeCostsToGo();
    problem.UpdateBridgeLines(bridge_lines, cost_to_go);
    min_cost_to_go = problem.UpdateGoalLine(goal_line, cost_to_go);
    // UpdatePoints(points, search.tree_.points_);

    problem.UpdateSphereLines(sphere_lines);
    problem.UpdateBoundingBoxLines(bounding_box_lines);
  };

  std::function<void(const glm::mat4 &, const glm::mat4 &)> draw_visualization =
      [&sphere_lines, &bridge_lines, &bounding_box_lines, &goal_line, &points, &window, &textbox,
       &search_mutex, &min_cost_to_go, &problem](const glm::mat4 &view, const glm::mat4 &proj) {
        glm::vec4 point_color = {1, 1, 1, 0.3};
        glm::vec4 bb_color = {1, 1, 1, 0.5};
        glm::vec4 goal_color = {1, 1, 1, 1};
        sphere_lines.Draw(view, proj, GL_LINES);
        bridge_lines.Draw(view, proj, GL_LINES);
        goal_line.Draw(view, proj, goal_color, GL_LINE_STRIP);
        points.Draw(view, proj, point_color, GL_POINTS);
        bounding_box_lines.Draw(view, proj, bb_color, GL_LINES);

        // some text
        std::stringstream message;
        {
          const std::lock_guard<std::mutex> lock(search_mutex);
          message << problem.search_.Cardinality() << " nodes in tree";
          if (min_cost_to_go > 0) {
            message << " optimal distance is " << min_cost_to_go;
          }
        }
        window.RenderText(textbox, message.str(), 25.0F, 25.0F, glm::vec3(1, 1, 1));
      };

  // it's theadn' time
  std::thread thread_object(
      [&pause, &search_mutex, &problem, &rng_engine, &sphere_lines, &bounding_box_lines]() {
        using namespace std::chrono_literals;
        for (;;) {
          int count = 0;
          while (count < 20000) {
            if (!pause) {
              const std::lock_guard<std::mutex> lock(search_mutex);
              for (int yolo = 0; yolo < 100; yolo++) {
                if (problem.search_.Step() == rrts::StepResult::kSuccess) {
                  count++;
                }
              }
            }
            std::this_thread::sleep_for(1us);
          }

          // sleep
          std::this_thread::sleep_for(500ms);

          // reset the problem to a new random one
          {
            const std::lock_guard<std::mutex> lock(search_mutex);
            problem = RandomProblem(rng_engine);
          }
        }
        std::cerr << "finished" << std::endl;
      });

  window.Run(handle_keypress, update_visualization, draw_visualization);

  return EXIT_SUCCESS;
}

int main(int argc __attribute__((unused)), char *argv[]) {
  try {
    Run(argv[0]);  // NOLINT
  } catch (const std::exception &e) {
    std::cerr << e.what();
  }
}
