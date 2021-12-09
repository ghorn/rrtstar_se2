#include <GL/glew.h>         // for GL_LINES, GL_LINE_STRIP, GL_POINTS
#include <bits/exception.h>  // for exception
#include <sys/types.h>       // for key_t

#include <algorithm>                   // for max
#include <chrono>                      // for operator""ms, operator""us, chrono_literals
#include <cmath>                       // for sqrt, M_PI
#include <cstdio>                      // for fprintf, size_t, stderr
#include <cstdlib>                     // for EXIT_SUCCESS
#include <ext/alloc_traits.h>          // for __alloc_traits<>::value_type
#include <functional>                  // for function
#include <glm/ext/vector_double2.hpp>  // for dvec2
#include <glm/glm.hpp>                 // for vec3, vec<>::(anonymous), operator+, operator-, vec2
#include <glm/gtx/norm.hpp>            // for distance2
#include <iostream>                    // for operator<<, basic_ostream, endl, ostream, cerr
#include <mutex>                       // for mutex, lock_guard
#include <random>                      // for mt19937_64, uniform_real_distribution, uniform_in...
#include <sstream>                     // for stringstream
#include <thread>                      // for sleep_for, thread
#include <tuple>                       // for tuple
#include <vector>                      // for vector

#include "bb3d/opengl_context.hpp"      // for Window
#include "bb3d/shader/colorlines.hpp"   // for ColoredVec3, ColorLines
#include "bb3d/shader/freetype.hpp"     // for Freetype
#include "bb3d/shader/lines.hpp"        // for Lines
#include "src/search.hpp"               // for Edge, Search, StepResult, StepResult::kSuccess
#include "src/space/dubins/dubins.hpp"  // for DubinsPath, DubinsStatus, Se2Coord
#include "src/space/se2.hpp"            // for Sphere, Se2, Se2Coord
#include "src/tree/fast.hpp"            // for Fast

using DubinsPath = rrts::dubins::DubinsPath;
using Line = DubinsPath;
using Se2Coord = rrts::space::se2::Se2Coord;
using Point = rrts::space::se2::Se2Coord;
using Sphere = rrts::space::se2::Sphere;
// using Tree = rrts::tree::Naive<Point, Line, 3>;  // comment in for testing Fast tree
using Tree = rrts::tree::Fast<Point, Line, 3>;
using Space = rrts::space::se2::Se2;

using DubinsStatus = rrts::dubins::DubinsStatus;

inline glm::vec3 ToVec3(const glm::vec2 &v2, float z) { return glm::vec3(v2.x, v2.y, z); }

// std::vector<std::vector<bb3d::ColoredVec3> > DrawBridge(const DubinsPath &path, double ctg0,
// double ctg1, double z __attribute__((unused))) {
//
//  std::vector<std::vector<bb3d::ColoredVec3> > lines;
//  lines.push_back({});
//
//  constexpr int N = 20;
//  for (int j=0; j<N; j++) {
//    double t = 0.999*path.total_length * static_cast<double>(j) / (N - 1); // static_cast)
//    std::array<double, 3> q{};
//    DubinsStatus sample_ret = DubinsPathSample(path, t, q);
//    if (sample_ret != DubinsStatus::kSuccess) {
//      std::cerr << "bad return code for sampling: " << static_cast<int>(sample_ret) << std::endl;
//    //double q[3];
//    //int sample_ret = dubins_path_sample(&path, t, q);
//    //if (sample_ret != 0) {
//    //  std::cerr << "bad return code for sampling: " << sample_ret << std::endl;
//      std::exit(EXIT_FAILURE);
//    }
//    //if (first) {
//    //  std::cout << "t " << t << ": " << q[0] << ", " << q[1] << std::endl;
//    //}
//    double ctg = ctg0 * (1 - t) + ctg1 * t;
//    glm::vec4 color = {ctg, 0, 1 - ctg, 0.6};
//    //bb3d::ColoredVec3 v = {{q[0], q[1], z}, color};
//    bb3d::ColoredVec3 v = {{q[0], q[1], q[2]}, color};
//    if (j > 0) {
//      const std::vector<bb3d::ColoredVec3> &latest_vec = lines.at(lines.size()-1);
//      double prev_q = latest_vec.at(latest_vec.size() - 1).position.z;
//      if ((prev_q > 2.2 && q[2] < -2.2) || (prev_q < -2.2 && q[2] > 2.2)) {
//        lines.push_back({});
//      }
//    }
//    lines.at(lines.size()-1).push_back(v);
//  }
//  return lines;
//}
std::vector<bb3d::ColoredVec3> DrawBridge(const DubinsPath &path, double ctg0, double ctg1,
                                          double z __attribute__((unused))) {
  std::vector<bb3d::ColoredVec3> line;

  constexpr int kN = 20;
  for (int j = 0; j < kN; j++) {
    double t = 0.999 * path.TotalLength() * static_cast<double>(j) / (kN - 1);  // static_cast)
    Se2Coord q = path.Sample(t);
    double ctg = ctg0 * (1 - t) + ctg1 * t;
    // std::cout << "t " << t << ": " << q[0] << ", " << q[1] << "   | " << ctg << std::endl;
    glm::vec4 color = {1 - ctg, 0, ctg, 0.6};
    bb3d::ColoredVec3 v = {{q[0], q[1], z}, color};
    // bb3d::ColoredVec3 v = {{q[0], q[1], q[2]}, color};

    line.push_back(v);
  }
  return line;
}

void DrawBridgeReverse(std::vector<glm::vec3> &line, const DubinsPath &path,
                       double z __attribute__((unused))) {
  constexpr int kN = 20;
  for (int j = 0; j < kN; j++) {
    double t =
        0.999 * path.TotalLength() * static_cast<double>(kN - 1 - j) / (kN - 1);  // static_cast)
    Se2Coord q = path.Sample(t);
    // if (first) {
    //  std::cout << "t " << t << ": " << q[0] << ", " << q[1] << std::endl;
    //}
    glm::vec3 v = {q[0], q[1], z};
    // glm::vec3 v = {q[0], q[1], q[2] + 0.05};
    line.push_back(v);
  }
}

struct GoalRegion {
  Sphere position;
  double min_angle;
  double max_angle;
};

struct Problem {
  Problem(double rho, double eta, const glm::dvec2 &lb, const glm::dvec2 &ub,
          const Sphere &goal_region, const std::vector<Sphere> &obstacles)
      : goal_region_{goal_region, -45.0 * M_PI / 180.0, 45.0 * M_PI / 180.0},
        obstacles_(obstacles),
        se2_space_(rho, lb, ub, obstacles),
        search_(x_init, se2_space_, eta){};
  Se2Coord x_init = {{0, 0}, 0};
  GoalRegion goal_region_;
  std::vector<Sphere> obstacles_;
  Space se2_space_;
  rrts::Search<Point, Line, 3, Tree, Space> search_;

  [[nodiscard]] const Point &Lb() const { return se2_space_.Lb(); };
  [[nodiscard]] const Point &Ub() const { return se2_space_.Ub(); };

 public:
  void Describe() {
    fprintf(stderr, "lb: x % 7.2f, y % 7.2f, theta % 7.2f\n", Lb().position.x, Lb().position.y,
            Lb().theta);
    fprintf(stderr, "ub: x % 7.2f, y % 7.2f, theta % 7.2f\n", Ub().position.x, Ub().position.y,
            Ub().theta);
    int k = 0;
    for (const Sphere &s : obstacles_) {
      fprintf(stderr, "obstacle %d: radius %7.2f, center % 7.2f % 7.2f\n", k, s.radius, s.center.x,
              s.center.y);
      k++;
    }
  }

  void UpdateSphereLines(bb3d::ColorLines &sphere_lines) const {
    const glm::vec4 obstacle_color = {1, 1, 0, 1};
    const glm::vec4 goal_color = {0, 1, 1, 1};
    std::vector<std::vector<bb3d::ColoredVec3> > sphere_axes;

    std::function<void(const Sphere &, const glm::vec4 &)> push_sphere =
        [&sphere_axes](const Sphere &sphere, glm::vec4 color) {
          glm::vec2 dx = {sphere.radius, 0};
          glm::vec2 dy = {0, sphere.radius};
          const glm::vec2 &center = sphere.center;
          sphere_axes.push_back(
              {{{ToVec3(center + dx, 0.f), color}, {ToVec3(center - dx, 0.f), color}}});
          sphere_axes.push_back(
              {{{ToVec3(center + dy, 0.f), color}, {ToVec3(center - dy, 0.f), color}}});
        };

    for (const Sphere &sphere : obstacles_) {
      push_sphere(sphere, obstacle_color);
    }
    push_sphere(goal_region_.position, goal_color);
    sphere_lines.Update(sphere_axes);
  }

  void UpdateBoundingBoxLines(bb3d::Lines &bounding_box_lines) const {
    std::vector<std::vector<glm::vec3> > bb_lines;
    bb_lines.push_back(
        {{Lb().position.x, Lb().position.y, 0}, {Lb().position.x, Ub().position.y, 0}});
    bb_lines.push_back(
        {{Lb().position.x, Ub().position.y, 0}, {Ub().position.x, Ub().position.y, 0}});
    bb_lines.push_back(
        {{Ub().position.x, Ub().position.y, 0}, {Ub().position.x, Lb().position.y, 0}});
    bb_lines.push_back(
        {{Ub().position.x, Lb().position.y, 0}, {Lb().position.x, Lb().position.y, 0}});
    // bb_lines.push_back({{lb_.x, lb_.y, lb_.z}, {ub_.x, lb_.y, lb_.z}});
    // bb_lines.push_back({{lb_.x, lb_.y, ub_.z}, {ub_.x, lb_.y, ub_.z}});
    // bb_lines.push_back({{lb_.x, ub_.y, lb_.z}, {ub_.x, ub_.y, lb_.z}});
    // bb_lines.push_back({{lb_.x, ub_.y, ub_.z}, {ub_.x, ub_.y, ub_.z}});
    //
    // bb_lines.push_back({{lb_.x, lb_.y, lb_.z}, {lb_.x, ub_.y, lb_.z}});
    // bb_lines.push_back({{lb_.x, lb_.y, ub_.z}, {lb_.x, ub_.y, ub_.z}});
    // bb_lines.push_back({{ub_.x, lb_.y, lb_.z}, {ub_.x, ub_.y, lb_.z}});
    // bb_lines.push_back({{ub_.x, lb_.y, ub_.z}, {ub_.x, ub_.y, ub_.z}});
    //
    // bb_lines.push_back({{lb_.x, lb_.y, lb_.z}, {lb_.x, lb_.y, ub_.z}});
    // bb_lines.push_back({{lb_.x, ub_.y, lb_.z}, {lb_.x, ub_.y, ub_.z}});
    // bb_lines.push_back({{ub_.x, lb_.y, lb_.z}, {ub_.x, lb_.y, ub_.z}});
    // bb_lines.push_back({{ub_.x, ub_.y, lb_.z}, {ub_.x, ub_.y, ub_.z}});
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
      const double ctg0 = cost_to_go.at(edge.parent_index_) / max_cost_to_go;
      const double ctg1 = cost_to_go.at(node_index) / max_cost_to_go;

      bridges.push_back(DrawBridge(edge.bridge_, ctg0, ctg1, 0));
      // std::vector<std::vector<bb3d::ColoredVec3> > next_bridge = DrawBridge(edge.bridge_, ctg0,
      // ctg1, 0); bridges.reserve(bridges.size() + next_bridge.size());
      // bridges.insert(bridges.end(), next_bridge.begin(), next_bridge.end());
      node_index++;
    }
    std::sort(
        bridges.begin(), bridges.end(),
        [](const std::vector<bb3d::ColoredVec3> &b0, const std::vector<bb3d::ColoredVec3> &b1) {
          return b0.at(b0.size() - 1).color[0] > b1.at(b1.size() - 1).color[0];
        });
    lines.Update(bridges);
  }

  [[nodiscard]] bool InGoalRegion(const Point &p) const {
    const bool heading_good =
        goal_region_.min_angle <= p.theta && p.theta <= goal_region_.max_angle;
    const bool xy_good = sqrt(glm::distance2(p.position, goal_region_.position.center)) <=
                         goal_region_.position.radius;
    const bool in_goal = xy_good && heading_good;

    return in_goal;
  }

  double UpdateGoalLine(bb3d::Lines &goal_line, const std::vector<double> &cost_to_go) const {
    const std::vector<rrts::Edge<Point, Line> > &edges = search_.Edges();

    double min_cost_to_go = 0;
    size_t winner_index = 0;
    bool got_winner = false;
    size_t index = 1;
    // best cost to go in a goal region
    for (const rrts::Edge<Point, Line> &edge : edges) {
      const Point &endpoint = edge.bridge_.Endpoint();
      if (InGoalRegion(endpoint) && (cost_to_go.at(index) < min_cost_to_go || !got_winner)) {
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
        DrawBridgeReverse(winning_route, edge.bridge_, -0.05F);
        //        glm::vec3 p = static_cast<glm::dvec3>(edge.bridge_.p1);
        //        p.z -= 0.05F;
        // std::cerr << edge.bridge.p1.x << " " << edge.bridge.p1.y << " " << edge.bridge.p1.z << "
        // " << std::endl;
        // winning_route.push_back(p);
        head = edge.parent_index_;

        // if (head == 0) {
        //  glm::vec3 pf = static_cast<glm::dvec3>(edge.bridge_.p0);
        //  pf.z -= 0.02F;
        //  winning_route.push_back(pf);
        //}
      }
      std::vector<std::vector<glm::vec3> > segments;
      segments.push_back(winning_route);

      goal_line.Update(segments);

      return min_cost_to_go;
    }

    // if no winner, update with no segments to erase any lines from previous problems
    std::vector<std::vector<glm::vec3> > segments;
    goal_line.Update(segments);
    return -1;
  }
};

static Problem RandomProblem(std::mt19937_64 &rng_engine) {
  std::uniform_real_distribution<double> uniform_distribution;
  std::function<double(void)> uniform = [&uniform_distribution, &rng_engine]() {
    return uniform_distribution(rng_engine);
  };
  std::function<Point(void)> uniform_p = [&uniform]() {
    return Point{{uniform(), uniform()}, uniform()};
  };

  const double dx = 3 + 2 * uniform();
  const double dy = 3 + 2 * uniform();
  const glm::dvec2 lb = {0, -dy / 2};
  const glm::dvec2 ub = {dx, dy / 2};

  const glm::dvec2 goal = {ub[0], dy * (uniform() - 0.5)};
  const double goal_radius = 0.5;
  Sphere goal_region = {goal, goal_radius};

  // obstacles
  const int max_num_obstacles = 20;
  const size_t n =
      static_cast<size_t>(std::uniform_int_distribution<>(1, max_num_obstacles)(rng_engine));
  const double total_volume = dx * dy;
  const double obstacle_fraction = 0.5;
  // upper bound (no overlap)
  const double volume_per_obstacle = total_volume * obstacle_fraction / static_cast<double>(n);
  const double radius_per_obstacle = sqrt(volume_per_obstacle / M_PI);

  std::vector<Sphere> obstacles;
  while (obstacles.size() < n) {
    const double obstacle_radius = radius_per_obstacle * (0.5 + 0.5 * uniform());
    const glm::dvec2 p = {dx * uniform(), dy * (uniform() - 0.5)};
    const bool avoids_goal = glm::distance(p, goal) > obstacle_radius + goal_radius;
    const bool avoids_start = glm::length(p) > obstacle_radius + 1.3;  // careful of turning radius
    if (avoids_goal && avoids_start) {
      Sphere obstacle = {p, obstacle_radius};
      obstacles.push_back(obstacle);
    }
  }

  double rho = 0.6;
  double eta = 4.5;
  return Problem(rho, eta, lb, ub, goal_region, obstacles);
}

int Run(char *argv0) {
  std::mt19937_64 rng_engine;  // NOLINT
  Problem problem = RandomProblem(rng_engine);

  problem.Describe();

  bb3d::Window window(argv0);
  window.SetCameraFocus({0, 0, 0});
  window.SetCameraAzimuthDeg(180);
  window.SetCameraElevationDeg(89.999F);

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
    // static float azimuth_deg = 0.f;
    // azimuth_deg += 0.5f;
    // while (azimuth_deg > 180.f) {
    //  azimuth_deg -= 360.f;
    //}
    // window.SetCameraAzimuthDeg(azimuth_deg);
    const glm::dvec2 xy_center = 0.5 * (problem.Lb().position + problem.Ub().position);
    window.SetCameraFocus({xy_center.x, xy_center.y, 0});

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
        bridge_lines.Draw(view, proj, GL_LINE_STRIP);
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
        try {
          for (;;) {
            int count = 0;
            while (count < 10000) {
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
            std::this_thread::sleep_for(1000ms);

            // reset the problem to a new random one
            {
              const std::lock_guard<std::mutex> lock(search_mutex);
              problem = RandomProblem(rng_engine);
            }
          }
          std::cerr << "finished" << std::endl;
        } catch (std::exception &e) {
          std::cerr << "Thread got exception: " << e.what() << std::endl;
          // return EXIT_FAILURE;
        }
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
