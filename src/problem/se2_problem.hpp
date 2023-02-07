#pragma once

// #include <sys/types.h>       // for key_t

// #include <algorithm>    // for max
// #include <chrono>       // for operator""ms, operator""us, chrono_literals
// #include <cmath>        // for sqrt, M_PI
// #include <cstdio>       // for fprintf, size_t, stderr
// #include <cstdlib>      // for EXIT_SUCCESS
#include <functional>  // for function
// #include <iostream>     // for operator<<, basic_ostream, endl, ostream, cerr
// #include <random>       // for mt19937_64, uniform_real_distribution, uniform_in...
// #include <sstream>      // for stringstream
// #include <thread>       // for sleep_for, thread
// #include <tuple>        // for tuple
// #include <vector>       // for vector

#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>

#include "src/problem/xyz_rgb.hpp"
#include "src/search.hpp"               // for Edge, Search, StepResult, StepResult::kSuccess
#include "src/space/dubins/dubins.hpp"  // for DubinsPath, DubinsStatus, Se2Coord
#include "src/space/se2.hpp"            // for Sphere, Se2, Se2Coord
#include "src/tree/fast.hpp"            // for Fast

struct Se2Problem {
  using DubinsPath = rrts::dubins::DubinsPath;
  using Line = DubinsPath;
  using Se2Coord = rrts::space::se2::Se2Coord;
  using Point = rrts::space::se2::Se2Coord;
  using Sphere = rrts::space::se2::Sphere;
  // using Tree = rrts::tree::Naive<Point, Line, 3>;  // comment in for testing Fast tree
  using Tree = rrts::tree::Fast<Point, Line, 3>;
  using Space = rrts::space::se2::Se2;

  using DubinsStatus = rrts::dubins::DubinsStatus;

  struct GoalRegion {
    Sphere position;
    double min_angle;
    double max_angle;
  };

  Se2Problem(double rho, double eta, const glm::dvec2 &lb, const glm::dvec2 &ub,
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
  // void UpdateSphereLines(bb3d::ColorLines &sphere_lines) const {
  //   const glm::vec4 obstacle_color = {1, 1, 0, 1};
  //   const glm::vec4 goal_color = {0, 1, 1, 1};
  //   std::vector<std::vector<bb3d::ColoredVec3> > sphere_axes;

  //   std::function<void(const Sphere &, const glm::vec4 &)> push_sphere =
  //       [&sphere_axes](const Sphere &sphere, glm::vec4 color) {
  //         glm::vec2 dx = {sphere.radius, 0};
  //         glm::vec2 dy = {0, sphere.radius};
  //         const glm::vec2 &center = sphere.center;
  //         sphere_axes.push_back(
  //             {{{ToVec3(center + dx, 0.f), color}, {ToVec3(center - dx, 0.f), color}}});
  //         sphere_axes.push_back(
  //             {{{ToVec3(center + dy, 0.f), color}, {ToVec3(center - dy, 0.f), color}}});
  //       };

  //   for (const Sphere &sphere : obstacles_) {
  //     push_sphere(sphere, obstacle_color);
  //   }
  //   push_sphere(goal_region_.position, goal_color);
  //   sphere_lines.Update(sphere_axes);
  // }

  // void UpdateBoundingBoxLines(bb3d::Lines &bounding_box_lines) const {
  //   std::vector<std::vector<glm::vec3> > bb_lines;
  //   bb_lines.push_back(
  //       {{Lb().position.x, Lb().position.y, 0}, {Lb().position.x, Ub().position.y, 0}});
  //   bb_lines.push_back(
  //       {{Lb().position.x, Ub().position.y, 0}, {Ub().position.x, Ub().position.y, 0}});
  //   bb_lines.push_back(
  //       {{Ub().position.x, Ub().position.y, 0}, {Ub().position.x, Lb().position.y, 0}});
  //   bb_lines.push_back(
  //       {{Ub().position.x, Lb().position.y, 0}, {Lb().position.x, Lb().position.y, 0}});
  //   // bb_lines.push_back({{lb_.x, lb_.y, lb_.z}, {ub_.x, lb_.y, lb_.z}});
  //   // bb_lines.push_back({{lb_.x, lb_.y, ub_.z}, {ub_.x, lb_.y, ub_.z}});
  //   // bb_lines.push_back({{lb_.x, ub_.y, lb_.z}, {ub_.x, ub_.y, lb_.z}});
  //   // bb_lines.push_back({{lb_.x, ub_.y, ub_.z}, {ub_.x, ub_.y, ub_.z}});
  //   //
  //   // bb_lines.push_back({{lb_.x, lb_.y, lb_.z}, {lb_.x, ub_.y, lb_.z}});
  //   // bb_lines.push_back({{lb_.x, lb_.y, ub_.z}, {lb_.x, ub_.y, ub_.z}});
  //   // bb_lines.push_back({{ub_.x, lb_.y, lb_.z}, {ub_.x, ub_.y, lb_.z}});
  //   // bb_lines.push_back({{ub_.x, lb_.y, ub_.z}, {ub_.x, ub_.y, ub_.z}});
  //   //
  //   // bb_lines.push_back({{lb_.x, lb_.y, lb_.z}, {lb_.x, lb_.y, ub_.z}});
  //   // bb_lines.push_back({{lb_.x, ub_.y, lb_.z}, {lb_.x, ub_.y, ub_.z}});
  //   // bb_lines.push_back({{ub_.x, lb_.y, lb_.z}, {ub_.x, lb_.y, ub_.z}});
  //   // bb_lines.push_back({{ub_.x, ub_.y, lb_.z}, {ub_.x, ub_.y, ub_.z}});
  //   bounding_box_lines.Update(bb_lines);
  // }

  std::vector<std::vector<XyzRgb> > GetBridgeLines() const;

  [[nodiscard]] bool InGoalRegion(const Point &p) const {
    const bool heading_good =
        goal_region_.min_angle <= p.theta && p.theta <= goal_region_.max_angle;
    const bool xy_good = sqrt(glm::distance2(p.position, goal_region_.position.center)) <=
                         goal_region_.position.radius;
    const bool in_goal = xy_good && heading_good;

    return in_goal;
  }

  std::vector<std::vector<XyzRgb> > GetGoalLine() const;

  static Se2Problem RandomProblem(std::mt19937_64 &rng_engine);
};  // class Se2Problem

// int Run(char *argv0) {
//   std::mt19937_64 rng_engine;  // NOLINT
//   Problem problem = RandomProblem(rng_engine);

//   problem.Describe();

//   bb3d::Window window(argv0);
//   window.SetCameraFocus({0, 0, 0});
//   window.SetCameraAzimuthDeg(180);
//   window.SetCameraElevationDeg(89.999F);

//   bb3d::Lines points;
//   points.SetPointSize(2);
//   bb3d::ColorLines bridge_lines;
//   bb3d::Lines goal_line;
//   bb3d::ColorLines sphere_lines;
//   bb3d::Lines bounding_box_lines;
//   bb3d::Freetype textbox(18);

//   bool pause = false;
//   std::function<void(key_t)> handle_keypress = [&window, &pause](key_t key) {
//     if (key == 80) {
//       pause = !pause;
//       std::cout << "pausing " << pause << std::endl;
//     };
//   };

//   std::mutex search_mutex;
//   double min_cost_to_go = -1;
//   std::function<void()> update_visualization = [&window, &problem, &bridge_lines, &goal_line,
//                                                 &points, &search_mutex, &min_cost_to_go,
//                                                 &sphere_lines, &bounding_box_lines]() {
//     const std::lock_guard<std::mutex> lock(search_mutex);
//     // static float azimuth_deg = 0.f;
//     // azimuth_deg += 0.5f;
//     // while (azimuth_deg > 180.f) {
//     //  azimuth_deg -= 360.f;
//     //}
//     // window.SetCameraAzimuthDeg(azimuth_deg);
//     const glm::dvec2 xy_center = 0.5 * (problem.Lb().position + problem.Ub().position);
//     window.SetCameraFocus({xy_center.x, xy_center.y, 0});

//     const std::vector<double> cost_to_go = problem.search_.ComputeCostsToGo();
//     problem.UpdateBridgeLines(bridge_lines, cost_to_go);
//     min_cost_to_go = problem.UpdateGoalLine(goal_line, cost_to_go);
//     // UpdatePoints(points, search.tree_.points_);

//     problem.UpdateSphereLines(sphere_lines);
//     problem.UpdateBoundingBoxLines(bounding_box_lines);
//   };

//   std::function<void(const glm::mat4 &, const glm::mat4 &)> draw_visualization =
//       [&sphere_lines, &bridge_lines, &bounding_box_lines, &goal_line, &points, &window, &textbox,
//        &search_mutex, &min_cost_to_go, &problem](const glm::mat4 &view, const glm::mat4 &proj) {
//         glm::vec4 point_color = {1, 1, 1, 0.3};
//         glm::vec4 bb_color = {1, 1, 1, 0.5};
//         glm::vec4 goal_color = {1, 1, 1, 1};
//         sphere_lines.Draw(view, proj, GL_LINES);
//         bridge_lines.Draw(view, proj, GL_LINE_STRIP);
//         goal_line.Draw(view, proj, goal_color, GL_LINE_STRIP);
//         points.Draw(view, proj, point_color, GL_POINTS);
//         bounding_box_lines.Draw(view, proj, bb_color, GL_LINES);

//         // some text
//         std::stringstream message;
//         {
//           const std::lock_guard<std::mutex> lock(search_mutex);
//           message << problem.search_.Cardinality() << " nodes in tree";
//           if (min_cost_to_go > 0) {
//             message << " optimal distance is " << min_cost_to_go;
//           }
//         }
//         window.RenderText(textbox, message.str(), 25.0F, 25.0F, glm::vec3(1, 1, 1));
//       };

//   // it's theadn' time
//   std::thread thread_object(
//       [&pause, &search_mutex, &problem, &rng_engine, &sphere_lines, &bounding_box_lines]() {
//         using namespace std::chrono_literals;
//         try {
//           for (;;) {
//             int count = 0;
//             while (count < 10000) {
//               if (!pause) {
//                 const std::lock_guard<std::mutex> lock(search_mutex);
//                 for (int yolo = 0; yolo < 100; yolo++) {
//                   if (problem.search_.Step() == rrts::StepResult::kSuccess) {
//                     count++;
//                   }
//                 }
//               }
//               std::this_thread::sleep_for(1us);
//             }

//             // sleep
//             std::this_thread::sleep_for(1000ms);

//             // reset the problem to a new random one
//             {
//               const std::lock_guard<std::mutex> lock(search_mutex);
//               problem = RandomProblem(rng_engine);
//             }
//           }
//           std::cerr << "finished" << std::endl;
//         } catch (std::exception &e) {
//           std::cerr << "Thread got exception: " << e.what() << std::endl;
//           // return EXIT_FAILURE;
//         }
//       });

//   window.Run(handle_keypress, update_visualization, draw_visualization);

//   return EXIT_SUCCESS;
// }

// int main(int argc __attribute__((unused)), char *argv[]) {
//   try {
//     Run(argv[0]);  // NOLINT
//   } catch (const std::exception &e) {
//     std::cerr << e.what();
//   }
// }
