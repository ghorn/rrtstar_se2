#include <bits/exception.h>  // for exception
#include <sys/types.h>       // for key_t, uint

#include <algorithm>   // for copy, max
#include <chrono>      // for operator""s, chrono_literals
#include <cstdio>      // for fprintf, stderr
#include <cstdlib>     // for EXIT_SUCCESS
#include <functional>  // for function
#include <iostream>    // for operator<<, basic_ostream, cerr, endl, ostream, cha...
#include <mutex>       // for mutex, lock_guard
#include <optional>    // for optional, nullopt
#include <queue>       // for queue
#include <sstream>
#include <thread>  // for sleep_for, thread
#include <vector>  // for vector

#include "bb3d/opengl_context.hpp"  // for Window
#include "bb3d/shader/colorlines.hpp"
#include "bb3d/shader/lines.hpp"
#include "src/space/dubins/dubins.hpp"
#include "src/tree/fast.hpp"
#include "src/tree/naive.hpp"

using Se2Coord = rrts::dubins::Se2Coord;
using DubinsPath = rrts::dubins::DubinsPath;
using DubinsPathType = rrts::dubins::DubinsPathType;
using DubinsWordStatus = rrts::dubins::DubinsWordStatus;
using DubinsStatus = rrts::dubins::DubinsStatus;

// std::array<double, 3> DrawDubinsSegment(double t, const std::array<double, 3> &qi, SegmentType
// type) {
//  const double st = sin(qi[2]);
//  const double ct = cos(qi[2]);
//
//  std::array<double, 3> qt = {0.0, 0.0, 0.0};
//
//  if (type == SegmentType::kLSeg) {
//    qt[0] = +sin(qi[2] + t) - st;
//    qt[1] = -cos(qi[2] + t) + ct;
//    qt[2] = t;
//  } else if (type == SegmentType::kRSeg) {
//    qt[0] = -sin(qi[2] - t) + st;
//    qt[1] = +cos(qi[2] - t) - ct;
//    qt[2] = -t;
//  } else if (type == SegmentType::kSSeg) {
//    qt[0] = ct * t;
//    qt[1] = st * t;
//    qt[2] = 0.0;
//  }
//  qt[0] += qi[0];
//  qt[1] += qi[1];
//  qt[2] += qi[2];
//
//  return qt;
//}

// std::vector<bb3d::ColoredVec3> DrawSegment(std::vector<bb3d::ColoredVec3> &tracks,
//                                              int points_per_circle,
//                                              double z,
//                                           const glm::vec4 &color0,
//                                           const glm::vec4 &color1,
//                                           double t0, double t1,
//                                           const DubinsPath &path,
//                                           double z, int points_per_circle) {
//
//}
//
// std::vector<bb3d::ColoredVec3> DubinsPathDraw(const DubinsPath &path,
//                                              int points_per_circle,
//                                              double z,
//                                              const glm::vec4 &color0,
//                                              const glm::vec4 &color1) {
//  /* initial configuration */
//  /* The translated initial configuration */
//  const std::array<double, 3> qi = {0.0, 0.0, path.qi[2]};
//
//  /* generate the target configuration */
//  const double p1 = path.param[0];
//  const double p2 = path.param[1];
//  const double p3 = path.param[2];
//  /* end-of segment 1 */
//  std::array<double, 3> q1 = DubinsSegment(p1, qi, types[0]);
//  /* end-of segment 2 */
//  std::array<double, 3> q2 = DubinsSegment(p2, q1, types[1]);
//
//  {{0, p1}, qi, types[0]}
//  {{p1, p1 + p2}, q1, types[1]}
//  {{p1+p2, p1+p2+p3}, q2, types[2]}
//
//  std::vector<bb3d::ColoredVec3> track;
//  track.DrawSegment(track, points_per_circle, z, color0, color1, 0, p1, qi, types[0]);
//
//  if (tprime < p1) {
//    q = DrawDubinsSegment(tprime, qi, types[0]);
//  } else if (tprime < (p1 + p2)) {
//    q = DrawDubinsSegment(tprime - p1, q1, types[1]);
//  } else {
//    q = DrawDubinsSegment(tprime - p1 - p2, q2, types[2]);
//  }
//
////  if (tprime < p1) {
////    q = DrawDubinsSegment(tprime, qi, types[0]);
////  } else if (tprime < (p1 + p2)) {
////    q = DrawDubinsSegment(tprime - p1, q1, types[1]);
////  } else {
////    q = DrawDubinsSegment(tprime - p1 - p2, q2, types[2]);
////  }
//
//  /* scale the target configuration, translate back to the original starting point */
//  q[0] = q[0] * path.rho + path.qi[0];
//  q[1] = q[1] * path.rho + path.qi[1];
//  q[2] = Mod2pi(q[2]);
//
//
//
//  SegmentType::kSSeg
//
//  const std::array<SegmentType, 3> types = kDirdata.at(static_cast<size_t>(path.type));
//
//}

struct SixPaths {
  SixPaths(const Se2Coord &c0, const Se2Coord &c1, double rho) {
    for (size_t k = 0; k < 6; k++) {
      auto path_type = static_cast<DubinsPathType>(k);
      DubinsWordStatus errcode = ComputeDubinsPath(paths[k], c0, c1, rho, path_type);
      if (errcode == DubinsWordStatus::kSuccess) {
        path_valid.at(k) = true;
      } else {
        path_valid.at(k) = false;
      }
    }

    shortest_path = DubinsPath(c0, c1, rho);
  }

  DubinsPath shortest_path{};
  std::array<DubinsPath, 6> paths{};
  std::array<bool, 6> path_valid{};

  std::vector<std::vector<bb3d::ColoredVec3> > DrawPaths() {
    std::vector<std::vector<bb3d::ColoredVec3> > all_paths;
    std::vector<glm::vec4> colors;
    colors.emplace_back(1, 0, 0, 1);
    colors.emplace_back(0, 1, 0, 1);
    colors.emplace_back(0, 0, 1, 1);
    colors.emplace_back(0, 1, 1, 1);
    colors.emplace_back(1, 0, 1, 1);
    colors.emplace_back(1, 1, 0, 1);

    for (size_t k = 0; k < 6; k++) {
      if (path_valid[k]) {
        // std::cerr << "path " << k << " valid, cost " << paths[k].total_length << std::endl;
        all_paths.push_back(DrawPath(paths.at(k), colors.at(k), 0.02 * static_cast<double>(k)));
      } else {
        // std::cerr << "path " << k << " INVALID" << std::endl;
      }
    }
    all_paths.push_back(DrawPath(shortest_path, {1, 1, 1, 1}, -0.5));
    return all_paths;
  }

 private:
  static std::vector<bb3d::ColoredVec3> DrawPath(const DubinsPath &path, const glm::vec4 &color,
                                                 double z) {
    std::vector<bb3d::ColoredVec3> line;

    for (int j = 0; j < 50; j++) {
      double t = 0.999 * path.TotalLength() * static_cast<double>(j) / (50 - 1);  // static_cast)
      Se2Coord q = path.Sample(t);
      // if (first) {
      //  std::cout << "t " << t << ": " << q[0] << ", " << q[1] << std::endl;
      //}
      bb3d::ColoredVec3 v = {{q[0], q[1], z}, color};
      line.push_back(v);
    }
    return line;
  }
};

int Run(char *argv0) {
  //  std::mt19937_64 rng_engine;  // NOLINT
  //  Problem problem = RandomProblem(rng_engine);
  //
  //  problem.Describe();

  bb3d::Window window(argv0);
  bb3d::ColorLines color_lines;
  bb3d::Freetype textbox(18);
  window.SetCameraFocus({0, 0, 0});
  window.SetCameraAzimuthDeg(180);
  window.SetCameraElevationDeg(80);

  bool pause = false;
  std::function<void(key_t)> handle_keypress = [&window, &pause](key_t key) {
    if (key == 80) {
      pause = !pause;
      std::cout << "pausing " << pause << std::endl;
    };
  };

  std::string message_string;
  int counter = 0;
  std::function<void()> update_visualization = [&counter, &message_string, &color_lines]() {
    double t = static_cast<double>(counter) / 60.0;
    Se2Coord configuration0 = {{1, 1}, 0.1 * t};
    Se2Coord configuration_f = {{1 + 3.5 * sin(t), 1 + cos(0.2 * t)}, 0.2 * t};
    const double rho = 1;
    SixPaths six_paths(configuration0, configuration_f, rho);
    std::vector<std::vector<bb3d::ColoredVec3> > lines = six_paths.DrawPaths();
    color_lines.Update(lines);
    counter++;
    std::stringstream message;
    const DubinsPath &best = six_paths.shortest_path;
    message << "best path " << best.Type() << " , cost " << best.TotalLength();
    message_string = message.str();
  };

  std::function<void(const glm::mat4 &, const glm::mat4 &)> draw_visualization =
      [&color_lines, &window, &textbox, &message_string](const glm::mat4 &view,
                                                         const glm::mat4 &proj) {
        color_lines.Draw(view, proj, GL_LINE_STRIP);
        // some text
        window.RenderText(textbox, message_string, 25.0F, 25.0F, glm::vec3(1, 1, 1));
      };

  //  // it's theadn' time
  //  std::thread thread_object(
  //      [&pause, &search_mutex, &problem, &rng_engine, &sphere_lines, &bounding_box_lines]() {
  //        using namespace std::chrono_literals;
  //        for (;;) {
  //          int count = 0;
  //          while (count < 20000) {
  //            if (!pause) {
  //              const std::lock_guard<std::mutex> lock(search_mutex);
  //              for (int yolo = 0; yolo < 100; yolo++) {
  //                if (problem.search_.Step() == rrts::StepResult::kSuccess) {
  //                  count++;
  //                }
  //              }
  //            }
  //            std::this_thread::sleep_for(1us);
  //          }
  //
  //          // sleep
  //          std::this_thread::sleep_for(500ms);
  //
  //          // reset the problem to a new random one
  //          {
  //            const std::lock_guard<std::mutex> lock(search_mutex);
  //            problem = RandomProblem(rng_engine);
  //          }
  //        }
  //        std::cerr << "finished" << std::endl;
  //      });

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
