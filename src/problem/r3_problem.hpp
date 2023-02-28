#pragma once

#ifdef __EMSCRIPTEN__
#include <emscripten/bind.h>
#endif

#include "src/problem/parameters.hpp"
#include "src/problem/xyz_rgb.hpp"
#include "src/search.hpp"     // for Edge, Search, StepResult, StepResult::kSuccess
#include "src/space/r3.hpp"   // for Sphere, R3Point, Line, R3
#include "src/tree/fast.hpp"  // for Fast

struct R3Problem {
  using Line = rrts::space::r3::Line;
  using Bridge = Line;
  using R3Point = rrts::space::r3::R3Point;
  using Sphere = rrts::space::r3::Sphere;
  // using Tree = rrts::tree::Naive<R3Point, Bridge, 3>;  // comment in for testing Fast tree
  using Tree = rrts::tree::Fast<R3Point, Bridge, 3>;
  using Space = rrts::space::r3::R3;

  R3Problem(const R3Point &x_init, const R3Point &lb, const R3Point &ub, const Sphere &goal_region,
            const std::vector<Sphere> &obstacles, double eta)
      : lb_(lb),
        ub_(ub),
        goal_region_(goal_region),
        obstacles_(obstacles),
        r3_space_(lb, ub, obstacles),
        search_(x_init, r3_space_, eta){};
  R3Point lb_;
  R3Point ub_;
  Sphere goal_region_;
  std::vector<Sphere> obstacles_;
  Space r3_space_;
  rrts::Search<R3Point, Line, 3, Tree, Space> search_;

  Sphere GetGoalRegion() const { return goal_region_; }
  std::vector<Sphere> GetObstacles() const { return obstacles_; }

  bool Step() { return search_.Step() == rrts::StepResult::kSuccess; }

  size_t NumEdges() const { return search_.Edges().size(); }

  std::vector<std::vector<XyzRgb>> ComputeBridgeLines(
      const std::vector<double> &cost_to_go,
      const std::vector<rrts::Edge<R3Point, Line>> &edges) const;
  std::vector<std::vector<XyzRgb>> ComputeGoalLine(
      const std::vector<double> &cost_to_go, const std::vector<rrts::Edge<R3Point, Line>> &edges,
      const glm::vec3 &color) const;

#ifdef __EMSCRIPTEN__
  void SetPathLines(const glm::vec3 &goal_line_color, emscripten::val positions,
                    emscripten::val colors, emscripten::val indices, size_t max_points) const;
#endif

  [[nodiscard]] bool InGoalRegion(const R3Point &p) const {
    double dist = glm::distance(glm::dvec3(p), goal_region_.center);
    return dist <= goal_region_.radius;
  }

  std::vector<std::vector<XyzRgb>> GetBoundingBoxLines(float bounding_box_opacity) const;

  static R3Problem RandomProblem(std::mt19937_64 &rng_engine, const ProblemParameters &params);
};
