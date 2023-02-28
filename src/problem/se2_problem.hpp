#pragma once

#include <functional>  // for function
#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>

#ifdef __EMSCRIPTEN__
#include <emscripten/bind.h>
#endif

#include "src/problem/parameters.hpp"
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

  Se2Problem(const Point &x_init, double rho, double eta, const glm::dvec2 &lb,
             const glm::dvec2 &ub, const Sphere &goal_region, const std::vector<Sphere> &obstacles)
      : goal_region_{goal_region, -90.0 * M_PI / 180.0, 90.0 * M_PI / 180.0},
        obstacles_(obstacles),
        se2_space_(rho, lb, ub, obstacles),
        search_(x_init, se2_space_, eta){};
  GoalRegion goal_region_;
  std::vector<Sphere> obstacles_;
  Space se2_space_;
  rrts::Search<Point, Line, 3, Tree, Space> search_;

  [[nodiscard]] const Point &Lb() const { return se2_space_.Lb(); };
  [[nodiscard]] const Point &Ub() const { return se2_space_.Ub(); };

  std::vector<std::vector<XyzRgb>> ComputeBridgeLines(
      const std::vector<double> &cost_to_go,
      const std::vector<rrts::Edge<Point, Line>> &edges) const;
  std::vector<std::vector<XyzRgb>> ComputeGoalLine(
      const std::vector<double> &cost_to_go, const std::vector<rrts::Edge<Point, Line>> &edges,
      const glm::vec3 &color) const;

 public:
#ifdef __EMSCRIPTEN__
  void SetPathLines(const glm::vec3 &goal_line_color, emscripten::val positions,
                    emscripten::val colors, emscripten::val indices, size_t max_points) const;
#endif

  std::vector<std::vector<XyzRgb>> GetBoundingBoxLines(float bounding_box_opacity) const;

  [[nodiscard]] bool InGoalRegion(const Point &p) const {
    const bool heading_good =
        goal_region_.min_angle <= p.theta && p.theta <= goal_region_.max_angle;
    const bool xy_good = sqrt(glm::distance2(p.position, goal_region_.position.center)) <=
                         goal_region_.position.radius;
    const bool in_goal = xy_good && heading_good;

    return in_goal;
  }

  GoalRegion GetGoalRegion() const { return goal_region_; }
  std::vector<Sphere> GetObstacles() const { return obstacles_; }

  bool Step() { return search_.Step() == rrts::StepResult::kSuccess; }

  size_t NumEdges() const { return search_.Edges().size(); }

  static Se2Problem RandomProblem(std::mt19937_64 &rng_engine, const ProblemParameters &params,
                                  double rho);
};  // class Se2Problem
