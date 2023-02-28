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
#include "src/space/xyzq.hpp"           // for Sphere, Se2, Se2Coord
#include "src/tree/fast.hpp"            // for Fast

struct XyzqProblem {
  using DubinsPath = rrts::dubins::DubinsPath;
  using XyzqCoord = rrts::space::xyzq::XyzqCoord;
  using XyzqPath = rrts::space::xyzq::XyzqPath;
  using Point = rrts::space::xyzq::XyzqCoord;
  using R3Sphere = rrts::space::xyzq::R3Sphere;
  // using Tree = rrts::tree::Naive<Point, XyzqPath, 4>;  // comment in for testing Fast tree
  using Tree = rrts::tree::Fast<Point, XyzqPath, 4>;
  using Space = rrts::space::xyzq::Xyzq;

  using DubinsStatus = rrts::dubins::DubinsStatus;

  struct GoalRegion {
    R3Sphere position;
    double min_angle;
    double max_angle;
  };

  XyzqProblem(const Point &x_init, double rho, double max_glideslope, double eta,
              const glm::dvec3 &lb, const glm::dvec3 &ub, const R3Sphere &goal_region,
              const std::vector<R3Sphere> &obstacles)
      : goal_region_{goal_region, -90.0 * M_PI / 180.0, 90.0 * M_PI / 180.0},
        obstacles_(obstacles),
        xyzq_space_(rho, max_glideslope, lb, ub, obstacles),
        search_(x_init, xyzq_space_, eta){};
  GoalRegion goal_region_;
  std::vector<R3Sphere> obstacles_;
  Space xyzq_space_;
  rrts::Search<Point, XyzqPath, 3, Tree, Space> search_;

  [[nodiscard]] const Point &Lb() const { return xyzq_space_.Lb(); };
  [[nodiscard]] const Point &Ub() const { return xyzq_space_.Ub(); };

  std::vector<std::vector<XyzRgb>> ComputeBridgeLines(
      const std::vector<double> &cost_to_go,
      const std::vector<rrts::Edge<Point, XyzqPath>> &edges) const;
  std::vector<std::vector<XyzRgb>> ComputeGoalLine(
      const std::vector<double> &cost_to_go, const std::vector<rrts::Edge<Point, XyzqPath>> &edges,
      const glm::vec3 &color) const;

 public:
#ifdef __EMSCRIPTEN__
  void SetPathLines(const glm::vec3 &goal_line_color, emscripten::val positions,
                    emscripten::val colors, emscripten::val indices, size_t max_points) const;
#endif

  std::vector<std::vector<XyzRgb>> GetBoundingBoxLines(float bounding_box_opacity) const;

  [[nodiscard]] bool InGoalRegion(const Point &p) const {
    const bool heading_good =
        goal_region_.min_angle <= p.se2.theta && p.se2.theta <= goal_region_.max_angle;
    const bool xyz_good =
        sqrt(glm::distance2(p.Xyz(), goal_region_.position.center)) <= goal_region_.position.radius;
    const bool in_goal = xyz_good && heading_good;

    return in_goal;
  }

  GoalRegion GetGoalRegion() const { return goal_region_; }
  std::vector<R3Sphere> GetObstacles() const { return obstacles_; }

  bool Step() { return search_.Step() == rrts::StepResult::kSuccess; }

  size_t NumEdges() const { return search_.Edges().size(); }

  static XyzqProblem RandomProblem(std::mt19937_64 &rng_engine, const ProblemParameters &params,
                                   double rho, double max_glideslope);
};  // class XyzqProblem
