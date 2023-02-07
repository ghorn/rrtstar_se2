#pragma once

#include "src/search.hpp"     // for Edge, Search, StepResult, StepResult::kSuccess
#include "src/space/r3.hpp"   // for Sphere, Point, Line, R3
#include "src/tree/fast.hpp"  // for Fast

struct XyzRgb {
  float x;
  float y;
  float z;
  float r;
  float g;
  float b;
  float a;
  XyzRgb() = default;
  XyzRgb(float x_, float y_, float z_, float r_, float g_, float b_, float a_)
      : x(x_), y(y_), z(z_), r(r_), g(g_), b(b_), a(a_){};
  XyzRgb(glm::vec3 xyz, float r_, float g_, float b_, float a_)
      : x(xyz.x), y(xyz.y), z(xyz.z), r(r_), g(g_), b(b_), a(a_){};
};

struct R3Problem {
  using Line = rrts::space::r3::Line;
  using Bridge = Line;
  using Point = rrts::space::r3::Point;
  using Sphere = rrts::space::r3::Sphere;
  // using Tree = rrts::tree::Naive<Point, Bridge, 3>;  // comment in for testing Fast tree
  using Tree = rrts::tree::Fast<Point, Bridge, 3>;
  using Space = rrts::space::r3::R3;

  struct Parameters {
    double eta;
    int32_t max_num_obstacles;
    double obstacle_fraction;
    double min_length;
    double max_length;
  };

  R3Problem(const Point &x_init, const Point &lb, const Point &ub, const Sphere &goal_region,
            const std::vector<Sphere> &obstacles, double eta)
      : lb_(lb),
        ub_(ub),
        goal_region_(goal_region),
        obstacles_(obstacles),
        r3_space_(lb, ub, obstacles),
        search_(x_init, r3_space_, eta){};
  Point lb_;
  Point ub_;
  Sphere goal_region_;
  std::vector<Sphere> obstacles_;
  Space r3_space_;
  rrts::Search<Point, Line, 3, Tree, Space> search_;

  Sphere GetGoalRegion() const { return goal_region_; }
  std::vector<Sphere> GetObstacles() const { return obstacles_; }

  bool Step() {
    if (search_.Step() == rrts::StepResult::kSuccess) {
      return true;
    }
    return false;
  }

  size_t NumEdges() const { return search_.Edges().size(); }

  std::vector<std::vector<XyzRgb> > GetBridgeLines() const;

  [[nodiscard]] bool InGoalRegion(const Point &p) const {
    double dist = glm::distance(glm::dvec3(p), goal_region_.center);
    return dist <= goal_region_.radius;
  }

  std::vector<std::vector<XyzRgb> > GetGoalLine() const;

  static R3Problem RandomProblem(std::mt19937_64 &rng_engine, const Parameters &params);
};

class R3ProblemFactory {
 public:
  R3ProblemFactory() = default;
  R3Problem RandomProblem(const R3Problem::Parameters &params) {
    return R3Problem::RandomProblem(rng_engine, params);
  };

 private:
  std::mt19937_64 rng_engine;
};
