#include <emscripten/bind.h>

#include "src/problem/r3_problem.hpp"
#include "src/problem/se2_problem.hpp"

// shim class that owns an RNG state and can create random problems
class ProblemFactory {
 public:
  ProblemFactory() = default;
  R3Problem RandomR3Problem(const ProblemParameters &params) {
    return R3Problem::RandomProblem(rng_engine, params);
  };
  Se2Problem RandomSe2Problem(const ProblemParameters &params, double rho) {
    return Se2Problem::RandomProblem(rng_engine, params, rho);
  };

 private:
  std::mt19937_64 rng_engine;
};  // class ProblemFactory

EMSCRIPTEN_BINDINGS(RrtStar) {
  emscripten::value_object<XyzRgb>("XyzRgb")
      .field("x", &XyzRgb::x)
      .field("y", &XyzRgb::y)
      .field("z", &XyzRgb::z)
      .field("r", &XyzRgb::r)
      .field("g", &XyzRgb::g)
      .field("b", &XyzRgb::b)
      .field("a", &XyzRgb::a);

  emscripten::value_object<glm::vec3>("Vec3")
      .field("x", &glm::vec3::x)
      .field("y", &glm::vec3::y)
      .field("z", &glm::vec3::z);

  emscripten::value_object<glm::dvec3>("DVec3")
      .field("x", &glm::dvec3::x)
      .field("y", &glm::dvec3::y)
      .field("z", &glm::dvec3::z);

  emscripten::value_object<glm::dvec2>("DVec2")
      .field("x", &glm::dvec2::x)
      .field("y", &glm::dvec2::y);

  emscripten::value_object<ProblemParameters>("ProblemParameters")
      .field("eta", &ProblemParameters::eta)
      .field("max_num_obstacles", &ProblemParameters::max_num_obstacles)
      .field("obstacle_fraction", &ProblemParameters::obstacle_fraction)
      .field("min_length", &ProblemParameters::min_length)
      .field("max_length", &ProblemParameters::max_length)
      .field("goal_radius", &ProblemParameters::goal_radius);

  // R3Sphere
  using R3Sphere = rrts::space::r3::Sphere;
  emscripten::value_object<R3Sphere>("R3Sphere")
      .field("center", &R3Sphere::center)
      .field("radius", &R3Sphere::radius);

  // R2Sphere
  using R2Sphere = Se2Problem::Sphere;
  emscripten::value_object<R2Sphere>("R2Sphere")
      .field("center", &R2Sphere::center)
      .field("radius", &R2Sphere::radius);

  // Se2::GoalRegion
  using Se2GoalRegion = Se2Problem::GoalRegion;
  emscripten::value_object<Se2GoalRegion>("Se2GoalRegion")
      .field("position", &Se2GoalRegion::position)
      .field("min_angle", &Se2GoalRegion::min_angle)
      .field("max_angle", &Se2GoalRegion::max_angle);

  emscripten::class_<ProblemFactory>("ProblemFactory")
      .constructor<>()
      .function("RandomR3Problem", &ProblemFactory::RandomR3Problem)
      .function("RandomSe2Problem", &ProblemFactory::RandomSe2Problem);

  emscripten::class_<R3Problem>("R3Problem")
      .function("GetBridgeLines", &R3Problem::GetBridgeLines)
      .function("GetGoalLine", &R3Problem::GetGoalLine)
      .function("GetBoundingBoxLines", &R3Problem::GetBoundingBoxLines)
      .function("Step", &R3Problem::Step)
      .function("GetGoalRegion", &R3Problem::GetGoalRegion)
      .function("GetObstacles", &R3Problem::GetObstacles)
      .function("NumEdges", &R3Problem::NumEdges);

  emscripten::class_<Se2Problem>("Se2Problem")
      .function("GetBridgeLines", &Se2Problem::GetBridgeLines)
      .function("GetGoalLine", &Se2Problem::GetGoalLine)
      .function("GetBoundingBoxLines", &Se2Problem::GetBoundingBoxLines)
      .function("Step", &Se2Problem::Step)
      .function("GetGoalRegion", &Se2Problem::GetGoalRegion)
      .function("GetObstacles", &Se2Problem::GetObstacles)
      .function("NumEdges", &Se2Problem::NumEdges);

  emscripten::register_vector<XyzRgb>("vector<XyzRgb>");
  emscripten::register_vector<R3Sphere>("vector<R3Sphere>");
  emscripten::register_vector<R2Sphere>("vector<R2Sphere>");
  emscripten::register_vector<std::vector<XyzRgb> >("vector<vector<XyzRgb> >");
}
