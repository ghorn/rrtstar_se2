#include <emscripten/bind.h>

#include "src/problem/r3_problem.hpp"

// shim class that owns an RNG state and can create random problems
class ProblemFactory {
 public:
  ProblemFactory() = default;
  R3Problem RandomR3Problem(const R3Problem::Parameters &params) {
    return R3Problem::RandomProblem(rng_engine, params);
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

  emscripten::value_object<glm::dvec3>("DVec3")
      .field("x", &glm::dvec3::x)
      .field("y", &glm::dvec3::y)
      .field("z", &glm::dvec3::z);

  emscripten::value_object<R3Problem::Parameters>("R3Params")
      .field("eta", &R3Problem::Parameters::eta)
      .field("max_num_obstacles", &R3Problem::Parameters::max_num_obstacles)
      .field("obstacle_fraction", &R3Problem::Parameters::obstacle_fraction)
      .field("min_length", &R3Problem::Parameters::min_length)
      .field("max_length", &R3Problem::Parameters::max_length);

  // R3Sphere
  using R3Sphere = rrts::space::r3::Sphere;
  emscripten::value_object<R3Sphere>("R3Sphere")
      .field("center", &R3Sphere::center)
      .field("radius", &R3Sphere::radius);

  emscripten::class_<ProblemFactory>("ProblemFactory")
      .constructor<>()
      .function("RandomR3Problem", &ProblemFactory::RandomR3Problem);

  emscripten::class_<R3Problem>("R3Problem")
      .function("GetBridgeLines", &R3Problem::GetBridgeLines)
      .function("GetGoalLine", &R3Problem::GetGoalLine)
      .function("Step", &R3Problem::Step)
      .function("GetGoalRegion", &R3Problem::GetGoalRegion)
      .function("GetObstacles", &R3Problem::GetObstacles)
      .function("NumEdges", &R3Problem::NumEdges);

  emscripten::register_vector<XyzRgb>("vector<XyzRgb>");
  emscripten::register_vector<R3Sphere>("vector<R3Sphere>");
  emscripten::register_vector<std::vector<XyzRgb> >("vector<vector<XyzRgb> >");
}
