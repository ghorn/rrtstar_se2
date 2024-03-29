#include <emscripten/bind.h>

#include "src/problem/r3_problem.hpp"
#include "src/problem/se2_problem.hpp"
#include "src/problem/xyzq_problem.hpp"

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
  XyzqProblem RandomXyzqProblem(const ProblemParameters &params, double rho,
                                double max_glideslope) {
    return XyzqProblem::RandomProblem(rng_engine, params, rho, max_glideslope);
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

  // Xyzq::GoalRegion
  using XyzqGoalRegion = XyzqProblem::GoalRegion;
  emscripten::value_object<XyzqGoalRegion>("XyzqGoalRegion")
      .field("position", &XyzqGoalRegion::position)
      .field("min_angle", &XyzqGoalRegion::min_angle)
      .field("max_angle", &XyzqGoalRegion::max_angle);

  emscripten::class_<ProblemFactory>("ProblemFactory")
      .constructor<>()
      .function("RandomR3Problem", &ProblemFactory::RandomR3Problem)
      .function("RandomSe2Problem", &ProblemFactory::RandomSe2Problem)
      .function("RandomXyzqProblem", &ProblemFactory::RandomXyzqProblem);

  emscripten::class_<R3Problem>("R3Problem")
      .function("SetPathLines", &R3Problem::SetPathLines)
      .function("GetBoundingBoxLines", &R3Problem::GetBoundingBoxLines)
      .function("Step", &R3Problem::Step)
      .function("GetGoalRegion", &R3Problem::GetGoalRegion)
      .function("GetObstacles", &R3Problem::GetObstacles)
      .function("NumEdges", &R3Problem::NumEdges);

  emscripten::class_<Se2Problem>("Se2Problem")
      .function("SetPathLines", &Se2Problem::SetPathLines)
      .function("GetBoundingBoxLines", &Se2Problem::GetBoundingBoxLines)
      .function("Step", &Se2Problem::Step)
      .function("GetGoalRegion", &Se2Problem::GetGoalRegion)
      .function("GetObstacles", &Se2Problem::GetObstacles)
      .function("NumEdges", &Se2Problem::NumEdges);

  emscripten::class_<XyzqProblem>("XyzqProblem")
      .function("SetPathLines", &XyzqProblem::SetPathLines)
      .function("GetBoundingBoxLines", &XyzqProblem::GetBoundingBoxLines)
      .function("Step", &XyzqProblem::Step)
      .function("GetGoalRegion", &XyzqProblem::GetGoalRegion)
      .function("GetObstacles", &XyzqProblem::GetObstacles)
      .function("NumEdges", &XyzqProblem::NumEdges);

  // ----------- dubins ----------------
  using rrts::dubins::Se2Coord;
  emscripten::value_object<Se2Coord>("Se2Coord")
      .field("position", &Se2Coord::position)
      .field("theta", &Se2Coord::theta);

  using rrts::dubins::DubinsWordStatus;
  emscripten::enum_<DubinsWordStatus>("DubinsWordStatus")
      .value("kSuccess", DubinsWordStatus::kSuccess)
      .value("kNoPath", DubinsWordStatus::kNoPath);

  using rrts::dubins::DubinsStatus;
  emscripten::enum_<DubinsStatus>("DubinsStatus")
      .value("kSuccess", DubinsStatus::kSuccess)
      .value("kNoPath", DubinsStatus::kNoPath);

  using rrts::dubins::DubinsPathType;
  emscripten::enum_<DubinsPathType>("DubinsPathType")
      .value("kLsl", DubinsPathType::kLsl)
      .value("kLsr", DubinsPathType::kLsr)
      .value("kRsl", DubinsPathType::kRsl)
      .value("kRsr", DubinsPathType::kRsr)
      .value("kRlr", DubinsPathType::kRlr)
      .value("kLrl", DubinsPathType::kLrl);

  using rrts::dubins::DubinsIntermediateResults;
  emscripten::class_<DubinsIntermediateResults>("DubinsIntermediateResults");  // opaque

  emscripten::function("ComputeDubinsIntermediateResults",
                       &rrts::dubins::ComputeDubinsIntermediateResults);
  using rrts::dubins::DubinsIntermediateResults;
  using rrts::dubins::DubinsPath;
  emscripten::function("ComputeDubinsPath",
                       emscripten::select_overload<DubinsWordStatus(
                           DubinsPath &, const Se2Coord &, const Se2Coord &, double, DubinsPathType,
                           const DubinsIntermediateResults &)>(&rrts::dubins::ComputeDubinsPath));

  using rrts::dubins::DubinsPath;
  emscripten::class_<DubinsPath>("DubinsPath")
      .constructor<>()
      .function("TotalLength", &DubinsPath::TotalLength)
      .function("Sample", &DubinsPath::Sample);

  // ------------ vectors --------------
  emscripten::register_vector<XyzRgb>("vector<XyzRgb>");
  emscripten::register_vector<R3Sphere>("vector<R3Sphere>");
  emscripten::register_vector<R2Sphere>("vector<R2Sphere>");
  emscripten::register_vector<std::vector<XyzRgb> >("vector<vector<XyzRgb> >");
}
