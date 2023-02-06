#include <emscripten/bind.h>

#include "src/problem/r3_problem.hpp"

// class HelloClass {
//  public:
//   static std::string SayHello(const std::string &name) { return "Yo! " + name; };
// };

// int say_hello() {
//   printf("Hello from your wasm module\n");
//   return 0;
// }

EMSCRIPTEN_BINDINGS(RrtStar) {
  // emscripten::class_<HelloClass>("HelloClass")
  //     .constructor<>()
  //     .class_function("SayHello", &HelloClass::SayHello);
  // emscripten::function("sayHello2", &say_hello);

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

  // R3Sphere
  using R3Sphere = rrts::space::r3::Sphere;
  emscripten::value_object<R3Sphere>("R3Sphere")
      .field("center", &R3Sphere::center)
      .field("radius", &R3Sphere::radius);

  emscripten::class_<R3ProblemFactory>("R3ProblemFactory")
      .constructor<>()
      .function("RandomProblem", &R3ProblemFactory::RandomProblem);

  emscripten::class_<R3Problem>("R3Problem")
      // .constructor<const>()
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
