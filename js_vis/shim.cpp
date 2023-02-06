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

  emscripten::class_<R3ProblemFactory>("R3ProblemFactory")
      .constructor<>()
      .function("RandomProblem", &R3ProblemFactory::RandomProblem);

  emscripten::class_<R3Problem>("R3Problem")
      // .constructor<const>()
      .function("GetBridgeLines", &R3Problem::GetBridgeLines)
      .function("GetGoalLine", &R3Problem::GetGoalLine)
      .function("Step", &R3Problem::Step)
      .function("NumEdges", &R3Problem::NumEdges);

  emscripten::register_vector<XyzRgb>("vector<XyzRgb>");
  emscripten::register_vector<std::vector<XyzRgb> >("vector<vector<XyzRgb> >");
}
