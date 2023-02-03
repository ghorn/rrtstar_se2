#include <emscripten/bind.h>

class HelloClass {
 public:
  static std::string SayHello(const std::string &name) { return "Yo! " + name; };
};

int say_hello() {
  printf("Hello from your wasm module\n");
  return 0;
}

EMSCRIPTEN_BINDINGS(Hello) {
  emscripten::class_<HelloClass>("HelloClass")
      .constructor<>()
      .class_function("SayHello", &HelloClass::SayHello);
  emscripten::function("sayHello2", &say_hello);
}
