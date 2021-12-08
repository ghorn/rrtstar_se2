#pragma once

#include <cstdio>   // for fprintf, stderr
#include <cstdlib>  // for exit, EXIT_FAILURE
#include <iostream>

namespace rrts {

[[noreturn]] void ExitThreadSafe(int exit_code);

#define ASSERT(expr)                                                                             \
  {                                                                                              \
    if (!expr) {                                                                                 \
      fprintf(stderr, "\n  Assertion '" #expr "' failed at %s, line %d.\n", __FILE__, __LINE__); \
      rrts::ExitThreadSafe(EXIT_FAILURE);                                                        \
    }                                                                                            \
  }

#define ASSERT_MSG(expr, msg)                                                                \
  {                                                                                          \
    if (!(expr)) {                                                                           \
      std::cerr << std::endl                                                                 \
                << "  Assertion '" #expr "' failed at " << __FILE__ << ", line " << __LINE__ \
                << "." << std::endl;                                                         \
      std::cerr << msg << std::endl;                                                         \
      rrts::ExitThreadSafe(EXIT_FAILURE);                                                    \
    }                                                                                        \
  }

#define FAIL_MSG(msg)                                                     \
  {                                                                       \
    fprintf(stderr, "\n  Failure at %s, line %d.\n", __FILE__, __LINE__); \
    std::cerr << msg << std::endl;                                        \
    rrts::ExitThreadSafe(EXIT_FAILURE);                                   \
  }

};  // namespace rrts

// stock/rsus - some flexibility
//
// north of 400k total
