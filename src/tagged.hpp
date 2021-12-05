#pragma once

#include <cstddef>

namespace rrts {
  template <class Point>
  struct Tagged {
    size_t index;
    Point point;
  };
} // namespace rrts
