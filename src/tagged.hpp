#pragma once

namespace rrts {
  template <class Point>
  struct Tagged {
    size_t index;
    Point point;
  };
}
