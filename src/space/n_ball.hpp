#pragma once

#include <cassert>

constexpr double VolumeOfNBall(int n, double radius) {
  assert(n > 0);
  switch (n) {
  case 0:
    return 1;
  case 1:
    return 2 * radius;
  default:
    return 2 * M_PI / static_cast<double>(n) * radius * radius * VolumeOfNBall(n - 2, radius);
  }
}
