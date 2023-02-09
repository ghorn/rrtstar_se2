#pragma once

#include <glm/glm.hpp>

struct XyzRgb {
  float x;
  float y;
  float z;
  float r;
  float g;
  float b;
  float a;
  XyzRgb() = default;
  XyzRgb(const glm::vec3 &xyz, const glm::vec4 &rgba)
      : x(xyz.x), y(xyz.y), z(xyz.z), r(rgba.r), g(rgba.g), b(rgba.b), a(rgba.a){};
};
