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
  XyzRgb(float x_, float y_, float z_, float r_, float g_, float b_, float a_)
      : x(x_), y(y_), z(z_), r(r_), g(g_), b(b_), a(a_){};
  XyzRgb(const glm::vec3 &xyz, float r_, float g_, float b_, float a_)
      : x(xyz.x), y(xyz.y), z(xyz.z), r(r_), g(g_), b(b_), a(a_){};
  XyzRgb(const glm::vec3 &xyz, const glm::vec4 &rgba)
      : x(xyz.x), y(xyz.y), z(xyz.z), r(rgba.r), g(rgba.g), b(rgba.b), a(rgba.a){};
};
