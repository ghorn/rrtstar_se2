#pragma once

#include <array>
#include <vector>

#include "src/tagged.hpp"

namespace rrts::tree {

template <typename Point, size_t D>
struct TreeBase {
  explicit TreeBase(std::array<bool, D> periodic) : periodic_(std::move(periodic)){};
  virtual ~TreeBase() = default;

  virtual void Insert(const Tagged<Point> &new_point) = 0;
  virtual Tagged<Point> Nearest(const Point &test_point) const = 0;
  virtual std::vector<Tagged<Point>> Near(const Point &test_point, double radius) const = 0;
  [[nodiscard]] virtual double Cardinality() const = 0;
  [[nodiscard]] const std::array<bool, D> &Periodic();

 private:
  std::array<bool, D> periodic_;
};

}  // namespace rrts::tree
