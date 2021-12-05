#pragma once

#include <vector>
#include "src/tagged.hpp"

namespace rrts {
  namespace tree {

    template <typename Point>
    struct TreeBase {
      TreeBase() {};
      virtual ~TreeBase() = default;

      virtual void Insert(const Tagged<Point> &new_point) = 0;
      virtual Tagged<Point> Nearest(const Point &test_point) const = 0;
      virtual std::vector<Tagged<Point>> Near(const Point &test_point, const double radius) const = 0;
      virtual double Cardinality() const = 0;
    };

  } // namespace tree
} // namespace rrts
