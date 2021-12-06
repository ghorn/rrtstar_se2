#pragma once

#include "src/tagged.hpp"
#include <vector>

namespace rrts::tree {

    template <typename Point>
    struct TreeBase {
      TreeBase() = default;;
      virtual ~TreeBase() = default;

      virtual void Insert(const Tagged<Point> &new_point) = 0;
      virtual Tagged<Point> Nearest(const Point &test_point) const = 0;
      virtual std::vector<Tagged<Point>> Near(const Point &test_point, double radius) const = 0;
      [[nodiscard]] virtual double Cardinality() const = 0;
    };

  } // namespace rrts::tree
