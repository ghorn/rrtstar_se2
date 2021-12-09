#pragma once

#include <cstddef>

namespace rrts {
template <class P>
class Tagged {
 public:
  Tagged(size_t index, const P& point) : index_(index), point_(point) {}
  size_t Index() const { return index_; }
  const P& Point() const { return point_; }

 private:
  size_t index_{};
  P point_{};
};
}  // namespace rrts
