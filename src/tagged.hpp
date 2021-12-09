#pragma once

#include <cstddef>

namespace rrts {
template <class P>
class Tagged {
public:
//  Tagged(size_t index, const P &point) : index_(index), point_(point) {}
  size_t Index() const {return index;}
  const P& Point() const {return point;}

  // TODO(greg): make private
//private:
  size_t index{};
  P point{};
};
}  // namespace rrts
