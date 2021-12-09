#include <gtest/gtest.h>  // for Test, EXPECT_EQ, Message, TestPartResult, SuiteAp...

#include <cmath>   // for M_PI
#include <memory>  // for allocator

#include "src/space/dubins/dubins.hpp"  // for AngleDifference

namespace rrts {

// NOLINTNEXTLINE
TEST(AngleDifference, Identities) {
  EXPECT_EQ(dubins::AngleDifference(M_PI, 0), -M_PI);
  EXPECT_EQ(dubins::AngleDifference(-M_PI, 0), M_PI);
  EXPECT_EQ(dubins::AngleDifference(0, M_PI), M_PI);
  EXPECT_EQ(dubins::AngleDifference(0, -M_PI), -M_PI);
}

// NOLINTNEXTLINE
TEST(AngleDifference, half_pi) {
  EXPECT_EQ(dubins::AngleDifference(1.5 * M_PI, M_PI), 1.5707963267948966);
  EXPECT_EQ(dubins::AngleDifference(3.5 * M_PI, M_PI), 1.5707963267948966);
  EXPECT_EQ(dubins::AngleDifference(5.5 * M_PI, M_PI), 1.5707963267948948);
  EXPECT_EQ(dubins::AngleDifference(7.5 * M_PI, M_PI), 1.5707963267948948);
  EXPECT_EQ(dubins::AngleDifference(M_PI, 1.5 * M_PI), -1.5707963267948966);
  EXPECT_EQ(dubins::AngleDifference(M_PI, 3.5 * M_PI), -1.5707963267948966);
  EXPECT_EQ(dubins::AngleDifference(M_PI, 5.5 * M_PI), -1.5707963267948948);
  EXPECT_EQ(dubins::AngleDifference(M_PI, 7.5 * M_PI), -1.5707963267948948);
}

}  // namespace rrts
