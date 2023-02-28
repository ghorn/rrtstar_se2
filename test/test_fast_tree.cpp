#include <gtest/gtest.h>
#include <stdio.h>  // for size_t, fprintf, stderr

#include <cassert>           // for assert
#include <chrono>            // for duration, operator-, high_resolution_clock, time_point
#include <cmath>             // for pow, log
#include <cstdlib>           // for exit, EXIT_FAILURE, EXIT_SUCCESS
#include <glm/glm.hpp>       // for vec<>::(anonymous), vec, dot, operator-, dvec3
#include <glm/gtx/norm.hpp>  // for distance2
#include <iostream>          // for operator<<, basic_ostream, endl, basic_ostream<>::__...
#include <random>            // for mt19937_64, uniform_real_distribution
#include <set>               // for set, operator!=, operator==, _Rb_tree_const_iterator
#include <tuple>             // for get, tuple
#include <unordered_map>     // for unordered_map
#include <utility>           // for __tuple_element_t
#include <vector>            // for vector

#include "src/space/n_ball.hpp"      // for VolumeOfNBall
#include "src/space/r3.hpp"          // for Point, R3, Sphere, Line
#include "src/space/se2.hpp"         // for Point, R3, Sphere, Line
#include "src/space/space_base.hpp"  // for SpaceBase
#include "src/space/xyzq.hpp"
#include "src/tagged.hpp"      // for Tagged
#include "src/tree/fast.hpp"   // for Fast
#include "src/tree/naive.hpp"  // for Naive

using namespace std::chrono_literals;

template <typename Point>
using Tagged = rrts::Tagged<Point>;

template <typename Point, typename Bridge, size_t D>
using SpaceBase = rrts::space::SpaceBase<Point, Bridge, D>;

template <typename Point, typename Bridge, size_t D>
using Naive = rrts::tree::Naive<Point, Bridge, D>;

template <typename Point, typename Bridge, size_t D>
using Fast = rrts::tree::Fast<Point, Bridge, D>;

template <typename Point, typename Bridge, size_t D>
bool TestNearest(const SpaceBase<Point, Bridge, D> &space,
                 const Naive<Point, Bridge, D> &naive_tree, const Fast<Point, Bridge, D> &fast_tree,
                 const Point &test_point, std::chrono::duration<double> &naive_time,
                 std::chrono::duration<double> &fast_time) {
  auto t0 = std::chrono::high_resolution_clock::now();
  std::tuple<Tagged<Point>, Bridge> naive_nearest_ret = naive_tree.Nearest(
      [&test_point, &space](const Point &p) { return space.FormBridge(p, test_point); },
      [&test_point, &space](double distance) { return space.BoundingBox(test_point, distance); });
  auto t1 = std::chrono::high_resolution_clock::now();
  std::tuple<Tagged<Point>, Bridge> fast_nearest_ret = fast_tree.Nearest(
      [&test_point, &space](const Point &p) { return space.FormBridge(p, test_point); },
      [&test_point, &space](double distance) { return space.BoundingBox(test_point, distance); });
  auto t2 = std::chrono::high_resolution_clock::now();
  Tagged<Point> naive_nearest = std::get<0>(naive_nearest_ret);
  Tagged<Point> fast_nearest = std::get<0>(fast_nearest_ret);

  Bridge naive_nearest_bridge = std::get<1>(naive_nearest_ret);
  Bridge fast_nearest_bridge = std::get<1>(fast_nearest_ret);

  naive_time += t1 - t0;
  fast_time += t2 - t1;

  EXPECT_EQ(fast_nearest.Index(), naive_nearest.Index())
      << "naive tree nearest index (" << naive_nearest.Index() << ") != fast tree nearest index ("
      << fast_nearest.Index() << ")" << std::endl;

  if (fast_nearest.Index() != naive_nearest.Index()) {
    // test point
    fprintf(stderr, "test  point:");
    for (int32_t k = 0; k < static_cast<int32_t>(D); k++) {
      fprintf(stderr, " % 7.3f", test_point[k]);
    }
    fprintf(stderr, "\n");
    // fast point
    fprintf(stderr, "fast  point:");
    for (int32_t k = 0; k < static_cast<int32_t>(D); k++) {
      fprintf(stderr, " % 7.3f", fast_nearest.Point()[k]);
    }
    fprintf(stderr, " (distance: % 7.3f)\n", fast_nearest_bridge.TrajectoryCost());
    // naive point
    fprintf(stderr, "naive point:");
    for (int32_t k = 0; k < static_cast<int32_t>(D); k++) {
      fprintf(stderr, " % 7.3f", naive_nearest.Point()[k]);
    }
    fprintf(stderr, " (distance: % 7.3f)\n", naive_nearest_bridge.TrajectoryCost());

    std::cerr << "points in tree: " << naive_tree.Cardinality() << ", " << fast_tree.Cardinality()
              << std::endl;
    fast_tree.Draw();
    return true;
  }

  return false;
}

template <typename Point, typename Bridge, size_t D>
bool TestNear(const SpaceBase<Point, Bridge, D> &space, const Naive<Point, Bridge, D> &naive_tree,
              const Fast<Point, Bridge, D> &fast_tree, const Point &test_point, const double radius,
              std::chrono::duration<double> &naive_time, std::chrono::duration<double> &fast_time) {
  auto t0 = std::chrono::high_resolution_clock::now();
  const std::vector<std::tuple<Tagged<Point>, Bridge> > naive_near_points = naive_tree.Near(
      [&test_point, &space](const Point &p) { return space.FormBridge(p, test_point); },
      space.BoundingBox(test_point, radius), radius);
  auto t1 = std::chrono::high_resolution_clock::now();
  const std::vector<std::tuple<Tagged<Point>, Bridge> > fast_near_points = fast_tree.Near(
      [&test_point, &space](const Point &p) { return space.FormBridge(p, test_point); },
      space.BoundingBox(test_point, radius), radius);
  auto t2 = std::chrono::high_resolution_clock::now();
  naive_time += t1 - t0;
  fast_time += t2 - t1;

  std::unordered_map<size_t, Point> naive_near_point_map;
  std::unordered_map<size_t, Point> fast_near_point_map;
  for (const std::tuple<Tagged<Point>, Bridge> &naive_near_point_w_bridge : naive_near_points) {
    const Tagged<Point> &naive_near_point = std::get<0>(naive_near_point_w_bridge);
    naive_near_point_map.insert({naive_near_point.Index(), naive_near_point.Point()});
  }
  for (const std::tuple<Tagged<Point>, Bridge> &fast_near_point_w_bridge : fast_near_points) {
    const Tagged<Point> &fast_near_point = std::get<0>(fast_near_point_w_bridge);
    fast_near_point_map.insert({fast_near_point.Index(), fast_near_point.Point()});
  }

  bool fail = false;
  for (const auto it : naive_near_point_map) {
    const size_t index = it.first;
    const Point naive_near_point = it.second;
    if (fast_near_point_map.find(index) == fast_near_point_map.end()) {
      fail = true;
      std::cerr << "node " << index << " is missing from fast tree" << std::endl;
      std::cerr << naive_near_point.Render() << std::endl;
    }
  }

  for (const auto it : fast_near_point_map) {
    const size_t index = it.first;
    const Point fast_near_point = it.second;
    if (naive_near_point_map.find(index) == naive_near_point_map.end()) {
      fail = true;
      std::cerr << "node " << index << " was incorrectly found by fast tree" << std::endl;
      std::cerr << fast_near_point.Render() << std::endl;
    }
  }
  if (fail) {
    std::cerr << "test point: " << test_point.Render() << std::endl;
    std::cerr << "radius:     " << radius << std::endl;
  }
  return fail;
}

template <typename Point, typename Bridge, typename Space, size_t D>
void TestSpace(Space space) {
  Naive<Point, Bridge, D> naive_tree(space.Lb(), space.Ub());
  Fast<Point, Bridge, D> fast_tree(space.Lb(), space.Ub());

  const double zeta_d = VolumeOfNBall(D, 1.0);
  const double d = static_cast<double>(D);
  const double mu_xfree = space.MuXfree();
  const double gamma_rrts = pow(2 * (1 + 1 / d), 1 / d) * pow(mu_xfree / zeta_d, 1 / d);

  std::chrono::duration<double> fast_insert_time{};
  std::chrono::duration<double> naive_insert_time{};

  std::chrono::duration<double> fast_nearest_time{};
  std::chrono::duration<double> naive_nearest_time{};

  std::chrono::duration<double> fast_near_time{};
  std::chrono::duration<double> naive_near_time{};

  for (size_t count = 0; count < 5000; count++) {
    const Tagged<Point> p = {count, space.SampleFree()};

    // insert a new point
    auto t0 = std::chrono::high_resolution_clock::now();
    naive_tree.Insert(p);
    auto t1 = std::chrono::high_resolution_clock::now();
    fast_tree.Insert(p);
    auto t2 = std::chrono::high_resolution_clock::now();
    naive_insert_time += t1 - t0;
    fast_insert_time += t2 - t1;

    // compare cardinality
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-array-to-pointer-decay,hicpp-no-array-decay)
    ASSERT_EQ(naive_tree.Cardinality(), fast_tree.Cardinality());

    // do a nearest search
    const Point test_point = space.SampleFree();
    ASSERT_FALSE(TestNearest(space, naive_tree, fast_tree, test_point, naive_nearest_time,
                             fast_nearest_time));

    // do a near search
    // const double radius = pow(100 * volume / fast_tree.Cardinality(), 1/3);
    const double card_v = naive_tree.Cardinality();
    const double radius = gamma_rrts * pow(log(card_v) / card_v, 1 / d);

    ASSERT_FALSE(TestNear(space, naive_tree, fast_tree, test_point, radius, naive_near_time,
                          fast_near_time));
  }

  double n = naive_tree.Cardinality();
  fprintf(stderr, "        |   naive    |    fast\n");
  fprintf(stderr, "insert  | %7.3f us | %7.3f us\n", 1e6 * naive_insert_time.count() / n,
          1e6 * fast_insert_time.count() / n);
  fprintf(stderr, "nearest | %7.3f us | %7.3f us\n", 1e6 * naive_nearest_time.count() / n,
          1e6 * fast_nearest_time.count() / n);
  fprintf(stderr, "near    | %7.3f us | %7.3f us\n", 1e6 * naive_near_time.count() / n,
          1e6 * fast_near_time.count() / n);
}

TEST(TestTreeInSpace, R3) {
  using R3Line = rrts::space::r3::Line;
  using R3Point = rrts::space::r3::R3Point;
  using R3 = rrts::space::r3::R3;

  const R3Point lb = {-2, -3, -0.3};
  const R3Point ub = {1, 1.1, 0.6};
  R3 space(lb, ub, {});
  TestSpace<R3Point, R3Line, R3, 3>(space);
}

TEST(TestTreeInSpace, Se2SmallRho) {
  using DubinsPath = rrts::dubins::DubinsPath;
  using Se2Coord = rrts::space::se2::Se2Coord;
  using Se2 = rrts::space::se2::Se2;

  const glm::dvec2 lb = {-2, -3};
  const glm::dvec2 ub = {1, 1.1};

  const double small_rho = 0.6;
  Se2 space_small_rho(small_rho, lb, ub, {});
  TestSpace<Se2Coord, DubinsPath, Se2, 3>(space_small_rho);
}

TEST(TestTreeInSpace, Se2LargeRho) {
  using DubinsPath = rrts::dubins::DubinsPath;
  using Se2Coord = rrts::space::se2::Se2Coord;
  using Se2 = rrts::space::se2::Se2;

  const glm::dvec2 lb = {-2, -3};
  const glm::dvec2 ub = {1, 1.1};

  const double large_rho = 1.6;
  Se2 space_large_rho(large_rho, lb, ub, {});
  TestSpace<Se2Coord, DubinsPath, Se2, 3>(space_large_rho);
}

TEST(TestTreeInSpace, XyzqSmallRho) {
  using XyzqPath = rrts::space::xyzq::XyzqPath;
  using XyzqCoord = rrts::space::xyzq::XyzqCoord;
  using Xyzq = rrts::space::xyzq::Xyzq;

  const glm::dvec3 lb = {-2, -3, -3};
  const glm::dvec3 ub = {1, 1.1, 2};

  const double rho = 0.6;
  const double max_glideslope = 2.2;
  Xyzq space(rho, max_glideslope, lb, ub, {});
  TestSpace<XyzqCoord, XyzqPath, Xyzq, 4>(space);
}

TEST(TestTreeInSpace, XyzqSLargeRho) {
  using XyzqPath = rrts::space::xyzq::XyzqPath;
  using XyzqCoord = rrts::space::xyzq::XyzqCoord;
  using Xyzq = rrts::space::xyzq::Xyzq;

  const glm::dvec3 lb = {-2, -3, -3};
  const glm::dvec3 ub = {1, 1.1, 2};

  const double rho = 1.6;
  const double max_glideslope = 2.2;
  Xyzq space(rho, max_glideslope, lb, ub, {});
  TestSpace<XyzqCoord, XyzqPath, Xyzq, 4>(space);
}
