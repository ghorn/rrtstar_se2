#include <chrono>  // for operator""s, chrono_literals
#include <glm/gtx/norm.hpp>
#include <iostream>
#include <random>
#include <set>

#include "src/space/n_ball.hpp"
#include "src/space/r3.hpp"
#include "src/space/space_base.hpp"
#include "src/tree/fast.hpp"
#include "src/tree/naive.hpp"

template <typename Point, typename Bridge, size_t D>
using SpaceBase = rrts::space::SpaceBase<Point, Bridge, D>;

template <typename P>
using Tagged = rrts::Tagged<P>;

using Bridge = rrts::space::r3::Line;

template <typename P, size_t D>
using Naive = rrts::tree::Naive<P, Bridge, D>;

template <typename P, size_t D>
using Fast = rrts::tree::Fast<P, Bridge, D>;

using Point = rrts::space::r3::Point;

using namespace std::chrono_literals;

static inline Point Sample(std::mt19937_64 &rng_engine,
                           std::uniform_real_distribution<double> &uniform_distribution,
                           const Point &lb, const Point &ub) {
  const double x = lb.x + (ub.x - lb.x) * uniform_distribution(rng_engine);
  const double y = lb.y + (ub.y - lb.y) * uniform_distribution(rng_engine);
  const double z = lb.z + (ub.z - lb.z) * uniform_distribution(rng_engine);
  return Point{x, y, z};
}

void TestNearest(const SpaceBase<Point, Bridge, 3> &space, const Naive<Point, 3> &naive_tree,
                 const Fast<Point, 3> &fast_tree, const Point &test_point,
                 std::chrono::duration<double> &naive_time,
                 std::chrono::duration<double> &fast_time) {
  auto t0 = std::chrono::high_resolution_clock::now();
  Tagged<Point> naive_nearest = std::get<0>(naive_tree.Nearest(
      [&test_point, &space](const Point &p) { return space.FormBridge(p, test_point); },
      [&test_point, &space](double distance) { return space.BoundingBox(test_point, distance); }));
  auto t1 = std::chrono::high_resolution_clock::now();
  Tagged<Point> fast_nearest = std::get<0>(fast_tree.Nearest(
      [&test_point, &space](const Point &p) { return space.FormBridge(p, test_point); },
      [&test_point, &space](double distance) { return space.BoundingBox(test_point, distance); }));
  auto t2 = std::chrono::high_resolution_clock::now();

  naive_time += t1 - t0;
  fast_time += t2 - t1;

  if (fast_nearest.index != naive_nearest.index) {
    std::cerr << "naive tree nearest index (" << naive_nearest.index
              << ") != fast tree nearest index (" << fast_nearest.index << ")" << std::endl;
    fprintf(stderr, "test  point: % 7.3f % 7.3f % 7.3f\n", test_point.x, test_point.y,
            test_point.z);
    fprintf(stderr, "fast  point: % 7.3f % 7.3f % 7.3f (dist: %7.3f)\n", fast_nearest.point.x,
            fast_nearest.point.y, fast_nearest.point.z,
            glm::distance2(glm::dvec3(test_point), glm::dvec3(fast_nearest.point)));
    fprintf(stderr, "naive point: % 7.3f % 7.3f % 7.3f (dist: %7.3f)\n", naive_nearest.point.x,
            naive_nearest.point.y, naive_nearest.point.z,
            glm::distance2(glm::dvec3(test_point), glm::dvec3(naive_nearest.point)));
    std::cerr << "points in tree: " << naive_tree.Cardinality() << ", " << fast_tree.Cardinality()
              << std::endl;
    fast_tree.Draw();
    std::exit(EXIT_FAILURE);  // NOLINT(concurrency-mt-unsafe)
  }
}

void TestNear(const SpaceBase<Point, Bridge, 3> &space, const Naive<Point, 3> &naive_tree,
              const Fast<Point, 3> &fast_tree, const Point &test_point, const double radius,
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

  std::set<size_t> naive_near_point_set;
  std::set<size_t> fast_near_point_set;
  for (const std::tuple<Tagged<Point>, Bridge> &naive_near_point : naive_near_points) {
    naive_near_point_set.insert(std::get<0>(naive_near_point).index);
  }
  for (const std::tuple<Tagged<Point>, Bridge> &fast_near_point : fast_near_points) {
    fast_near_point_set.insert(std::get<0>(fast_near_point).index);
  }

  bool fail = false;
  for (size_t index : naive_near_point_set) {
    if (fast_near_point_set.find(index) == fast_near_point_set.end()) {
      fail = true;
      std::cerr << "node " << index << " is missing from fast tree" << std::endl;
    }
  }
  for (size_t index : fast_near_point_set) {
    if (naive_near_point_set.find(index) == naive_near_point_set.end()) {
      fail = true;
      std::cerr << "node " << index << " was incorrectly found by fast tree" << std::endl;
    }
  }
  if (fail) {
    std::exit(EXIT_FAILURE);  // NOLINT(concurrency-mt-unsafe)
  }
}

void Run() {
  const Point lb = {-2, -3, -0.3};
  const Point ub = {1, 1.1, 0.6};

  Naive<Point, 3> naive_tree(lb, ub);
  Fast<Point, 3> fast_tree(lb, ub);

  std::mt19937_64 rng_engine;  // NOLINT(cert-msc32-c,cert-msc51-cpp)
  std::uniform_real_distribution<double> uniform_distribution;

  const double zeta_d = VolumeOfNBall(3, 1.0);
  const double d = 3;
  const double mu_xfree = (ub[0] - lb[0]) * (ub[1] - lb[1]) * (ub[2] - lb[2]);
  const double gamma_rrts = pow(2 * (1 + 1 / d), 1 / d) * pow(mu_xfree / zeta_d, 1 / d);

  std::chrono::duration<double> fast_insert_time{};
  std::chrono::duration<double> naive_insert_time{};

  std::chrono::duration<double> fast_nearest_time{};
  std::chrono::duration<double> naive_nearest_time{};

  std::chrono::duration<double> fast_near_time{};
  std::chrono::duration<double> naive_near_time{};

  rrts::space::r3::R3 space(lb, ub, {});

  for (size_t count = 0; count < 50000; count++) {
    const Tagged<Point> p = {count, Sample(rng_engine, uniform_distribution, lb, ub)};

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
    assert(naive_tree.Cardinality() == fast_tree.Cardinality());

    // do a nearest search
    const Point test_point = Sample(rng_engine, uniform_distribution, lb, ub);
    TestNearest(space, naive_tree, fast_tree, test_point, naive_nearest_time, fast_nearest_time);

    // do a near search
    // const double radius = pow(100 * volume / fast_tree.Cardinality(), 1/3);
    const double card_v = naive_tree.Cardinality();
    const double radius = gamma_rrts * pow(log(card_v) / card_v, 1 / d);

    TestNear(space, naive_tree, fast_tree, test_point, radius, naive_near_time, fast_near_time);
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

int main() {
  try {
    Run();
    return EXIT_SUCCESS;
  } catch (const std::exception &e) {
    std::cerr << e.what();
    return EXIT_FAILURE;
  }
}
