#include <random>
#include <set>
#include <iostream>
#include <chrono>              // for operator""s, chrono_literals

#include "src/r3_point.hpp"
#include "src/rrt_star.hpp"
#include "src/tree/fast.hpp"
#include "src/tree/naive.hpp"

using namespace rrts;
using Point = R3::Point;
using namespace std::chrono_literals;

static inline Point Sample(std::mt19937_64 &rng_engine,
                           std::uniform_real_distribution<double> &uniform_distribution,
                           const Point &lb, const Point &ub) {
  const double x = lb.x + (ub.x - lb.x) * uniform_distribution(rng_engine);
  const double y = lb.y + (ub.y - lb.y) * uniform_distribution(rng_engine);
  const double z = lb.z + (ub.z - lb.z) * uniform_distribution(rng_engine);
  return Point{x, y, z};
}

void TestNearest(const tree::Naive<Point> &naive_tree,
                 const tree::Fast<Point> &fast_tree,
                 const Point &test_point,
                 std::chrono::duration<double> &naive_time,
                 std::chrono::duration<double> &fast_time) {
  auto t0 = std::chrono::high_resolution_clock::now();
  Tagged<Point> naive_nearest = naive_tree.Nearest(test_point);
  auto t1 = std::chrono::high_resolution_clock::now();
  Tagged<Point> fast_nearest = fast_tree.Nearest(test_point);
  auto t2 = std::chrono::high_resolution_clock::now();

  naive_time += t1 - t0;
  fast_time += t2 - t1;

  if (fast_nearest.index != naive_nearest.index) {
    std::cerr << "naive tree nearest index (" << naive_nearest.index << ") != fast tree nearest index (" << fast_nearest.index << ")" << std::endl;
    fprintf(stderr, "test  point: % 7.3f % 7.3f % 7.3f\n", test_point.x, test_point.y, test_point.z);
    fprintf(stderr, "fast  point: % 7.3f % 7.3f % 7.3f (dist: %7.3f)\n", fast_nearest.point.x, fast_nearest.point.y, fast_nearest.point.z, R3::DistanceSquared(test_point, fast_nearest.point));
    fprintf(stderr, "naive point: % 7.3f % 7.3f % 7.3f (dist: %7.3f)\n", naive_nearest.point.x, naive_nearest.point.y, naive_nearest.point.z, R3::DistanceSquared(test_point, naive_nearest.point));
    std::cerr << "points in tree: " << naive_tree.Cardinality() << ", " << fast_tree.Cardinality() << std::endl;
    fast_tree.Draw();
    exit(EXIT_FAILURE);
  }
}


void TestNear(const tree::Naive<Point> &naive_tree,
              const tree::Fast<Point> &fast_tree,
              const Point &test_point,
              const double radius,
              std::chrono::duration<double> &naive_time,
              std::chrono::duration<double> &fast_time) {
  auto t0 = std::chrono::high_resolution_clock::now();
  const std::vector<Tagged<Point> > naive_near_points = naive_tree.Near(test_point, radius);
  auto t1 = std::chrono::high_resolution_clock::now();
  const std::vector<Tagged<Point> > fast_near_points = fast_tree.Near(test_point, radius);
  auto t2 = std::chrono::high_resolution_clock::now();
  naive_time += t1 - t0;
  fast_time += t2 - t1;

  std::set<size_t> naive_near_point_set;
  std::set<size_t> fast_near_point_set;
  for (const Tagged<Point> &naive_near_point : naive_near_points) {
    naive_near_point_set.insert(naive_near_point.index);
  }
  for (const Tagged<Point> &fast_near_point : fast_near_points) {
    fast_near_point_set.insert(fast_near_point.index);
  }

  bool fail = false;
  for (size_t index : naive_near_point_set) {
    if (fast_near_point_set.find(index) == fast_near_point_set.end()) {
      fail = true;
      std::cerr << "node " << index << " is missing from fast test" << std::endl;
    }
  }
  for (size_t index : fast_near_point_set) {
    if (naive_near_point_set.find(index) == naive_near_point_set.end()) {
      fail = true;
      std::cerr << "node " << index << " was incorrectly found by fast test" << std::endl;
    }
  }
  if (fail) {
    exit(EXIT_FAILURE);
  }
}

int main() {
  const R3::Point lb = {-2, -3, -0.3};
  const R3::Point ub = {1, 1.1, 0.6};

  tree::Naive<Point> naive_tree;
  tree::Fast<Point> fast_tree(lb, ub);

  std::mt19937_64 rng_engine;
  std::uniform_real_distribution<double> uniform_distribution;

  const double zeta_d = VolumeOfNBall(3, 1.0);
  const double d = 3;
  const double mu_Xfree = (ub[0] - lb[0]) * (ub[1] - lb[1]) * (ub[2] - lb[2]);
  const double gamma_rrts = pow(2 * (1 + 1 / d), 1/d) * pow(mu_Xfree / zeta_d, 1 / d);

  std::chrono::duration<double> fast_insert_time{};
  std::chrono::duration<double> naive_insert_time{};

  std::chrono::duration<double> fast_nearest_time{};
  std::chrono::duration<double> naive_nearest_time{};

  std::chrono::duration<double> fast_near_time{};
  std::chrono::duration<double> naive_near_time{};

  for (size_t count=0; count<50000; count++) {
    const Tagged<R3::Point> p = {count, Sample(rng_engine, uniform_distribution, lb, ub)};

    // insert a new point
    auto t0 = std::chrono::high_resolution_clock::now();
    naive_tree.Insert(p);
    auto t1 = std::chrono::high_resolution_clock::now();
    fast_tree.Insert(p);
    auto t2 = std::chrono::high_resolution_clock::now();
    naive_insert_time += t1 - t0;
    fast_insert_time += t2 - t1;

    // compare cardinality
    assert(naive_tree.Cardinality() == fast_tree.Cardinality());

    // do a nearest search
    const Point test_point = Sample(rng_engine, uniform_distribution, lb, ub);
    TestNearest(naive_tree, fast_tree, test_point, naive_nearest_time, fast_nearest_time);

    // do a near search
    //const double radius = pow(100 * volume / fast_tree.Cardinality(), 1/3);
    const double cardV = naive_tree.Cardinality();
    const double radius = gamma_rrts * pow(log(cardV) / cardV, 1/d);

    TestNear(naive_tree, fast_tree, test_point, radius, naive_near_time, fast_near_time);
  }

  double n = naive_tree.Cardinality();
  fprintf(stderr, "        |   naive    |    fast\n");
  fprintf(stderr, "insert  | %7.3f us | %7.3f us\n",
          1e6*naive_insert_time.count()/n,
          1e6*fast_insert_time.count()/n);
  fprintf(stderr, "nearest | %7.3f us | %7.3f us\n",
          1e6*naive_nearest_time.count()/n,
          1e6*fast_nearest_time.count()/n);
  fprintf(stderr, "near    | %7.3f us | %7.3f us\n",
          1e6*naive_near_time.count()/n,
          1e6*fast_near_time.count()/n);
}