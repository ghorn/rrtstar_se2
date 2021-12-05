#pragma once

#include <iostream>
#include <vector>
#include <set>

#include "src/rrt_star.hpp"
#include "src/r3_search.hpp"
#include "src/tree/fast.hpp"
#include "src/tree/naive.hpp"

namespace rrts {
  namespace R3 {

    // dummy obstacle
    struct Sphere {
      glm::dvec3 center;
      double radius;
    };

    struct Line {
      Point p0;
      Point p1;
      double dist;
    };

    class R3Search : public Search<Point, Line, 3> {
    public:
      R3Search(Point x_init, Point lb, Point ub, std::vector<Sphere> sphere_obstacles) :
        lb_(lb), ub_(ub),
        naive_tree_(lb, ub),
        fast_tree_(lb, ub),
        sphere_obstacles_(sphere_obstacles)
      {
        assert(x_init.x <= ub.x);
        assert(x_init.y <= ub.y);
        assert(x_init.z <= ub.z);
        assert(lb.x <= x_init.x);
        assert(lb.y <= x_init.y);
        assert(lb.z <= x_init.z);

        naive_tree_.Insert(Tagged<Point>{0, x_init});
        fast_tree_.Insert(Tagged<Point>{0, x_init});
      };
      ~R3Search() = default;

      double mu_Xfree();
      Point SampleFree();
      double Cardinality() const {
        const double fast_card = fast_tree_.Cardinality();
        if (crosscheck_) {
          const double naive_card =  naive_tree_.Cardinality();
          if (fast_card != naive_card) {
            fprintf(stderr, "cardinality mismatch! %.1f != %.1f\n", fast_card, naive_card);
            exit(EXIT_FAILURE);
          }
        }
        return fast_card;
      };
      std::vector<Tagged<Point> > Near(const Point& test_point, double radius) const {
        const std::vector<Tagged<Point> > p0s = fast_tree_.Near(test_point, radius);

        if (crosscheck_) {
          const std::vector<Tagged<Point> > p1s = naive_tree_.Near(test_point, radius);

          // slow test
          std::set<size_t> p1_set;
          std::set<size_t> p0_set;
          for (const Tagged<Point> &p1 : p1s) {
            p1_set.insert(p1.index);
          }
          for (const Tagged<Point> &p0 : p0s) {
            p0_set.insert(p0.index);
          }

          bool fail = false;
          for (size_t index : p1_set) {
            if (p0_set.find(index) == p0_set.end()) {
              fail = true;
              std::cerr << "node " << index << " is missing from fast test" << std::endl;
            }
          }
          for (size_t index : p0_set) {
            if (p1_set.find(index) == p1_set.end()) {
              fail = true;
              std::cerr << "node " << index << " was incorrectly found by fast test" << std::endl;
            }
          }
          if (fail) {
            exit(EXIT_FAILURE);
          }
        }

        return p0s;
      }
      Tagged<Point> Nearest(const Point& test_point) const {
        Tagged<Point> p0 = fast_tree_.Nearest(test_point);

        if (crosscheck_) {
          Tagged<Point> p1 = naive_tree_.Nearest(test_point);

          if (p0.index != p1.index) {
            std::cerr << "naive tree nearest index (" << p1.index << ") != fast tree nearest index (" << p0.index << ")" << std::endl;
            fprintf(stderr, "test  point: % 7.3f % 7.3f % 7.3f\n", test_point.x, test_point.y, test_point.z);
            fprintf(stderr, "fast  point: % 7.3f % 7.3f % 7.3f (dist: %7.3f)\n", p0.point.x, p0.point.y, p0.point.z, R3::DistanceSquared(test_point, p0.point));
            fprintf(stderr, "naive point: % 7.3f % 7.3f % 7.3f (dist: %7.3f)\n", p1.point.x, p1.point.y, p1.point.z, R3::DistanceSquared(test_point, p1.point));
            std::cerr << "points in tree: " << naive_tree_.Cardinality() << ", " << fast_tree_.Cardinality() << std::endl;
            fast_tree_.Draw();
            exit(EXIT_FAILURE);
          }
        }

        return p0;
      }

      void InsertPoint(const Tagged<Point> &new_point) {
        fast_tree_.Insert(new_point);
        if (crosscheck_) {
          naive_tree_.Insert(new_point);
        }
      }

      Point Steer(const Point&, const Point&);

      // bridges
      bool CollisionFree(const Line&);
      double BridgeCost(const Line &line) {return line.dist;};
      Line FormBridge(const Point &v0, const Point &v1);

      const Point lb_;
      const Point ub_;
      tree::Naive<Point> naive_tree_;
      tree::Fast fast_tree_;
      std::vector<Sphere> sphere_obstacles_;
    private:
      Point Sample();
      bool PointInSphere(const Point &p);
      static const bool crosscheck_ = false;
    };

  }  // namespace R3
}  // namespace rrts
