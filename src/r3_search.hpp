#pragma once

#include <vector>

#include "src/rrt_star.hpp"

namespace rrts {
  namespace R3 {

    // dummy obstacle
    struct Sphere {
      glm::dvec3 center;
      double radius;
    };

    using Point = glm::dvec3;
    struct Line {
      Point p0;
      Point p1;
      double dist;
    };

    class NaiveTree {
    public:
      NaiveTree(Tagged<Point> x_init) : points_{x_init} {};
      ~NaiveTree() = default;
      // TODO(greg): this might be wrong
      double Cardinality() {
        return static_cast<double>(points_.size());
      };
      std::vector<Tagged<Point> > points_;
    private:
    };

    class R3Search : public Search<Point, Line, 3> {
    public:
      R3Search(Point x_init, Point lb, Point ub, std::vector<Sphere> sphere_obstacles) :
        lb_(lb), ub_(ub), tree_(Tagged<Point>{0, x_init}), sphere_obstacles_(sphere_obstacles)
      {
        assert(x_init.x <= ub.x);
        assert(x_init.y <= ub.y);
        assert(x_init.z <= ub.z);
        assert(lb.x <= x_init.x);
        assert(lb.y <= x_init.y);
        assert(lb.z <= x_init.z);
      };
      ~R3Search() = default;

      double mu_Xfree();
      Point SampleFree();
      double CardV() {
        return tree_.Cardinality();
      };
      std::vector<Tagged<Point> > Near(const Point&, double);
      Tagged<Point> Nearest(const Point&);
      Point Steer(const Point&, const Point&);
      void InsertPoint(const Tagged<Point>&);

      // bridges
      bool CollisionFree(const Line&);
      double BridgeCost(const Line &line) {return line.dist;};
      Line FormBridge(const Point &v0, const Point &v1);

      const Point lb_;
      const Point ub_;
      NaiveTree tree_;
      std::vector<Sphere> sphere_obstacles_;
    private:
      Point Sample();
      bool PointInSphere(const Point &p);
    };

  }  // namespace R3
}  // namespace rrts
