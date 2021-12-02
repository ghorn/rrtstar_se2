#include "r3_search.hpp"

#include <glm/gtx/intersect.hpp>
#include <glm/gtx/norm.hpp>

namespace rrts {
  namespace R3 {

    Point R3Search::Sample() {
      const double x = lb_.x + (ub_.x - lb_.x) * uniform_distribution(rng_engine);
      const double y = lb_.y + (ub_.y - lb_.y) * uniform_distribution(rng_engine);
      const double z = lb_.z + (ub_.z - lb_.z) * uniform_distribution(rng_engine);
      return {x, y, z};
    }

    bool R3Search::PointInSphere(const Point &p) {
      for (const Sphere &s : sphere_obstacles_) {
        if (glm::distance2(p, s.center) <= s.radius*s.radius) {
          return true;
        }
      }
      return false;
    }

    Point R3Search::SampleFree() {
      Point p = Sample();
      if (!PointInSphere(p)) {
        return p;
      }
      return SampleFree();
    }

    double R3Search::mu_Xfree(){
      const double dx = ub_.x - lb_.x;
      const double dy = ub_.y - lb_.y;
      const double dz = ub_.z - lb_.z;
      double volume = dx * dy * dz;
      //// subtract sphere obstacles
      //for (const Sphere &s : sphere_obstacles_) {
      //  const double r = s.radius;
      //  volume -= 4/3*M_PI*r*r*r;
      //}
      return volume;
    }

    static double DistSquared(const Point &x0, const Point &x1) {
      const double dx = x1.x - x0.x;
      const double dy = x1.y - x0.y;
      const double dz = x1.z - x0.z;
      return dx*dx + dy*dy + dz*dz;
    }

    std::vector<Tagged<Point> > R3Search::Near(const Point &x_test, double radius) {
      std::vector<Tagged<Point> > ret;
      const double radius2 = radius*radius;
      for (Tagged<Point> &x : tree_.points_) {
        if (DistSquared(x.point, x_test) < radius2) {
          ret.push_back(x);
        }
      }
      return ret;
    }

    Tagged<Point> R3Search::Nearest(const Point &x_test) {
      assert(!tree_.points_.empty());
      Tagged<Point> nearest_point = tree_.points_.at(0);
      double nearest_distance_squared = DistSquared(nearest_point.point, x_test);
      for (size_t k=1; k<tree_.points_.size(); k++) {
        Tagged<Point> x = tree_.points_.at(k);
        double distance_squared = DistSquared(x.point, x_test);
        if (distance_squared < nearest_distance_squared) {
          nearest_distance_squared = distance_squared;
          nearest_point = x;
        }
      }
      return nearest_point;
    }

    Line R3Search::FormBridge(const Point &v0, const Point &v1) {
      double dx = v1.x - v0.x;
      double dy = v1.y - v0.y;
      double dz = v1.z - v0.z;

      Line line;
      line.p0 = v0;
      line.p1 = v1;
      line.dist = sqrt(dx*dx + dy*dy + dz*dz);

      return line;
    }

    Point R3Search::Steer(const Point &v0, const Point &v1) {
      const double dx = v1.x - v0.x;
      const double dy = v1.y - v0.y;
      const double dz = v1.z - v0.z;

      const double dist = sqrt(dx*dx + dy*dy + dz*dz);

      // if distance is shorter than eta, then go all the way to the second point
      if (dist < eta_) {
        return v1;
      }

      // otherwise we have to shorten the distance to eta
      const double scale_factor = eta_ / dist;

      Point vret = v0;
      vret.x += dx * scale_factor;
      vret.y += dy * scale_factor;
      vret.z += dz * scale_factor;

      assert(std::isnormal(vret.x));
      assert(std::isnormal(vret.y));
      assert(std::isnormal(vret.z));

      //fprintf(stderr, "%8.4f %8.4f %8.4f\n", vret.x, vret.y, vret.z);
      return vret;
    }

    bool R3Search::CollisionFree(const Line &line) {
      // intersect with a couple dummy spheres
      const glm::dvec3 &gx0 = line.p0;
      const glm::dvec3 &gx1 = line.p1;
      for (const Sphere &s : sphere_obstacles_) {
        glm::dvec3 intersectPos1;
        glm::dvec3 intersectNormal1;
        glm::dvec3 intersectPos2;
        glm::dvec3 intersectNormal2;
        bool intersect = glm::intersectLineSphere<glm::dvec3>(gx0, gx1, s.center, s.radius,
                                                              intersectPos1, intersectNormal1,
                                                              intersectPos2, intersectNormal2);
        if (intersect) {
          return false;
        }
      }
      return true;
    }

    void R3Search::InsertPoint(const Tagged<Point> &new_point) {
      tree_.points_.push_back(new_point);
    }

  }  // namespace R3
}  // namespace rrts
