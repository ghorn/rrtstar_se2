#include "r3_search.hpp"

#include <iostream>
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

  }  // namespace R3
}  // namespace rrts
