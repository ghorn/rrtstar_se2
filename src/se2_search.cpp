#include "rrt_star.hpp"

#include <glm/gtx/intersect.hpp>

namespace rrts {

  Se2::Point Algorithm::SampleFree() {
    const double x = uniform_distribution(rng_engine);
    const double y = uniform_distribution(rng_engine);
    const double theta = uniform_distribution(rng_engine);
    const Se2::Point point {x, y, theta};
    return point;
  }

  Se2::Point Algorithm::Steer(Se2::Point v0, Se2::Point v1) {
    const double dx = v1.x - v0.x;
    const double dy = v1.y - v0.y;
    const double dq = v1.theta - v0.theta;
    const double scale_factor = eta_ / sqrt(dx*dx + dy*dy + dq*dq);

    v0.x += dx * scale_factor;
    v0.y += dy * scale_factor;
    v0.theta += dq * scale_factor;

    assert(std::isnormal(v0.x));
    assert(std::isnormal(v0.y));
    assert(std::isnormal(v0.theta));

    return v0;
  }


  void Algorithm::InsertPoint(Se2::Point new_point) {
    // TODO(greg): might need a parent
    tree_.Insert(new_point);
  }

  bool Algorithm::CollisionFree(Se2::Point v0,
                                Se2::Point v1) {
    // intersect with a couple dummy spheres
    glm::vec3 gv0 = {v0.x, v0.y, v0.theta};
    glm::vec3 gv1 = {v1.x, v1.y, v1.theta};
    for (Sphere s : sphere_obstacles) {
      glm::vec3 intersectPos1;
      glm::vec3 intersectNormal1;
      glm::vec3 intersectPos2;
      glm::vec3 intersectNormal2;
      bool intersect = glm::intersectLineSphere<glm::vec3>(gv0, gv1, s.center, static_cast<float>(s.radius),
                                                           intersectPos1, intersectNormal1,
                                                           intersectPos2, intersectNormal2);
      if (intersect) {
        return false;
      }
    }
    return true;
  }

  std::vector<Se2::Point> Algorithm::Near(Se2::Point test_point) {
    double cardV = static_cast<double>(tree_.num_nodes_); // TODO(greg): this might be wrong
    double radius = fmin(gamma_rrts * pow(log(cardV) / cardV, 1 / d), eta_);

    return tree_.PointsWithinRadiusOf(test_point, radius);
  }
}  //  namespace rrts
