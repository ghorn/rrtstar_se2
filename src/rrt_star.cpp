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

  Algorithm::StepResult Algorithm::Step() {
    // ********** Sample new point to add to tree. ***********
    // L3 from paper
    const Se2::Point x_rand = SampleFree();

    // L4 from paper
    const Se2::Point x_nearest = tree_.Nearest(x_rand);

    // L5 from paper
    const Se2::Point x_new = Steer(x_nearest, x_rand);

    // L6 from paper
    if (!CollisionFree(x_nearest, x_rand)) {
      return StepResult::kNotObstacleFree;
    }

    // L7 from paper
    std::vector<Se2::Point> X_near = Near(x_new);

    // ************** Add new node to graph. ****************
    // L8 from paper
    InsertPoint(x_new);

    // ********** Find best node to connect new node t. ***********
    // L9 from paper
    Se2::Point x_min = x_nearest;
    double c_min = 0;
    // TODO(greg): c_min = Cost(x_nearest) + c(Line(x_nearest, x_new));

    // L10-12 in paper
    for (const Se2::Point x_near : X_near) {
      double c = 0; // TODO(greg): Cost(x_near) + c(Line(x_near, x_new));
      if (CollisionFree(x_near, x_new) && c < c_min) {
        x_min = x_near;
        c_min = c;
      }
    }

    // L13 in paper
    //InsertEdge(x_min, x_new);

    // ******* Rewire tree: see if any near node is better off going to new node. ******
    // L14-16 in paper
    for (const Se2::Point x_near : X_near) {
      double c_near = 0; // TODO(greg): c_near = Cost(x_near);
      double c = 0; // TODO(greg): Cost(x_new) + c(Line(x_new, x_near));
      if (CollisionFree(x_new, x_near) && c < c_near) {
        // TODO(greg): Change parent.
      }
    }

    return StepResult::kSuccess;
  }

  double Algorithm::VolumeOfNBall(int n, double radius) {
    assert(n > 0);
    switch (n) {
    case 0:
      return 1;
    case 1:
      return 2 * radius;
    default:
      return 2 * M_PI / static_cast<double>(n) * radius * radius * VolumeOfNBall(n - 2, radius);
    }
  }

}  //  namespace rrts
