#include "rrt_star.hpp"

#include <glm/gtx/intersect.hpp>

namespace rrts {

  double VolumeOfNBall(int n, double radius) {
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

//  Se2::Point Search::SampleFree() {
//    const double x = uniform_distribution(rng_engine);
//    const double y = uniform_distribution(rng_engine);
//    const double theta = uniform_distribution(rng_engine);
//    const Se2::Point point {x, y, theta};
//    return point;
//  }
//
//  Se2::Point Search::Steer(Se2::Point v0, Se2::Point v1) {
//    const double dx = v1.x - v0.x;
//    const double dy = v1.y - v0.y;
//    const double dq = v1.theta - v0.theta;
//    const double scale_factor = eta_ / sqrt(dx*dx + dy*dy + dq*dq);
//
//    v0.x += dx * scale_factor;
//    v0.y += dy * scale_factor;
//    v0.theta += dq * scale_factor;
//
//    assert(std::isnormal(v0.x));
//    assert(std::isnormal(v0.y));
//    assert(std::isnormal(v0.theta));
//
//    return v0;
//  }
//
//
//  void Search::InsertPoint(Se2::Point new_point) {
//    // TODO(greg): might need a parent
//    tree_.Insert(new_point);
//  }
//
//
//  std::vector<Se2::Point> Search::Near(Se2::Point test_point) {
//    double cardV = static_cast<double>(tree_.num_nodes_); // TODO(greg): this might be wrong
//    double radius = fmin(gamma_rrts * pow(log(cardV) / cardV, 1 / d), eta_);
//
//    return tree_.PointsWithinRadiusOf(test_point, radius);
//  }

}  //  namespace rrts
