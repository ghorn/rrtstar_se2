#include "src/space/se2.hpp"

#include <algorithm>
#include <glm/gtx/intersect.hpp>
#include <glm/gtx/norm.hpp>

#include "src/assert.hpp"
#include "src/space/dubins/dubins.hpp"

namespace rrts::space::se2 {

using DubinsStatus = dubins::DubinsStatus;

Se2Coord Se2::Sample() {
  const double x =
      lb_.position.x + (ub_.position.x - lb_.position.x) * uniform_distribution_(rng_engine_);
  const double y =
      lb_.position.y + (ub_.position.y - lb_.position.y) * uniform_distribution_(rng_engine_);
  const double q = lb_.theta + (ub_.theta - lb_.theta) * uniform_distribution_(rng_engine_);
  return {{x, y}, q};
}

bool Se2::Se2CoordInSpheres(const Se2Coord &p) const {
  auto point_in_sphere = [&p](const Sphere &s) {
    return glm::distance2(s.center, p.position) <= s.radius * s.radius;
  };
  return std::any_of(sphere_obstacles_.cbegin(), sphere_obstacles_.cend(), point_in_sphere);
}

Se2Coord Se2::SampleFree() {
  for (;;) {
    Se2Coord p = Sample();
    if (!Se2CoordInSpheres(p)) {
      return p;
    }
  }
}

double Se2::MuXfree() const {
  const double dx = ub_.position.x - lb_.position.x;
  const double dy = ub_.position.y - lb_.position.y;
  const double dz = ub_.theta - lb_.theta;
  double volume = dx * dy * dz;
  //// subtract sphere obstacles
  // for (const Sphere &s : sphere_obstacles_) {
  //  const double r = s.radius;
  //  volume -= 4/3*M_PI*r*r*r;
  //}
  return volume;
}

DubinsPath Se2::FormBridge(const Se2Coord &v0, const Se2Coord &v1) const {
  DubinsPath dubins_path{};
  const DubinsStatus errcode = DubinsShortestPath(dubins_path, v0, v1, rho_);
  ASSERT_MSG(errcode == DubinsStatus::kSuccess,
             "FormBridge failed (" << static_cast<int>(errcode) << ")");

  return dubins_path;
}

Se2Coord Se2::Steer(const Se2Coord &v0, const Se2Coord &v1, double eta) const {
  DubinsPath dubins_path = FormBridge(v0, v1);
  const double dist = dubins_path.TotalLength();

  // if distance is shorter than eta, then go all the way to the second point
  if (dist <= eta) {
    return v1;
  }

  return dubins_path.Sample(eta);
}

bool Se2::CollisionFree(const DubinsPath &path) const {
  constexpr int kN = 20;
  for (int j = 0; j < kN; j++) {
    double t = 0.999999 * path.TotalLength() * static_cast<double>(j) / (kN - 1);  // static_cast)
    Se2Coord q = path.Sample(t);

    if ((q[0] < lb_[0]) || (ub_[0] < q[0]) || q[1] < lb_[1] || ub_[1] < q[1]) {
      return false;
    }

    auto point_in_sphere = [&q](const Sphere &s) {
      return glm::distance2({q[0], q[1]}, s.center) <= s.radius * s.radius;
    };
    if (std::any_of(sphere_obstacles_.cbegin(), sphere_obstacles_.cend(), point_in_sphere)) {
      return false;
    }
  }

  return true;
}

std::array<BoundingBoxIntervals, 3> Se2::BoundingBox(const Se2Coord &p, double max_distance) const {
  std::array<BoundingBoxIntervals, 3> bbs{};
  // x
  bbs[0].centroid = p.position.x;
  bbs[0].intervals.push_back(
      BoundingBoxInterval{p.position.x - max_distance, p.position.x + max_distance});

  // y
  bbs[1].centroid = p.position.y;
  bbs[1].intervals.push_back(
      BoundingBoxInterval{p.position.y - max_distance, p.position.y + max_distance});

  // theta (periodic)
  bbs[2].centroid = p.theta;

  // TODO(greg): double-check this
  // rho * theta = distance
  // theta = distance / rho
  double delta_theta = max_distance / rho_;
  BoundingBoxInterval theta_interval{p.theta - delta_theta, p.theta + delta_theta};

  // handle theta periodicity
  const double ub = M_PI;
  const double lb = -M_PI;
  const double right_overlap = theta_interval.ub - ub;
  const double left_overlap = lb - theta_interval.lb;
  if (right_overlap > 0) {
    bbs[2].intervals.push_back(BoundingBoxInterval{lb, lb + right_overlap});
    theta_interval.ub = ub;
  }
  if (left_overlap > 0) {
    bbs[2].intervals.push_back(BoundingBoxInterval{ub - left_overlap, ub});
    theta_interval.lb = lb;
  }
  bbs[2].intervals.push_back(theta_interval);

  return bbs;
}

}  // namespace rrts::space::se2
