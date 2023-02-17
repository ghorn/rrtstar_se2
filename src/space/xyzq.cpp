#include "src/space/xyzq.hpp"

#include <algorithm>         // for any_of
#include <glm/glm.hpp>       // for vec<>::(anonymous), dot, operator-
#include <glm/gtx/norm.hpp>  // for distance2

#include "src/space/dubins/dubins.hpp"  // for DubinsStatus

namespace rrts::space::xyzq {

using DubinsStatus = dubins::DubinsStatus;

XyzqCoord Xyzq::Sample() {
  const double x = lb_.se2.position.x +
                   (ub_.se2.position.x - lb_.se2.position.x) * uniform_distribution_(rng_engine_);
  const double y = lb_.se2.position.y +
                   (ub_.se2.position.y - lb_.se2.position.y) * uniform_distribution_(rng_engine_);
  const double z = lb_.z + (ub_.z - lb_.z) * uniform_distribution_(rng_engine_);
  const double q =
      lb_.se2.theta + (ub_.se2.theta - lb_.se2.theta) * uniform_distribution_(rng_engine_);
  return {dubins::Se2Coord{{x, y}, q}, z};
}

bool Xyzq::XyzqCoordInSpheres(const XyzqCoord &p) const {
  auto point_in_sphere = [&p](const R3Sphere &s) {
    return glm::distance2(s.center, p.Xyz()) <= s.radius * s.radius;
  };
  return std::any_of(sphere_obstacles_.cbegin(), sphere_obstacles_.cend(), point_in_sphere);
}

XyzqCoord Xyzq::SampleFree() {
  for (;;) {
    XyzqCoord p = Sample();
    if (!XyzqCoordInSpheres(p)) {
      return p;
    }
  }
}

double Xyzq::MuXfree() const {
  const double dx = ub_.se2.position.x - lb_.se2.position.x;
  const double dy = ub_.se2.position.y - lb_.se2.position.y;
  const double dq = ub_.se2.theta - lb_.se2.theta;
  const double dz = ub_.z - lb_.z;
  double volume = dx * dy * dz * dq;
  return volume;
}

XyzqPath Xyzq::FormBridge(const XyzqCoord &v0, const XyzqCoord &v1) const {
  DubinsPath dubins(v0.se2, v1.se2, rho_);
  return XyzqPath(dubins, v0.z, v1.z);
}

std::tuple<XyzqCoord, XyzqPath> Xyzq::Steer(const XyzqCoord &v0, const XyzqCoord &v1,
                                            double eta) const {
  const XyzqPath xyzq_path = FormBridge(v0, v1);

  // if distance is shorter than eta, then go all the way to the second point
  if (xyzq_path.TotalLength() <= eta) {
    return std::make_tuple(v1, xyzq_path);
  }

  // if distance is longer than eta, then go eta distance along the path
  const XyzqCoord vret = xyzq_path.Sample(eta);
  return std::make_tuple(vret, FormBridge(v0, vret));
}

bool Xyzq::CollisionFree(const XyzqPath &path) const {
  // handle glideslope
  if (path.ZF() <= path.Z0()) {
    return false;
  }
  if (std::fabs(path.Glideslope()) < max_glideslope_) {
    return false;
  }
  // // discard paths that turn too much too fast
  // if (fabs(path.Dubins().AngleDifference()) >= M_PI / 2) {
  //   return false;
  // }
  // if (path.Dubins().MaxAngle() >= M_PI / 4) {
  //   return false;
  // }

  constexpr int kN = 20;
  for (int j = 0; j < kN; j++) {
    double t = 0.999999 * path.TotalLength() * static_cast<double>(j) / (kN - 1);
    XyzqCoord q = path.Sample(t);

    if ((q.se2.position.x < lb_.se2.position.x) || (ub_.se2.position.x < q.se2.position.x) ||
        (q.se2.position.y < lb_.se2.position.y) || (ub_.se2.position.y < q.se2.position.y) ||
        (q.z < lb_.z) || (ub_.z < q.z)) {
      return false;
    }

    glm::dvec3 xyz = q.Xyz();
    auto point_in_sphere = [&xyz](const R3Sphere &s) {
      return glm::distance2(xyz, s.center) <= s.radius * s.radius;
    };
    if (std::any_of(sphere_obstacles_.cbegin(), sphere_obstacles_.cend(), point_in_sphere)) {
      return false;
    }
  }

  return true;
}

std::array<BoundingBoxIntervals, 4> Xyzq::BoundingBox(const XyzqCoord &p,
                                                      double max_distance) const {
  std::array<BoundingBoxIntervals, 4> bbs{};
  // x
  bbs[0].centroid = p.se2.position.x;
  bbs[0].intervals.push_back(
      BoundingBoxInterval{p.se2.position.x - max_distance, p.se2.position.x + max_distance});

  // y
  bbs[1].centroid = p.se2.position.y;
  bbs[1].intervals.push_back(
      BoundingBoxInterval{p.se2.position.y - max_distance, p.se2.position.y + max_distance});

  // theta (periodic)
  bbs[2].centroid = p.se2.theta;

  // TODO(greg): double-check this
  // rho * theta = distance
  // theta = distance / rho
  double delta_theta = max_distance / rho_;
  BoundingBoxInterval theta_interval{p.se2.theta - delta_theta, p.se2.theta + delta_theta};

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

  // z
  bbs[3].centroid = p.z;
  double delta_z = max_distance * sin_glideangle_;
  bbs[3].intervals.push_back(BoundingBoxInterval{p.z - delta_z, p.z + delta_z});

  return bbs;
}

}  // namespace rrts::space::xyzq
