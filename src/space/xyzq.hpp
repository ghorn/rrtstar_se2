#pragma once

#include <array>                       // for array
#include <cmath>                       // for M_PI
#include <cstdint>                     // for int32_t
#include <functional>                  // for function
#include <glm/ext/vector_double2.hpp>  // for dvec2
#include <random>                      // for mt19937_64, uniform_real_distribution
#include <tuple>                       // for tuple
#include <utility>                     // for move
#include <vector>                      // for vector

#include "src/space/dubins/dubins.hpp"  // for DubinsPath, Se2Coord
#include "src/space/r3.hpp"             // for Sphere
#include "src/space/space_base.hpp"     // for SpaceBase
#include "src/tree/tree_base.hpp"       // for BoundingBoxInterval, BoundingBoxIntervals
// #include "src/tree/node.hpp"  // BoundingBox

namespace rrts::space::xyzq {

using DubinsPath = dubins::DubinsPath;
using BoundingBoxInterval = rrts::tree::BoundingBoxInterval;
using BoundingBoxIntervals = rrts::tree::BoundingBoxIntervals;

// simple obstacle
using R3Sphere = rrts::space::r3::Sphere;

using BoundingBoxReflector =
    std::function<std::vector<BoundingBoxInterval>(int32_t axis, BoundingBoxInterval bb)>;

struct XyzqCoord {
  dubins::Se2Coord se2;
  double z;

  glm::dvec3 Xyz() const { return glm::dvec3{se2.position.x, se2.position.y, z}; }

  std::string Render() const {
    return std::to_string(se2.position.x) + " " + std::to_string(se2.position.y) + " " +
           std::to_string(z) + " " + std::to_string(se2.theta);
  }

  // NOLINTNEXTLINE(fuchsia-overloaded-operator)
  double &operator[](int32_t axis) {
    switch (axis) {
      case 0:
        return se2.position.x;
      case 1:
        return se2.position.y;
      case 2:
        return se2.theta;
      case 3:
        return z;
      default:
        FAIL_MSG("Se2Coord doesn't have axis " << axis);
    }
  }

  // NOLINTNEXTLINE(fuchsia-overloaded-operator)
  const double &operator[](int32_t axis) const {
    switch (axis) {
      case 0:
        return se2.position.x;
      case 1:
        return se2.position.y;
      case 2:
        return se2.theta;
      case 3:
        return z;
      default:
        FAIL_MSG("Se2Coord doesn't have axis " << axis);
    }
  }
};

class XyzqPath : public rrts::space::Trajectory<XyzqCoord> {
 public:
  XyzqPath(const DubinsPath &dubins, double z0, double zF) : dubins_(dubins), z0_(z0), zF_(zF) {
    double dxy = dubins_.TotalLength();
    double dz = zF - z0;
    // double dxyz = TotalLength();

    glideslope_ = dxy / dz;
    // glide_angle = std::atan(glideslope_);
    // cos_glideslope_ = dxy / dxyz;
    cos_glideslope_ = 1 / std::sqrt(1 + dz * dz / (dxy * dxy));  // TODO(greg): double-check

    // double w = std::fabs(dz / dxy);
    // sin_glideslope_ = w / std::sqrt(1 + w * w);  // TODO(greg): double-check
    // double glide_angle = std::atan(glideslope_);
    // cos_glideslope_ = std::cos(glide_angle);

    total_length_ = std::sqrt(dxy * dxy + dz * dz);
  }

 private:
  DubinsPath dubins_;
  double z0_;
  double zF_;
  double glideslope_;
  double cos_glideslope_;
  // double sin_glideslope_;
  double total_length_;  // memoize

 public:
  XyzqCoord Sample(double t) const {
    dubins::Se2Coord se2 = dubins_.Sample(t * cos_glideslope_);
    double z = z0_ + (zF_ - z0_) * t / total_length_;
    return XyzqCoord{se2, z};
  };
  double TotalLength() const { return total_length_; };
  XyzqCoord Endpoint() const {
    dubins::Se2Coord se2 = dubins_.Endpoint();
    return XyzqCoord{se2, zF_};
  };
  double TrajectoryCost() const override { return TotalLength(); };

  const DubinsPath &Dubins() const { return dubins_; }
  double Glideslope() const { return glideslope_; }
  double Z0() const { return z0_; }
  double ZF() const { return zF_; }
};  // struct XyzqPath

class Xyzq : public SpaceBase<XyzqCoord, XyzqPath, 4> {
 public:
  Xyzq(double rho, double max_glideslope, const glm::dvec3 &lb, const glm::dvec3 &ub,
       std::vector<R3Sphere> sphere_obstacles)
      : rho_(rho),
        max_glideslope_(max_glideslope),
        lb_{{{lb[0], lb[1]}, -M_PI}, lb[2]},
        ub_{{{ub[0], ub[1]}, M_PI}, ub[2]},
        sphere_obstacles_(std::move(sphere_obstacles)) {
    sin_glideangle_ = std::sin(std::atan(std::fabs(max_glideslope)));
    double sin_glideangle2_ =
        std::fabs(max_glideslope_) / std::sqrt(1 + max_glideslope_ * max_glideslope_);
    ASSERT_MSG(std::fabs(sin_glideangle_ - sin_glideangle2_) < 1e-9,
               "sin_glideangle_ " << sin_glideangle_ << " vs " << sin_glideangle2_);
  };
  ~Xyzq() override = default;

  [[nodiscard]] double MuXfree() const override;
  XyzqCoord SampleFree() override;
  [[nodiscard]] std::tuple<XyzqCoord, XyzqPath> Steer(const XyzqCoord &v0, const XyzqCoord &v1,
                                                      double eta) const override;

  // bridges
  [[nodiscard]] bool CollisionFree(const XyzqPath &path) const override;
  [[nodiscard]] XyzqPath FormBridge(const XyzqCoord &v0, const XyzqCoord &v1) const override;

  // efficient search
  [[nodiscard]] std::array<BoundingBoxIntervals, 4> BoundingBox(const XyzqCoord &p,
                                                                double max_distance) const override;

  [[nodiscard]] const XyzqCoord &Lb() const override { return lb_; };
  [[nodiscard]] const XyzqCoord &Ub() const override { return ub_; };

 private:
  XyzqCoord Sample();
  [[nodiscard]] bool XyzqCoordInSpheres(const XyzqCoord &p) const;

  double rho_;
  double max_glideslope_;
  double sin_glideangle_;

  XyzqCoord lb_;
  XyzqCoord ub_;
  std::vector<R3Sphere> sphere_obstacles_;
  std::mt19937_64 rng_engine_{};  // NOLINT(cert-msc32-c,cert-msc51-cpp)
  std::uniform_real_distribution<double> uniform_distribution_{};
};

}  // namespace rrts::space::xyzq
