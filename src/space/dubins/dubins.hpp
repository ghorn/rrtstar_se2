#pragma once

/*
 * Copyright (c) 2008-2018, Andrew Walker
 * Copyright (c)      2021, Greg Horn (heavily modified)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <array>                       // for array
#include <cstdint>                     // for int32_t
#include <glm/ext/vector_double2.hpp>  // for dvec2
#include <glm/glm.hpp>                 // for vec<>::(anonymous), vec
#include <sstream>                     // for operator<<, basic_ostream, endl, basic_ostream::op...
#include <string>                      // for char_traits, string

#include "src/assert.hpp"            // for FAIL_MSG
#include "src/space/space_base.hpp"  // for Trajectory

namespace rrts::dubins {

// Same as computing x - y but guaranteed to be in [-pi, pi].
double AngleDifference(double x, double y);

struct Se2Coord {
  Se2Coord() = default;
  glm::dvec2 position;
  double theta;

  // NOLINTNEXTLINE(fuchsia-overloaded-operator)
  double &operator[](int32_t axis) {
    switch (axis) {
      case 0:
        return position.x;
      case 1:
        return position.y;
      case 2:
        return theta;
      default:
        FAIL_MSG("Se2Coord doesn't have axis " << axis);
    }
  }

  // NOLINTNEXTLINE(fuchsia-overloaded-operator)
  const double &operator[](int32_t axis) const {
    switch (axis) {
      case 0:
        return position.x;
      case 1:
        return position.y;
      case 2:
        return theta;
      default:
        FAIL_MSG("Se2Coord doesn't have axis " << axis);
    }
  }
};

// NOLINTNEXTLINE(fuchsia-overloaded-operator)
std::ostream &operator<<(std::ostream &os, const Se2Coord &coordpath_type);

enum class DubinsPathType { kLsl = 0, kLsr = 1, kRsl = 2, kRsr = 3, kRlr = 4, kLrl = 5 };

// NOLINTNEXTLINE(fuchsia-overloaded-operator)
std::ostream &operator<<(std::ostream &os, DubinsPathType path_type);

enum class DubinsWordStatus {
  kSuccess,
  kNoPath, /* no connection between configurations with this word */
};

enum class DubinsStatus {
  kSuccess = 0, /* No error */
  kNoPath,      /* no connection between configurations with this word */
};

class DubinsPath : public rrts::space::Trajectory<Se2Coord> {
 public:
  DubinsPath() = default;
  DubinsPath(const Se2Coord &q0, const Se2Coord &q1, double rho);
  ~DubinsPath() override = default;
  [[nodiscard]] double TrajectoryCost() const override { return TotalLength(); }

 private:
  /* the initial configuration */
  Se2Coord qi_{};
  /* the final configuration */
  Se2Coord qf_{};
  /* the lengths of the three segments */
  std::array<double, 3> normalized_segment_lengths_{};
  /* model forward velocity / model angular velocity */
  double rho_{};
  /* the path type described */
  DubinsPathType type_{};
  /* total length */
  double total_length_{};

 public:
  [[nodiscard]] const Se2Coord &Endpoint() const { return qf_; }
  [[nodiscard]] DubinsPathType Type() const { return type_; }
  [[nodiscard]] double TotalLength() const { return total_length_; }

  [[nodiscard]] Se2Coord Sample(double t) const;
  [[nodiscard]] double AngleDifference() const {
    return rrts::dubins::AngleDifference(qf_.theta, qi_.theta);
  }
  [[nodiscard]] double MaxAngle() const {
    const double max_02 = fmax(normalized_segment_lengths_[0], normalized_segment_lengths_[2]);
    switch (type_) {
      case DubinsPathType::kLsl:
      case DubinsPathType::kLsr:
      case DubinsPathType::kRsl:
      case DubinsPathType::kRsr:
        return max_02;
        break;
      case DubinsPathType::kRlr:
      case DubinsPathType::kLrl:
        return fmax(max_02, normalized_segment_lengths_[1]);
        break;
      default:
        FAIL_MSG("Unknown Dubins path type " << type_);
    }
    FAIL_MSG("Unknown Dubins path type " << type_);
  }

  std::string Describe() {
    std::stringstream message;
    message << "type: " << type_ << std::endl;
    message << "qi: " << qi_ << std::endl;
    message << "normalized lengths: " << normalized_segment_lengths_.at(0) << " "
            << normalized_segment_lengths_.at(1) << " " << normalized_segment_lengths_.at(2)
            << std::endl;
    message << "rho: " << rho_ << std::endl;
    return message.str();
  }

  // TODO(greg): consider into constructor
  friend DubinsWordStatus ComputeDubinsPath(DubinsPath &path, const Se2Coord &q0,
                                            const Se2Coord &q1, double rho,
                                            DubinsPathType pathType);
};

struct DubinsIntermediateResults {
  double alpha;
  double beta;
  double d;
  double sa;
  double sb;
  double ca;
  double cb;
  double c_ab;
  double d_sq;
};

DubinsWordStatus DubinsWord(const DubinsIntermediateResults &in, DubinsPathType pathType,
                            std::array<double, 3> &normalized_segment_lengths);
DubinsIntermediateResults ComputeDubinsIntermediateResults(const Se2Coord &q0, const Se2Coord &q1,
                                                           double rho);

/**
 * Generate a path with a specified word from an initial configuration to
 * a target configuration, with a specified turning radius
 *
 * @param path     - the resultant path
 * @param q0       - a configuration specified as an array of x, y, theta
 * @param q1       - a configuration specified as an array of x, y, theta
 * @param rho      - turning radius of the vehicle (forward velocity divided by maximum angular
 * velocity)
 * @param pathType - the specific path type to use
 * @return         - non-zero on error
 */
DubinsWordStatus ComputeDubinsPath(DubinsPath &path, const Se2Coord &q0, const Se2Coord &q1,
                                   double rho, DubinsPathType pathType);

}  //  namespace rrts::dubins
