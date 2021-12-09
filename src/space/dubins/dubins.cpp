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
#include "dubins.hpp"

#include <cassert>
#include <cmath>
#include <cstdlib>
#include <iostream>

#include "src/assert.hpp"

namespace rrts::dubins {

#define EPSILON (10e-10)

enum class SegmentType { kLSeg = 0, kSSeg = 1, kRSeg = 2 };

/* The segment types for each of the Path types */
const std::array<std::array<SegmentType, 3>, 6> kDirdata = {
    std::array{SegmentType::kLSeg, SegmentType::kSSeg, SegmentType::kLSeg},
    std::array{SegmentType::kLSeg, SegmentType::kSSeg, SegmentType::kRSeg},
    std::array{SegmentType::kRSeg, SegmentType::kSSeg, SegmentType::kLSeg},
    std::array{SegmentType::kRSeg, SegmentType::kSSeg, SegmentType::kRSeg},
    std::array{SegmentType::kRSeg, SegmentType::kLSeg, SegmentType::kRSeg},
    std::array{SegmentType::kLSeg, SegmentType::kRSeg, SegmentType::kLSeg}};

static inline const std::array<SegmentType, 3> &Dirdata(DubinsPathType type) {
  return kDirdata.at(static_cast<size_t>(type));
}

// Same as computing x - y but guaranteed to be in [-pi, pi].
double AngleDifference(double x, double y) {
  const double diff = x - y;
  double correction{};
  if (diff <= -M_PI) {
    correction = M_PI;
  } else {
    correction = -M_PI;
  }

  const double wrapped_difference = fmod(diff + M_PI, 2 * M_PI) + correction;

  // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-array-to-pointer-decay,hicpp-no-array-decay)
  assert(wrapped_difference <= M_PI);
  // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-array-to-pointer-decay,hicpp-no-array-decay)
  assert(wrapped_difference >= -M_PI);

  return wrapped_difference;
}

/**
 * Floating point modulus suitable for rings
 *
 * fmod doesn't behave correctly for angular quantities, this function does
 */
double Fmodr(double x, double y) { return x - y * floor(x / y); }

double Mod2pi(double theta) {
  const double ret = Fmodr(theta, 2 * M_PI);

  // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-array-to-pointer-decay,hicpp-no-array-decay)
  assert(ret <= 2 * M_PI);
  // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-array-to-pointer-decay,hicpp-no-array-decay)
  assert(ret >= 0);
  return ret;
}

// shortest path
DubinsPath::DubinsPath(const Se2Coord &q0, const Se2Coord &q1, double rho) {
  const DubinsIntermediateResults in = ComputeDubinsIntermediateResults(q0, q1, rho);

  qi_ = q0;
  qf_ = q1;
  rho_ = rho;

  double best_cost = INFINITY;
  for (DubinsPathType path_type :
       // The paper states without justification that we can skip RLR and LRL.
       {DubinsPathType::kLsl, DubinsPathType::kLsr, DubinsPathType::kRsl, DubinsPathType::kRsr}) {
    std::array<double, 3> normalized_segment_lengths{};
    DubinsWordStatus errcode = DubinsWord(in, path_type, normalized_segment_lengths);
    if (errcode == DubinsWordStatus::kSuccess) {
      const double cost = normalized_segment_lengths[0] + normalized_segment_lengths[1] +
                          normalized_segment_lengths[2];
      if (cost < best_cost) {
        best_cost = cost;
        normalized_segment_lengths_ = normalized_segment_lengths;
        type_ = path_type;
      }
    }
  }
  ASSERT_MSG(best_cost < (double)INFINITY, "DubinsShortestPath: no path found");  // lets see

  total_length_ = best_cost * rho;
}

DubinsWordStatus ComputeDubinsPath(DubinsPath &path, const Se2Coord &q0, const Se2Coord &q1,
                                   double rho, DubinsPathType pathType) {
  DubinsIntermediateResults in = ComputeDubinsIntermediateResults(q0, q1, rho);

  std::array<double, 3> normalized_segment_lengths{};
  DubinsWordStatus errcode = DubinsWord(in, pathType, normalized_segment_lengths);
  if (errcode == DubinsWordStatus::kSuccess) {
    path.normalized_segment_lengths_ = normalized_segment_lengths;
    path.qi_ = q0;
    path.qf_ = q1;
    path.rho_ = rho;
    path.type_ = pathType;
    path.total_length_ = rho * (normalized_segment_lengths[0] + normalized_segment_lengths[1] +
                                normalized_segment_lengths[2]);
  }
  return errcode;
}

std::array<double, 3> DubinsSegment(double t, const std::array<double, 3> &qi, SegmentType type) {
  const double st = sin(qi[2]);
  const double ct = cos(qi[2]);

  std::array<double, 3> qt = {0.0, 0.0, 0.0};

  if (type == SegmentType::kLSeg) {
    qt[0] = +sin(qi[2] + t) - st;
    qt[1] = -cos(qi[2] + t) + ct;
    qt[2] = t;
  } else if (type == SegmentType::kRSeg) {
    qt[0] = -sin(qi[2] - t) + st;
    qt[1] = +cos(qi[2] - t) - ct;
    qt[2] = -t;
  } else if (type == SegmentType::kSSeg) {
    qt[0] = ct * t;
    qt[1] = st * t;
    qt[2] = 0.0;
  }
  qt[0] += qi[0];
  qt[1] += qi[1];
  qt[2] += qi[2];

  return qt;
}

Se2Coord DubinsPath::Sample(double t) const {
  /* tprime is the normalised variant of the parameter t */
  double tprime = t / rho_;
  const std::array<SegmentType, 3> &types = Dirdata(type_);

  ASSERT_MSG(t >= 0 && t <= total_length_, "DubinsPathSample got bad input");

  /* initial configuration */
  /* The translated initial configuration */
  const std::array<double, 3> qi_translated = {0.0, 0.0, qi_[2]};

  /* generate the target configuration */
  const double p1 = normalized_segment_lengths_[0];
  const double p2 = normalized_segment_lengths_[1];
  /* end-of segment 1 */
  std::array<double, 3> q1 = DubinsSegment(p1, qi_translated, types[0]);
  /* end-of segment 2 */
  std::array<double, 3> q2 = DubinsSegment(p2, q1, types[1]);
  std::array<double, 3> q3{};
  if (tprime < p1) {
    q3 = DubinsSegment(tprime, qi_translated, types[0]);
  } else if (tprime < (p1 + p2)) {
    q3 = DubinsSegment(tprime - p1, q1, types[1]);
  } else {
    q3 = DubinsSegment(tprime - p1 - p2, q2, types[2]);
  }

  /* scale the target configuration, translate back to the original starting point */
  Se2Coord q{};
  q.position.x = q3[0] * rho_ + qi_[0];
  q.position.y = q3[1] * rho_ + qi_[1];
  q.theta = Mod2pi(q3[2]);

  if (q.theta > M_PI) {
    q.theta -= 2 * M_PI;
  }

  return q;
}

DubinsIntermediateResults ComputeDubinsIntermediateResults(const Se2Coord &q0, const Se2Coord &q1,
                                                           double rho) {
  ASSERT_MSG(rho > 0.0, "invalid rho: " << rho);

  const double dx = q1[0] - q0[0];
  const double dy = q1[1] - q0[1];
  const double d = sqrt(dx * dx + dy * dy) / rho;

  /* test required to prevent domain errors if dx=0 and dy=0 */
  double theta = 0;
  if (d > 0) {
    theta = Mod2pi(atan2(dy, dx));
  }

  const double alpha = Mod2pi(q0[2] - theta);
  const double beta = Mod2pi(q1[2] - theta);

  DubinsIntermediateResults in{};
  in.alpha = alpha;
  in.beta = beta;
  in.d = d;
  in.sa = sin(alpha);
  in.sb = sin(beta);
  in.ca = cos(alpha);
  in.cb = cos(beta);
  in.c_ab = cos(alpha - beta);
  in.d_sq = d * d;

  return in;
}

DubinsWordStatus DubinsLsl(const DubinsIntermediateResults &in, std::array<double, 3> &out) {
  double tmp0 = in.d + in.sa - in.sb;
  double p_sq = 2 + in.d_sq - (2 * in.c_ab) + (2 * in.d * (in.sa - in.sb));

  if (p_sq >= 0) {
    double tmp1 = atan2((in.cb - in.ca), tmp0);
    out[0] = Mod2pi(tmp1 - in.alpha);
    out[1] = sqrt(p_sq);
    out[2] = Mod2pi(in.beta - tmp1);
    return DubinsWordStatus::kSuccess;
  }

  return DubinsWordStatus::kNoPath;
}

DubinsWordStatus DubinsRsr(const DubinsIntermediateResults &in, std::array<double, 3> &out) {
  double tmp0 = in.d - in.sa + in.sb;
  double p_sq = 2 + in.d_sq - (2 * in.c_ab) + (2 * in.d * (in.sb - in.sa));
  if (p_sq >= 0) {
    double tmp1 = atan2((in.ca - in.cb), tmp0);
    out[0] = Mod2pi(in.alpha - tmp1);
    out[1] = sqrt(p_sq);
    out[2] = Mod2pi(tmp1 - in.beta);
    return DubinsWordStatus::kSuccess;
  }
  return DubinsWordStatus::kNoPath;
}

DubinsWordStatus DubinsLsr(const DubinsIntermediateResults &in, std::array<double, 3> &out) {
  double p_sq = -2 + (in.d_sq) + (2 * in.c_ab) + (2 * in.d * (in.sa + in.sb));
  if (p_sq >= 0) {
    double p = sqrt(p_sq);
    double tmp0 = atan2((-in.ca - in.cb), (in.d + in.sa + in.sb)) - atan2(-2.0, p);
    out[0] = Mod2pi(tmp0 - in.alpha);
    out[1] = p;
    out[2] = Mod2pi(tmp0 - Mod2pi(in.beta));
    return DubinsWordStatus::kSuccess;
  }
  return DubinsWordStatus::kNoPath;
}

DubinsWordStatus DubinsRsl(const DubinsIntermediateResults &in, std::array<double, 3> &out) {
  double p_sq = -2 + in.d_sq + (2 * in.c_ab) - (2 * in.d * (in.sa + in.sb));
  if (p_sq >= 0) {
    double p = sqrt(p_sq);
    double tmp0 = atan2((in.ca + in.cb), (in.d - in.sa - in.sb)) - atan2(2.0, p);
    out[0] = Mod2pi(in.alpha - tmp0);
    out[1] = p;
    out[2] = Mod2pi(in.beta - tmp0);
    return DubinsWordStatus::kSuccess;
  }
  return DubinsWordStatus::kNoPath;
}

DubinsWordStatus DubinsRlr(const DubinsIntermediateResults &in, std::array<double, 3> &out) {
  double tmp0 = (6. - in.d_sq + 2 * in.c_ab + 2 * in.d * (in.sa - in.sb)) / 8.;
  double phi = atan2(in.ca - in.cb, in.d - in.sa + in.sb);
  if (fabs(tmp0) <= 1) {
    double p = Mod2pi((2 * M_PI) - acos(tmp0));
    double t = Mod2pi(in.alpha - phi + Mod2pi(p / 2.));
    out[0] = t;
    out[1] = p;
    out[2] = Mod2pi(in.alpha - in.beta - t + Mod2pi(p));
    return DubinsWordStatus::kSuccess;
  }
  return DubinsWordStatus::kNoPath;
}

DubinsWordStatus DubinsLrl(const DubinsIntermediateResults &in, std::array<double, 3> &out) {
  double tmp0 = (6. - in.d_sq + 2 * in.c_ab + 2 * in.d * (in.sb - in.sa)) / 8.;
  double phi = atan2(in.ca - in.cb, in.d + in.sa - in.sb);
  if (fabs(tmp0) <= 1) {
    double p = Mod2pi(2 * M_PI - acos(tmp0));
    double t = Mod2pi(-in.alpha - phi + p / 2.);
    out[0] = t;
    out[1] = p;
    out[2] = Mod2pi(Mod2pi(in.beta) - in.alpha - t + Mod2pi(p));
    return DubinsWordStatus::kSuccess;
  }
  return DubinsWordStatus::kNoPath;
}

DubinsWordStatus DubinsWord(const DubinsIntermediateResults &in, DubinsPathType pathType,
                            std::array<double, 3> &normalized_segment_lengths) {
  switch (pathType) {
    case DubinsPathType::kLsl:
      return DubinsLsl(in, normalized_segment_lengths);
      break;
    case DubinsPathType::kRsl:
      return DubinsRsl(in, normalized_segment_lengths);
      break;
    case DubinsPathType::kLsr:
      return DubinsLsr(in, normalized_segment_lengths);
      break;
    case DubinsPathType::kRsr:
      return DubinsRsr(in, normalized_segment_lengths);
      break;
    case DubinsPathType::kLrl:
      return DubinsLrl(in, normalized_segment_lengths);
      break;
    case DubinsPathType::kRlr:
      return DubinsRlr(in, normalized_segment_lengths);
      break;
    default:
      FAIL_MSG("invalid dubins path type " << pathType);
      break;
  }
}

// NOLINTNEXTLINE(fuchsia-overloaded-operator)
std::ostream &operator<<(std::ostream &os, const DubinsPathType path_type) {
  switch (path_type) {
    case DubinsPathType::kLsl:
      return (os << "LSL");
      break;
    case DubinsPathType::kRsl:
      return (os << "RSL");
      break;
    case DubinsPathType::kLsr:
      return (os << "LSR");
      break;
    case DubinsPathType::kRsr:
      return (os << "RSR");
      break;
    case DubinsPathType::kLrl:
      return (os << "LRL");
      break;
    case DubinsPathType::kRlr:
      return (os << "RLR");
      break;
    default:
      return (os << "<INVALID>");
  }
}

// NOLINTNEXTLINE(fuchsia-overloaded-operator)
std::ostream &operator<<(std::ostream &os, const Se2Coord &coord) {
  return os << "{{" << coord.position.x << ", " << coord.position.y << "}, " << coord.theta << "}";
}

}  //  namespace rrts::dubins
