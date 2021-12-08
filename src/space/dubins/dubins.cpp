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

DubinsStatus DubinsShortestPath(DubinsPath &path, const std::array<double, 3> &q0,
                                const std::array<double, 3> &q1, double rho) {
  const DubinsIntermediateResults in = ComputeDubinsIntermediateResults(q0, q1, rho);

  // DubinsPath path{};
  path.qi = q0;
  path.rho = rho;

  int best_word = -1;
  double best_cost = INFINITY;
  for (int i = 0; i < 6; i++) {
    auto path_type = static_cast<DubinsPathType>(i);
    std::array<double, 3> params{};
    DubinsWordStatus errcode = DubinsWord(in, path_type, params);
    if (errcode == DubinsWordStatus::kSuccess) {
      const double cost = params[0] + params[1] + params[2];
      if (cost < best_cost) {
        best_word = i;
        best_cost = cost;
        path.param = params;
        path.type = path_type;
      }
    }
  }
  ASSERT_MSG(best_word != -1, "DubinsShortestPath: no path found");  // lets see
  if (best_word == -1) {
    return DubinsStatus::kNoPath;
  }
  path.total_length = best_cost * path.rho;
  return DubinsStatus::kSuccess;
}

DubinsWordStatus ComputeDubinsPath(DubinsPath &path, const std::array<double, 3> &q0,
                                   const std::array<double, 3> &q1, double rho,
                                   DubinsPathType pathType) {
  DubinsIntermediateResults in = ComputeDubinsIntermediateResults(q0, q1, rho);

  std::array<double, 3> params{};
  DubinsWordStatus errcode = DubinsWord(in, pathType, params);
  if (errcode == DubinsWordStatus::kSuccess) {
    path.param = params;
    path.qi = q0;
    path.rho = rho;
    path.type = pathType;
    path.total_length = rho * (params[0] + params[1] + params[2]);
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

DubinsStatus DubinsPathSample(const DubinsPath &path, double t, std::array<double, 3> &q) {
  /* tprime is the normalised variant of the parameter t */
  double tprime = t / path.rho;
  const std::array<SegmentType, 3> &types = Dirdata(path.type);

  if (t < 0 || t > path.total_length) {
    return DubinsStatus::kPathParameterizationError;
  }

  /* initial configuration */
  /* The translated initial configuration */
  const std::array<double, 3> qi = {0.0, 0.0, path.qi[2]};

  /* generate the target configuration */
  const double p1 = path.param[0];
  const double p2 = path.param[1];
  /* end-of segment 1 */
  std::array<double, 3> q1 = DubinsSegment(p1, qi, types[0]);
  /* end-of segment 2 */
  std::array<double, 3> q2 = DubinsSegment(p2, q1, types[1]);
  if (tprime < p1) {
    q = DubinsSegment(tprime, qi, types[0]);
  } else if (tprime < (p1 + p2)) {
    q = DubinsSegment(tprime - p1, q1, types[1]);
  } else {
    q = DubinsSegment(tprime - p1 - p2, q2, types[2]);
  }

  /* scale the target configuration, translate back to the original starting point */
  q[0] = q[0] * path.rho + path.qi[0];
  q[1] = q[1] * path.rho + path.qi[1];
  q[2] = Mod2pi(q[2]);

  if (q[2] > M_PI) {
    q[2] -= 2 * M_PI;
  }

  return DubinsStatus::kSuccess;
}

DubinsStatus DubinsPathEndpoint(const DubinsPath &path, std::array<double, 3> &q) {
  return DubinsPathSample(path, path.total_length - EPSILON, q);
}

DubinsStatus DubinsExtractSubpath(DubinsPath *path, double t, DubinsPath *newpath) {
  /* calculate the true parameter */
  double tprime = t / path->rho;

  if ((t < 0) || (t > path->total_length)) {
    return DubinsStatus::kPathParameterizationError;
  }

  /* copy most of the data */
  newpath->qi = path->qi;
  newpath->rho = path->rho;
  newpath->type = path->type;

  /* fix the parameters */
  newpath->param.at(0) = fmin(path->param.at(0), tprime);
  newpath->param.at(1) = fmin(path->param.at(1), tprime - newpath->param.at(0));
  newpath->param.at(2) =
      fmin(path->param.at(2), tprime - newpath->param.at(0) - newpath->param.at(1));
  return DubinsStatus::kSuccess;
}

DubinsIntermediateResults ComputeDubinsIntermediateResults(const std::array<double, 3> &q0,
                                                           const std::array<double, 3> &q1,
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
                            std::array<double, 3> &out) {
  switch (pathType) {
    case DubinsPathType::kLsl:
      return DubinsLsl(in, out);
      break;
    case DubinsPathType::kRsl:
      return DubinsRsl(in, out);
      break;
    case DubinsPathType::kLsr:
      return DubinsLsr(in, out);
      break;
    case DubinsPathType::kRsr:
      return DubinsRsr(in, out);
      break;
    case DubinsPathType::kLrl:
      return DubinsLrl(in, out);
      break;
    case DubinsPathType::kRlr:
      return DubinsRlr(in, out);
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
