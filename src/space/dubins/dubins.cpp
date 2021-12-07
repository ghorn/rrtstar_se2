/*
 * Copyright (c) 2008-2018, Andrew Walker
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

#include <cmath>

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

int DubinsWord(DubinsIntermediateResults* in, DubinsPathType pathType, std::array<double, 3> out);
int ComputeDubinsIntermediateResults(DubinsIntermediateResults* in, std::array<double, 3> q0,
                                     std::array<double, 3> q1, double rho);

/**
 * Floating point modulus suitable for rings
 *
 * fmod doesn't behave correctly for angular quantities, this function does
 */
double Fmodr(double x, double y) { return x - y * floor(x / y); }

double Mod2pi(double theta) { return Fmodr(theta, 2 * M_PI); }

int DubinsShortestPath(DubinsPath* path, std::array<double, 3> q0, std::array<double, 3> q1,
                       double rho) {
  DubinsIntermediateResults in{};
  int errcode = ComputeDubinsIntermediateResults(&in, q0, q1, rho);
  if (errcode != EDUBOK) {
    return errcode;
  }

  path->qi = q0;
  path->rho = rho;

  int best_word = -1;
  double best_cost = INFINITY;
  for (int i = 0; i < 6; i++) {
    auto path_type = static_cast<DubinsPathType>(i);
    std::array<double, 3> params{};
    errcode = DubinsWord(&in, path_type, params);
    if (errcode == EDUBOK) {
      const double cost = params[0] + params[1] + params[2];
      if (cost < best_cost) {
        best_word = i;
        best_cost = cost;
        path->param = params;
        path->type = path_type;
      }
    }
  }
  if (best_word == -1) {
    return EDUBNOPATH;
  }
  return EDUBOK;
}

int ComputeDubinsPath(DubinsPath* path, std::array<double, 3> q0, std::array<double, 3> q1,
                      double rho, DubinsPathType pathType) {
  int errcode = 0;
  DubinsIntermediateResults in{};
  errcode = ComputeDubinsIntermediateResults(&in, q0, q1, rho);
  if (errcode == EDUBOK) {
    std::array<double, 3> params{};
    errcode = DubinsWord(&in, pathType, params);
    if (errcode == EDUBOK) {
      path->param = params;
      path->qi = q0;
      path->rho = rho;
      path->type = pathType;
    }
  }
  return errcode;
}

double DubinsPathLength(DubinsPath* path) {
  double length = 0.;
  length += path->param[0];
  length += path->param[1];
  length += path->param[2];
  length = length * path->rho;
  return length;
}

double DubinsSegmentLength(DubinsPath* path, int i) {
  if ((i < 0) || (i > 2)) {
    return INFINITY;
  }
  return path->param.at(static_cast<size_t>(i)) * path->rho;
}

double DubinsSegmentLengthNormalized(DubinsPath* path, int i) {
  if ((i < 0) || (i > 2)) {
    return INFINITY;
  }
  return path->param.at(static_cast<size_t>(i));
}

DubinsPathType ComputeDubinsPathType(DubinsPath* path) { return path->type; }

void DubinsSegment(double t, std::array<double, 3> qi, std::array<double, 3> qt, SegmentType type) {
  const double st = sin(qi[2]);
  const double ct = cos(qi[2]);
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
}

int DubinsPathSample(DubinsPath* path, double t, std::array<double, 3> q) {
  /* tprime is the normalised variant of the parameter t */
  double tprime = t / path->rho;
  std::array<double, 3> qi{}; /* The translated initial configuration */
  std::array<double, 3> q1{}; /* end-of segment 1 */
  std::array<double, 3> q2{}; /* end-of segment 2 */
  const std::array<SegmentType, 3> types = kDirdata.at(static_cast<size_t>(path->type));
  double p1{};
  double p2{};

  if (t < 0 || t > DubinsPathLength(path)) {
    return EDUBPARAM;
  }

  /* initial configuration */
  qi[0] = 0.0;
  qi[1] = 0.0;
  qi[2] = path->qi[2];

  /* generate the target configuration */
  p1 = path->param[0];
  p2 = path->param[1];
  DubinsSegment(p1, qi, q1, types[0]);
  DubinsSegment(p2, q1, q2, types[1]);
  if (tprime < p1) {
    DubinsSegment(tprime, qi, q, types[0]);
  } else if (tprime < (p1 + p2)) {
    DubinsSegment(tprime - p1, q1, q, types[1]);
  } else {
    DubinsSegment(tprime - p1 - p2, q2, q, types[2]);
  }

  /* scale the target configuration, translate back to the original starting point */
  q[0] = q[0] * path->rho + path->qi[0];
  q[1] = q[1] * path->rho + path->qi[1];
  q[2] = Mod2pi(q[2]);

  return EDUBOK;
}

int DubinsPathSampleMany(DubinsPath* path, double stepSize, DubinsPathSamplingCallback cb,
                         void* user_data) {
  int retcode = 0;
  std::array<double, 3> q{};
  double x = 0.0;
  double length = DubinsPathLength(path);
  while (x < length) {
    DubinsPathSample(path, x, q);
    retcode = cb(q, x, user_data);
    if (retcode != 0) {
      return retcode;
    }
    x += stepSize;
  }
  return 0;
}

int DubinsPathEndpoint(DubinsPath* path, std::array<double, 3> q) {
  return DubinsPathSample(path, DubinsPathLength(path) - EPSILON, q);
}

int DubinsExtractSubpath(DubinsPath* path, double t, DubinsPath* newpath) {
  /* calculate the true parameter */
  double tprime = t / path->rho;

  if ((t < 0) || (t > DubinsPathLength(path))) {
    return EDUBPARAM;
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
  return 0;
}

int ComputeDubinsIntermediateResults(DubinsIntermediateResults* in, std::array<double, 3> q0,
                                     std::array<double, 3> q1, double rho) {
  if (rho <= 0.0) {
    return EDUBBADRHO;
  }

  const double dx = q1[0] - q0[0];
  const double dy = q1[1] - q0[1];
  const double d = sqrt(dx * dx + dy * dy) / rho;
  double theta = 0;

  /* test required to prevent domain errors if dx=0 and dy=0 */
  if (d > 0) {
    theta = Mod2pi(atan2(dy, dx));
  }

  const double alpha = Mod2pi(q0[2] - theta);
  const double beta = Mod2pi(q1[2] - theta);

  in->alpha = alpha;
  in->beta = beta;
  in->d = d;
  in->sa = sin(alpha);
  in->sb = sin(beta);
  in->ca = cos(alpha);
  in->cb = cos(beta);
  in->c_ab = cos(alpha - beta);
  in->d_sq = d * d;

  return EDUBOK;
}

int DubinsLsl(DubinsIntermediateResults* in, std::array<double, 3> out) {
  double tmp0 = in->d + in->sa - in->sb;
  double p_sq = 2 + in->d_sq - (2 * in->c_ab) + (2 * in->d * (in->sa - in->sb));

  if (p_sq >= 0) {
    double tmp1 = atan2((in->cb - in->ca), tmp0);
    out[0] = Mod2pi(tmp1 - in->alpha);
    out[1] = sqrt(p_sq);
    out[2] = Mod2pi(in->beta - tmp1);
    return EDUBOK;
  }

  return EDUBNOPATH;
}

int DubinsRsr(DubinsIntermediateResults* in, std::array<double, 3> out) {
  double tmp0 = in->d - in->sa + in->sb;
  double p_sq = 2 + in->d_sq - (2 * in->c_ab) + (2 * in->d * (in->sb - in->sa));
  if (p_sq >= 0) {
    double tmp1 = atan2((in->ca - in->cb), tmp0);
    out[0] = Mod2pi(in->alpha - tmp1);
    out[1] = sqrt(p_sq);
    out[2] = Mod2pi(tmp1 - in->beta);
    return EDUBOK;
  }
  return EDUBNOPATH;
}

int DubinsLsr(DubinsIntermediateResults* in, std::array<double, 3> out) {
  double p_sq = -2 + (in->d_sq) + (2 * in->c_ab) + (2 * in->d * (in->sa + in->sb));
  if (p_sq >= 0) {
    double p = sqrt(p_sq);
    double tmp0 = atan2((-in->ca - in->cb), (in->d + in->sa + in->sb)) - atan2(-2.0, p);
    out[0] = Mod2pi(tmp0 - in->alpha);
    out[1] = p;
    out[2] = Mod2pi(tmp0 - Mod2pi(in->beta));
    return EDUBOK;
  }
  return EDUBNOPATH;
}

int DubinsRsl(DubinsIntermediateResults* in, std::array<double, 3> out) {
  double p_sq = -2 + in->d_sq + (2 * in->c_ab) - (2 * in->d * (in->sa + in->sb));
  if (p_sq >= 0) {
    double p = sqrt(p_sq);
    double tmp0 = atan2((in->ca + in->cb), (in->d - in->sa - in->sb)) - atan2(2.0, p);
    out[0] = Mod2pi(in->alpha - tmp0);
    out[1] = p;
    out[2] = Mod2pi(in->beta - tmp0);
    return EDUBOK;
  }
  return EDUBNOPATH;
}

int DubinsRlr(DubinsIntermediateResults* in, std::array<double, 3> out) {
  double tmp0 = (6. - in->d_sq + 2 * in->c_ab + 2 * in->d * (in->sa - in->sb)) / 8.;
  double phi = atan2(in->ca - in->cb, in->d - in->sa + in->sb);
  if (fabs(tmp0) <= 1) {
    double p = Mod2pi((2 * M_PI) - acos(tmp0));
    double t = Mod2pi(in->alpha - phi + Mod2pi(p / 2.));
    out[0] = t;
    out[1] = p;
    out[2] = Mod2pi(in->alpha - in->beta - t + Mod2pi(p));
    return EDUBOK;
  }
  return EDUBNOPATH;
}

int DubinsLrl(DubinsIntermediateResults* in, std::array<double, 3> out) {
  double tmp0 = (6. - in->d_sq + 2 * in->c_ab + 2 * in->d * (in->sb - in->sa)) / 8.;
  double phi = atan2(in->ca - in->cb, in->d + in->sa - in->sb);
  if (fabs(tmp0) <= 1) {
    double p = Mod2pi(2 * M_PI - acos(tmp0));
    double t = Mod2pi(-in->alpha - phi + p / 2.);
    out[0] = t;
    out[1] = p;
    out[2] = Mod2pi(Mod2pi(in->beta) - in->alpha - t + Mod2pi(p));
    return EDUBOK;
  }
  return EDUBNOPATH;
}

int DubinsWord(DubinsIntermediateResults* in, DubinsPathType pathType, std::array<double, 3> out) {
  int result = 0;
  switch (pathType) {
    case DubinsPathType::kLsl:
      result = DubinsLsl(in, out);
      break;
    case DubinsPathType::kRsl:
      result = DubinsRsl(in, out);
      break;
    case DubinsPathType::kLsr:
      result = DubinsLsr(in, out);
      break;
    case DubinsPathType::kRsr:
      result = DubinsRsr(in, out);
      break;
    case DubinsPathType::kLrl:
      result = DubinsLrl(in, out);
      break;
    case DubinsPathType::kRlr:
      result = DubinsRlr(in, out);
      break;
    default:
      result = EDUBNOPATH;
  }
  return result;
}
