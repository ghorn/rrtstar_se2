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

#include <array>
#include <sstream>
#include <string>

enum class DubinsPathType { kLsl = 0, kLsr = 1, kRsl = 2, kRsr = 3, kRlr = 4, kLrl = 5 };

// NOLINTNEXTLINE(fuchsia-overloaded-operator)
std::ostream &operator<<(std::ostream &os, DubinsPathType path_type);

enum class DubinsWordStatus {
  kSuccess,
  kNoPath, /* no connection between configurations with this word */
};

struct DubinsPath {
  /* the initial configuration */
  std::array<double, 3> qi;
  /* the lengths of the three segments */
  std::array<double, 3> param;
  /* model forward velocity / model angular velocity */
  double rho;
  /* the path type described */
  DubinsPathType type;
  /* total length */
  double total_length;

  std::string Describe() {
    std::stringstream message;
    message << "type: " << type << std::endl;
    message << "qi: " << qi.at(0) << " " << qi.at(1) << " " << qi.at(2) << std::endl;
    message << "lengths: " << param.at(0) << " " << param.at(1) << " " << param.at(2) << std::endl;
    message << "rho: " << rho << std::endl;
    return message.str();
  }
};

enum class DubinsStatus {
  kSuccess = 0,               /* No error */
  kPathParameterizationError, /* Path parameterisitation error */
                              //  kInvalidRho,                /* the rho value is invalid */
  kNoPath,                    /* no connection between configurations with this word */
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
                            std::array<double, 3> &out);
DubinsIntermediateResults ComputeDubinsIntermediateResults(const std::array<double, 3> &q0,
                                                           const std::array<double, 3> &q1,
                                                           double rho);

/**
 * Generate a path from an initial configuration to
 * a target configuration, with a specified maximum turning
 * radii
 *
 * A configuration is (x, y, theta), where theta is in radians, with zero
 * along the line x = 0, and counter-clockwise is positive
 *
 * @param path  - the resultant path
 * @param q0    - a configuration specified as an array of x, y, theta
 * @param q1    - a configuration specified as an array of x, y, theta
 * @param rho   - turning radius of the vehicle (forward velocity divided by maximum angular
 * velocity)
 * @return      - non-zero on error
 */
DubinsStatus DubinsShortestPath(DubinsPath &path, const std::array<double, 3> &q0,
                                const std::array<double, 3> &q1, double rho);

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
DubinsWordStatus ComputeDubinsPath(DubinsPath &path, const std::array<double, 3> &q0,
                                   const std::array<double, 3> &q1, double rho,
                                   DubinsPathType pathType);

/**
 * Calculate the configuration along the path, using the parameter t
 *
 * @param path - an initialised path
 * @param t    - a length measure, where 0 <= t < dubins_path_length(path)
 * @param q    - the configuration result
 * @returns    - non-zero if 't' is not in the correct range
 */
DubinsStatus DubinsPathSample(const DubinsPath &path, double t, std::array<double, 3> &q);

/**
 * Convenience function to identify the endpoint of a path
 *
 * @param path - an initialised path
 * @param q    - the configuration result
 */
DubinsStatus DubinsPathEndpoint(const DubinsPath &path, std::array<double, 3> &q);
