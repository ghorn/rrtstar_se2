#pragma once

#include <cstdint>

struct ProblemParameters {
  double eta;
  int32_t max_num_obstacles;
  double obstacle_fraction;
  double min_length;
  double max_length;
  double goal_radius;
};
