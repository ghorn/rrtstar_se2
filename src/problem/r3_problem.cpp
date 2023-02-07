#include "r3_problem.hpp"

std::vector<std::vector<XyzRgb> > R3Problem::GetBridgeLines() const {
  const std::vector<double> cost_to_go = search_.ComputeCostsToGo();
  const std::vector<rrts::Edge<Point, Line> > &edges = search_.Edges();

  // Find max cost to go in order to scale lines
  double max_cost_to_go = 0;
  for (double c : cost_to_go) {
    max_cost_to_go = std::max(max_cost_to_go, c);
  }

  // Draw all bridges
  std::vector<std::vector<XyzRgb> > bridges;
  size_t node_index = 1;
  for (const rrts::Edge<Point, Line> &edge : edges) {
    XyzRgb cv0{};
    XyzRgb cv1{};
    cv0.x = static_cast<float>(edge.bridge_.p0.x);
    cv0.y = static_cast<float>(edge.bridge_.p0.y);
    cv0.z = static_cast<float>(edge.bridge_.p0.z);

    cv1.x = static_cast<float>(edge.bridge_.p1.x);
    cv1.y = static_cast<float>(edge.bridge_.p1.y);
    cv1.z = static_cast<float>(edge.bridge_.p1.z);
    const float ctg0 = static_cast<float>(cost_to_go.at(edge.parent_index_) / max_cost_to_go);
    const float ctg1 = static_cast<float>(cost_to_go.at(node_index) / max_cost_to_go);
    // cv0.color = r0 = {1-ctg0, 0, ctg0, 0.6};
    // cv1.color = {1 - ctg1, 0, ctg1, 0.6};
    float r0 = 1 - ctg0;
    float g0 = 0;
    float b0 = ctg0;
    float a0 = 0.6f;
    float r1 = 1 - ctg1;
    float g1 = 0;
    float b1 = ctg1;
    float a1 = 0.6f;
    cv0.r = r0;
    cv0.g = g0;
    cv0.b = b0;
    cv0.a = a0;

    cv1.r = r1;
    cv1.g = g1;
    cv1.b = b1;
    cv1.a = a1;

    bridges.push_back({cv0, cv1});
    node_index++;
  }

  // // I don't remember why I used to sort this...
  // std::sort(
  //     bridges.begin(), bridges.end(),
  //     [](const std::vector<bb3d::ColoredVec3> &b0, const std::vector<bb3d::ColoredVec3> &b1) {
  //       return b0.at(b0.size() - 1).color[0] > b1.at(b1.size() - 1).color[0];
  //     });

  return bridges;
}

std::vector<std::vector<XyzRgb> > R3Problem::GetGoalLine() const {
  const std::vector<double> cost_to_go = search_.ComputeCostsToGo();
  const std::vector<rrts::Edge<Point, Line> > &edges = search_.Edges();
  std::vector<XyzRgb> goal_line;

  double min_cost_to_go = 0;
  size_t winner_index = 0;
  bool got_winner = false;
  size_t index = 1;
  // best cost to go in a goal region
  for (const rrts::Edge<Point, Line> &edge : edges) {
    if (InGoalRegion(edge.bridge_.p1) && (cost_to_go.at(index) < min_cost_to_go || !got_winner)) {
      winner_index = index;
      got_winner = true;
      min_cost_to_go = cost_to_go.at(index);
    }
    index++;
  }

  // trace back route from winner
  std::vector<std::vector<XyzRgb> > segments;
  if (got_winner) {
    std::vector<XyzRgb> winning_route;
    size_t head = winner_index;

    while (head != 0) {
      rrts::Edge<Point, Line> edge = edges.at(head - 1);
      glm::vec3 p = static_cast<glm::dvec3>(edge.bridge_.p1);
      // HACK - offset z to make sure we can see the line
      p.z -= 0.05F;
      winning_route.push_back(XyzRgb(p, 1, 1, 1, 1));
      head = edge.parent_index_;

      if (head == 0) {
        glm::vec3 pf = static_cast<glm::dvec3>(edge.bridge_.p0);
        pf.z -= 0.02F;
        winning_route.push_back(XyzRgb(pf, 1, 1, 1, 1));
      }
    }
    segments.push_back(winning_route);
  }

  return segments;
}

R3Problem R3Problem::RandomProblem(std::mt19937_64 &rng_engine, const ProblemParameters &params) {
  std::uniform_real_distribution<double> uniform_distribution;
  std::function<double(void)> uniform = [&uniform_distribution, &rng_engine]() {
    return uniform_distribution(rng_engine);
  };
  std::function<Point(void)> uniform_p = [&uniform]() {
    return Point{uniform(), uniform(), uniform()};
  };

  const double dx = params.min_length + (params.max_length - params.min_length) * uniform();
  const double dy = params.min_length + (params.max_length - params.min_length) * uniform();
  const double dz = params.min_length + (params.max_length - params.min_length) * uniform();
  const Point lb = {-dx / 2, -dy / 2, -dz / 2};
  const Point ub = {dx / 2, dy / 2, dz / 2};
  const Point x_init = {lb[0], 0, 0};
  const Point goal = {ub[0], dy * (uniform() - 0.5), dz * (uniform() - 0.5)};
  Sphere goal_region = {goal, params.goal_radius};

  // obstacles
  const size_t n = static_cast<size_t>(
      std::uniform_int_distribution<>(1, params.max_num_obstacles + 1)(rng_engine));
  const double total_volume = dx * dy * dz;
  // upper bound (no overlap)
  const double volume_per_obstacle =
      total_volume * params.obstacle_fraction / static_cast<double>(n);
  const double radius_per_obstacle = pow(3 * volume_per_obstacle / (4 * M_PI), 1.0 / 3.0);
  std::vector<Sphere> obstacles;

  int32_t num_failed_insertions = 0;
  while (obstacles.size() < n) {
    const double obstacle_radius = radius_per_obstacle * (0.7 + 0.7 * uniform());
    const Point p = {dx * (uniform() - 0.5), dy * (uniform() - 0.5), dz * (uniform() - 0.5)};
    const bool avoids_goal =
        glm::distance(glm::dvec3(p), glm::dvec3(goal)) > obstacle_radius + params.goal_radius;
    const bool avoids_start = glm::distance(x_init, glm::dvec3(p)) > obstacle_radius + 0.5;
    if (avoids_goal && avoids_start) {
      Sphere obstacle = {p, obstacle_radius};
      obstacles.push_back(obstacle);
    } else {
      num_failed_insertions++;
      if (num_failed_insertions >= 1000) {
        std::cerr << "Failed to create " << n << " obstacles after " << num_failed_insertions
                  << " attempts. Giving up." << std::endl;
        break;
      }
    }
  }

  return R3Problem(x_init, lb, ub, goal_region, obstacles, params.eta);
}

//   void UpdateBoundingBoxLines(bb3d::Lines &bounding_box_lines) const {
//     std::vector<std::vector<glm::vec3> > bb_lines;
//     bb_lines.push_back({{lb_.x, lb_.y, lb_.z}, {ub_.x, lb_.y, lb_.z}});
//     bb_lines.push_back({{lb_.x, lb_.y, ub_.z}, {ub_.x, lb_.y, ub_.z}});
//     bb_lines.push_back({{lb_.x, ub_.y, lb_.z}, {ub_.x, ub_.y, lb_.z}});
//     bb_lines.push_back({{lb_.x, ub_.y, ub_.z}, {ub_.x, ub_.y, ub_.z}});

//     bb_lines.push_back({{lb_.x, lb_.y, lb_.z}, {lb_.x, ub_.y, lb_.z}});
//     bb_lines.push_back({{lb_.x, lb_.y, ub_.z}, {lb_.x, ub_.y, ub_.z}});
//     bb_lines.push_back({{ub_.x, lb_.y, lb_.z}, {ub_.x, ub_.y, lb_.z}});
//     bb_lines.push_back({{ub_.x, lb_.y, ub_.z}, {ub_.x, ub_.y, ub_.z}});

//     bb_lines.push_back({{lb_.x, lb_.y, lb_.z}, {lb_.x, lb_.y, ub_.z}});
//     bb_lines.push_back({{lb_.x, ub_.y, lb_.z}, {lb_.x, ub_.y, ub_.z}});
//     bb_lines.push_back({{ub_.x, lb_.y, lb_.z}, {ub_.x, lb_.y, ub_.z}});
//     bb_lines.push_back({{ub_.x, ub_.y, lb_.z}, {ub_.x, ub_.y, ub_.z}});
//     bounding_box_lines.Update(bb_lines);
//   }
