#include "se2_problem.hpp"

// inline glm::vec3 ToVec3(const glm::vec2 &v2, float z) { return glm::vec3(v2.x, v2.y, z); }

// std::vector<std::vector<bb3d::ColoredVec3> > DrawBridge(const DubinsPath &path, double ctg0,
// double ctg1, double z __attribute__((unused))) {
//
//  std::vector<std::vector<bb3d::ColoredVec3> > lines;
//  lines.push_back({});
//
//  constexpr int N = 20;
//  for (int j=0; j<N; j++) {
//    double t = 0.999*path.total_length * static_cast<double>(j) / (N - 1); // static_cast)
//    std::array<double, 3> q{};
//    DubinsStatus sample_ret = DubinsPathSample(path, t, q);
//    if (sample_ret != DubinsStatus::kSuccess) {
//      std::cerr << "bad return code for sampling: " << static_cast<int>(sample_ret) <<
//      std::endl;
//    //double q[3];
//    //int sample_ret = dubins_path_sample(&path, t, q);
//    //if (sample_ret != 0) {
//    //  std::cerr << "bad return code for sampling: " << sample_ret << std::endl;
//      std::exit(EXIT_FAILURE);
//    }
//    //if (first) {
//    //  std::cout << "t " << t << ": " << q[0] << ", " << q[1] << std::endl;
//    //}
//    double ctg = ctg0 * (1 - t) + ctg1 * t;
//    glm::vec4 color = {ctg, 0, 1 - ctg, 0.6};
//    //bb3d::ColoredVec3 v = {{q[0], q[1], z}, color};
//    bb3d::ColoredVec3 v = {{q[0], q[1], q[2]}, color};
//    if (j > 0) {
//      const std::vector<bb3d::ColoredVec3> &latest_vec = lines.at(lines.size()-1);
//      double prev_q = latest_vec.at(latest_vec.size() - 1).position.z;
//      if ((prev_q > 2.2 && q[2] < -2.2) || (prev_q < -2.2 && q[2] > 2.2)) {
//        lines.push_back({});
//      }
//    }
//    lines.at(lines.size()-1).push_back(v);
//  }
//  return lines;
//}
std::vector<XyzRgb> DrawBridge(const Se2Problem::DubinsPath &path, double ctg0, double ctg1,
                               double z __attribute__((unused))) {
  std::vector<XyzRgb> line;

  constexpr int kN = 20;
  for (int j = 0; j < kN; j++) {
    double t = 0.999 * path.TotalLength() * static_cast<double>(j) / (kN - 1);  // static_cast)
    Se2Problem::Se2Coord q = path.Sample(t);
    double ctg = ctg0 * (1 - t) + ctg1 * t;
    // std::cout << "t " << t << ": " << q[0] << ", " << q[1] << "   | " << ctg << std::endl;
    glm::vec4 color = {1 - ctg, 0, ctg, 0.6};
    XyzRgb v({q[0], q[1], z}, color);
    // XyzRgb v({q[0], q[1], q[2]}, color);

    line.push_back(v);
  }
  return line;
}

void DrawBridgeReverse(std::vector<XyzRgb> &line, const Se2Problem::DubinsPath &path,
                       double z __attribute__((unused))) {
  constexpr int kN = 20;
  for (int j = 0; j < kN; j++) {
    double t =
        0.999 * path.TotalLength() * static_cast<double>(kN - 1 - j) / (kN - 1);  // static_cast)
    Se2Problem::Se2Coord q = path.Sample(t);
    // if (first) {
    //  std::cout << "t " << t << ": " << q[0] << ", " << q[1] << std::endl;
    //}
    glm::vec3 v = {q[0], q[1], z};
    glm::vec4 color = {1, 1, 1, 1};
    // glm::vec3 v = {q[0], q[1], q[2] + 0.05};
    line.push_back(XyzRgb(v, color));
  }
}

std::vector<std::vector<XyzRgb> > Se2Problem::GetBridgeLines() const {
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
    const double ctg0 = cost_to_go.at(edge.parent_index_) / max_cost_to_go;
    const double ctg1 = cost_to_go.at(node_index) / max_cost_to_go;

    bridges.push_back(DrawBridge(edge.bridge_, ctg0, ctg1, 0));
    // std::vector<std::vector<bb3d::ColoredVec3> > next_bridge = DrawBridge(edge.bridge_, ctg0,
    // ctg1, 0); bridges.reserve(bridges.size() + next_bridge.size());
    // bridges.insert(bridges.end(), next_bridge.begin(), next_bridge.end());
    node_index++;
  }

  // TODO(greg): why did I used to sort these?
  // std::sort(bridges.begin(), bridges.end(),
  //           [](const std::vector<bb3d::ColoredVec3> &b0, const std::vector<bb3d::ColoredVec3>
  //           &b1) {
  //             return b0.at(b0.size() - 1).color[0] > b1.at(b1.size() - 1).color[0];
  //           });
  return bridges;
}

std::vector<std::vector<XyzRgb> > Se2Problem::GetGoalLine() const {
  const std::vector<double> cost_to_go = search_.ComputeCostsToGo();
  const std::vector<rrts::Edge<Point, Line> > &edges = search_.Edges();
  std::vector<XyzRgb> goal_line;

  double min_cost_to_go = 0;
  size_t winner_index = 0;
  bool got_winner = false;
  size_t index = 1;
  // best cost to go in a goal region
  for (const rrts::Edge<Point, Line> &edge : edges) {
    const Point &endpoint = edge.bridge_.Endpoint();
    if (InGoalRegion(endpoint) && (cost_to_go.at(index) < min_cost_to_go || !got_winner)) {
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
      DrawBridgeReverse(winning_route, edge.bridge_, -0.05);
      //        glm::vec3 p = static_cast<glm::dvec3>(edge.bridge_.p1);
      //        p.z -= 0.05F;
      // std::cerr << edge.bridge.p1.x << " " << edge.bridge.p1.y << " " << edge.bridge.p1.z <<
      // " " << std::endl; winning_route.push_back(p);
      head = edge.parent_index_;

      // if (head == 0) {
      //  glm::vec3 pf = static_cast<glm::dvec3>(edge.bridge_.p0);
      //  pf.z -= 0.02F;
      //  winning_route.push_back(pf);
      //}
    }
    segments.push_back(winning_route);
  }
  return segments;
}

Se2Problem Se2Problem::RandomProblem(std::mt19937_64 &rng_engine, const ProblemParameters &params,
                                     double rho) {
  std::uniform_real_distribution<double> uniform_distribution;
  std::function<double(void)> uniform = [&uniform_distribution, &rng_engine]() {
    return uniform_distribution(rng_engine);
  };
  std::function<Point(void)> uniform_p = [&uniform]() {
    return Point{{uniform(), uniform()}, uniform()};
  };

  const double dx = params.min_length + (params.max_length - params.min_length) * uniform();
  const double dy = params.min_length + (params.max_length - params.min_length) * uniform();
  const glm::dvec2 lb = {-dx / 2, -dy / 2};
  const glm::dvec2 ub = {dx / 2, dy / 2};
  Se2Coord x_init = {{lb[0], 0}, 0};

  const glm::dvec2 goal = {ub[0], dy * (uniform() - 0.5)};
  Sphere goal_region = {goal, params.goal_radius};

  // obstacles
  const size_t n =
      static_cast<size_t>(std::uniform_int_distribution<>(1, params.max_num_obstacles)(rng_engine));
  const double total_volume = dx * dy;
  // upper bound (no overlap)
  const double volume_per_obstacle =
      total_volume * params.obstacle_fraction / static_cast<double>(n);
  const double radius_per_obstacle = sqrt(volume_per_obstacle / M_PI);

  int32_t num_failed_insertions = 0;
  std::vector<Sphere> obstacles;
  while (obstacles.size() < n) {
    const double obstacle_radius = radius_per_obstacle * (0.5 + 0.5 * uniform());
    const glm::dvec2 p = {dx * (uniform() - 0.5), dy * (uniform() - 0.5)};
    const bool avoids_goal = glm::distance(p, goal) > obstacle_radius + params.goal_radius;
    const bool avoids_start =
        glm::distance(p, x_init.position) > obstacle_radius + 1.3;  // careful of turning radius
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

  return Se2Problem(x_init, rho, params.eta, lb, ub, goal_region, obstacles);
}
