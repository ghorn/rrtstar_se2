#include "se2_problem.hpp"

std::vector<XyzRgb> DrawBridge(const Se2Problem::DubinsPath &path, double ctg0, double ctg1,
                               double z) {
  std::vector<XyzRgb> line;

  constexpr int kN = 20;
  for (int j = 0; j < kN; j++) {
    double t = 0.999 * static_cast<double>(j) / (kN - 1);
    Se2Problem::Se2Coord q = path.Sample(t * path.TotalLength());
    double ctg = ctg0 * (1 - t) + ctg1 * t;
    // std::cout << "t " << t << ": " << q[0] << ", " << q[1] << "   | " << ctg << std::endl;
    glm::vec4 color = {1 - ctg, 0, ctg, 1};
    XyzRgb v({q[0], q[1], z}, color);
    // XyzRgb v({q[0], q[1], q[2]}, color);

    line.push_back(v);
  }
  return line;
}

void DrawBridgeReverse(std::vector<XyzRgb> &line, const glm::vec4 &color,
                       const Se2Problem::DubinsPath &path, double z) {
  constexpr int kN = 20;
  for (int j = 0; j < kN; j++) {
    double t = 0.999 * path.TotalLength() * static_cast<double>(kN - 1 - j) / (kN - 1);
    Se2Problem::Se2Coord q = path.Sample(t);
    glm::vec3 v = {q[0], q[1], z};
    // glm::vec3 v = {q[0], q[1], q[2] + 0.05};
    line.push_back(XyzRgb(v, color));
  }
}

std::vector<std::vector<XyzRgb>> Se2Problem::ComputeBridgeLines(
    const std::vector<double> &cost_to_go,
    const std::vector<rrts::Edge<Point, Line>> &edges) const {
  // Find max cost to go in order to scale lines
  double max_cost_to_go = 0;
  for (double c : cost_to_go) {
    max_cost_to_go = std::max(max_cost_to_go, c);
  }

  // Draw all bridges
  std::vector<std::vector<XyzRgb>> bridges;
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

std::vector<std::vector<XyzRgb>> Se2Problem::ComputeGoalLine(
    const std::vector<double> &cost_to_go, const std::vector<rrts::Edge<Point, Line>> &edges,
    const glm::vec3 &color) const {
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
  std::vector<std::vector<XyzRgb>> segments;
  if (got_winner) {
    std::vector<XyzRgb> winning_route;
    size_t head = winner_index;
    const glm::vec4 color_with_alpha = {color.x, color.y, color.z, 1};

    while (head != 0) {
      rrts::Edge<Point, Line> edge = edges.at(head - 1);
      DrawBridgeReverse(winning_route, color_with_alpha, edge.bridge_, -0.05);
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

#ifdef __EMSCRIPTEN__
void Se2Problem::SetPathLines(const glm::vec3 &goal_line_color, emscripten::val positions,
                              emscripten::val colors, emscripten::val indices,
                              size_t max_points) const {
  // compute bridge and goal lines
  const std::vector<double> cost_to_go = search_.ComputeCostsToGo();
  const std::vector<rrts::Edge<Point, Line>> &edges = search_.Edges();

  const std::vector<std::vector<XyzRgb>> bridge_lines = ComputeBridgeLines(cost_to_go, edges);
  const std::vector<std::vector<XyzRgb>> goal_lines =
      ComputeGoalLine(cost_to_go, edges, goal_line_color);

  // TODO(greg): check max points
  (void)max_points;

  // now copy the lines out
  size_t count = 0;

  // first copy every line segment to flat vectors
  std::vector<float> positions_vec;
  std::vector<float> colors_vec;

  for (const std::vector<std::vector<XyzRgb>> &lines : {bridge_lines, goal_lines}) {
    for (const std::vector<XyzRgb> &line : lines) {
      for (size_t k = 0; k < line.size(); k++) {
        const XyzRgb &cv = line.at(k);

        if (k < line.size() - 1) {
          indices.call<void>("push", count);
          indices.call<void>("push", count + 1);
        }

        positions_vec.push_back(cv.x);
        positions_vec.push_back(cv.y);
        positions_vec.push_back(cv.z);

        colors_vec.push_back(cv.r);
        colors_vec.push_back(cv.g);
        colors_vec.push_back(cv.b);
        colors_vec.push_back(cv.a);

        count++;
      }
    }
  }

  // finally, make memory view of the flat vectors and copy to the javascript TypedArrays
  emscripten::val positions_view =
      emscripten::val(emscripten::typed_memory_view(positions_vec.size(), positions_vec.data()));
  emscripten::val colors_view =
      emscripten::val(emscripten::typed_memory_view(colors_vec.size(), colors_vec.data()));

  positions.call<void>("set", positions_view);
  colors.call<void>("set", colors_view);

  // return true;
}
#endif

std::vector<std::vector<XyzRgb>> Se2Problem::GetBoundingBoxLines(float bounding_box_opacity) const {
  const glm::vec2 lb = Lb().position;
  const glm::vec2 ub = Ub().position;
  std::vector<std::vector<glm::vec3>> bb_lines;
  bb_lines.push_back({{lb.x, lb.y, 0}, {lb.x, ub.y, 0}});
  bb_lines.push_back({{lb.x, ub.y, 0}, {ub.x, ub.y, 0}});
  bb_lines.push_back({{ub.x, ub.y, 0}, {ub.x, lb.y, 0}});
  bb_lines.push_back({{ub.x, lb.y, 0}, {lb.x, lb.y, 0}});

  // add a color to each one
  const glm::vec4 color = {1, 1, 1, bounding_box_opacity};
  std::vector<std::vector<XyzRgb>> ret;
  for (const std::vector<glm::vec3> &segment : bb_lines) {
    std::vector<XyzRgb> colored_segment;
    for (const glm::vec3 &point : segment) {
      colored_segment.push_back(XyzRgb(point, color));
    }
    ret.push_back(colored_segment);
  }
  return ret;
}

Se2Problem Se2Problem::RandomProblem(std::mt19937_64 &rng_engine, const ProblemParameters &params,
                                     double rho) {
  std::uniform_real_distribution<double> uniform_distribution;
  std::function<double(void)> uniform = [&uniform_distribution, &rng_engine]() {
    return uniform_distribution(rng_engine);
  };

  const double dx = params.min_length + (params.max_length - params.min_length) * uniform();
  const double dy = params.min_length + (params.max_length - params.min_length) * uniform();
  const glm::dvec2 lb = {-dx / 2, -dy / 2};
  const glm::dvec2 ub = {dx / 2, dy / 2};
  Se2Coord x_init = {{lb[0], 0}, 0};

  const glm::dvec2 goal = {ub[0], dy * (uniform() - 0.5)};
  Sphere goal_region = {goal, params.goal_radius};

  // obstacles
  std::vector<Sphere> obstacles;
  if (params.max_num_obstacles > 0) {
    const size_t n = static_cast<size_t>(
        std::uniform_int_distribution<>(1, params.max_num_obstacles)(rng_engine));
    const double total_volume = dx * dy;
    // upper bound (no overlap)
    const double volume_per_obstacle =
        total_volume * params.obstacle_fraction / static_cast<double>(n);
    const double radius_per_obstacle = sqrt(volume_per_obstacle / M_PI);

    int32_t num_failed_insertions = 0;
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
  }

  return Se2Problem(x_init, rho, params.eta, lb, ub, goal_region, obstacles);
}
