#include "xyzq_problem.hpp"

#include "src/space/se2.hpp"

std::vector<XyzRgb> DrawBridge(const XyzqProblem::XyzqPath &path, double ctg0, double ctg1) {
  std::vector<XyzRgb> line;

  constexpr int kN = 20;
  for (int j = 0; j < kN; j++) {
    double t = 0.999 * static_cast<double>(j) / (kN - 1);
    XyzqProblem::XyzqCoord q = path.Sample(t * path.TotalLength());
    double ctg = ctg0 * (1 - t) + ctg1 * t;
    // std::cout << "t " << t << ": " << q[0] << ", " << q[1] << "   | " << ctg << std::endl;
    glm::vec4 color = {1 - ctg, 0, ctg, 1};
    XyzRgb v({q.se2.position[0], q.se2.position[1], q.z}, color);

    line.push_back(v);
  }
  return line;
}

void DrawBridgeReverse(std::vector<XyzRgb> &line, const glm::vec4 &color,
                       const XyzqProblem::XyzqPath &path) {
  constexpr int kN = 20;
  for (int j = 0; j < kN; j++) {
    double t = 0.999 * path.TotalLength() * static_cast<double>(kN - 1 - j) / (kN - 1);
    XyzqProblem::XyzqCoord q = path.Sample(t);
    glm::vec3 v = {q.se2.position[0], q.se2.position[1], q.z - 0.05};
    // glm::vec3 v = {q[0], q[1], q[2] + 0.05};
    line.push_back(XyzRgb(v, color));
  }
}

std::vector<std::vector<XyzRgb>> XyzqProblem::ComputeBridgeLines(
    const std::vector<double> &cost_to_go,
    const std::vector<rrts::Edge<Point, XyzqPath>> &edges) const {
  // Find max cost to go in order to scale lines
  double max_cost_to_go = 0;
  for (double c : cost_to_go) {
    max_cost_to_go = std::max(max_cost_to_go, c);
  }

  // Draw all bridges
  std::vector<std::vector<XyzRgb>> bridges;
  size_t node_index = 1;
  for (const rrts::Edge<Point, XyzqPath> &edge : edges) {
    const double ctg0 = cost_to_go.at(edge.parent_index_) / max_cost_to_go;
    const double ctg1 = cost_to_go.at(node_index) / max_cost_to_go;

    bridges.push_back(DrawBridge(edge.bridge_, ctg0, ctg1));
    node_index++;
  }

  return bridges;
}

std::vector<std::vector<XyzRgb>> XyzqProblem::ComputeGoalLine(
    const std::vector<double> &cost_to_go, const std::vector<rrts::Edge<Point, XyzqPath>> &edges,
    const glm::vec3 &color) const {
  std::vector<XyzRgb> goal_line;

  double min_cost_to_go = 0;
  size_t winner_index = 0;
  bool got_winner = false;
  size_t index = 1;
  // best cost to go in a goal region
  for (const rrts::Edge<Point, XyzqPath> &edge : edges) {
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
      rrts::Edge<Point, XyzqPath> edge = edges.at(head - 1);
      DrawBridgeReverse(winning_route, color_with_alpha, edge.bridge_);
      head = edge.parent_index_;
    }
    segments.push_back(winning_route);
  }
  return segments;
}

#ifdef __EMSCRIPTEN__
void XyzqProblem::SetPathLines(const glm::vec3 &goal_line_color, emscripten::val positions,
                               emscripten::val colors, emscripten::val indices,
                               size_t max_points) const {
  // compute bridge and goal lines
  const std::vector<double> cost_to_go = search_.ComputeCostsToGo();
  const std::vector<rrts::Edge<Point, XyzqPath>> &edges = search_.Edges();

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

std::vector<std::vector<XyzRgb>> XyzqProblem::GetBoundingBoxLines(
    float bounding_box_opacity) const {
  const glm::vec3 lb = Lb().Xyz();
  const glm::vec3 ub = Ub().Xyz();

  std::vector<std::vector<glm::vec3>> bb_lines;
  bb_lines.push_back({{lb.x, lb.y, lb.z}, {ub.x, lb.y, lb.z}});
  bb_lines.push_back({{lb.x, lb.y, ub.z}, {ub.x, lb.y, ub.z}});
  bb_lines.push_back({{lb.x, ub.y, lb.z}, {ub.x, ub.y, lb.z}});
  bb_lines.push_back({{lb.x, ub.y, ub.z}, {ub.x, ub.y, ub.z}});

  bb_lines.push_back({{lb.x, lb.y, lb.z}, {lb.x, ub.y, lb.z}});
  bb_lines.push_back({{lb.x, lb.y, ub.z}, {lb.x, ub.y, ub.z}});
  bb_lines.push_back({{ub.x, lb.y, lb.z}, {ub.x, ub.y, lb.z}});
  bb_lines.push_back({{ub.x, lb.y, ub.z}, {ub.x, ub.y, ub.z}});

  bb_lines.push_back({{lb.x, lb.y, lb.z}, {lb.x, lb.y, ub.z}});
  bb_lines.push_back({{lb.x, ub.y, lb.z}, {lb.x, ub.y, ub.z}});
  bb_lines.push_back({{ub.x, lb.y, lb.z}, {ub.x, lb.y, ub.z}});
  bb_lines.push_back({{ub.x, ub.y, lb.z}, {ub.x, ub.y, ub.z}});

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

XyzqProblem XyzqProblem::RandomProblem(std::mt19937_64 &rng_engine, const ProblemParameters &params,
                                       double rho, double max_glideslope) {
  std::uniform_real_distribution<double> uniform_distribution;
  std::function<double(void)> uniform = [&uniform_distribution, &rng_engine]() {
    return uniform_distribution(rng_engine);
  };
  const double dx = params.min_length + (params.max_length - params.min_length) * uniform();
  const double dy = params.min_length + (params.max_length - params.min_length) * uniform();
  const double dz = (params.min_length + (params.max_length - params.min_length) * uniform()) /
                    max_glideslope * 4;
  const glm::dvec3 lb = {-dx / 2, -dy / 2, -dz / 2};
  const glm::dvec3 ub = {dx / 2, dy / 2, dz / 2};
  XyzqCoord x_init = {rrts::space::se2::Se2Coord{{lb[0], 0}, 0}, lb[2]};

  const glm::dvec3 goal = {ub[0] - params.goal_radius, dy * (uniform() - 0.5), ub[2]};
  R3Sphere goal_region = {goal, params.goal_radius};

  // obstacles
  std::vector<R3Sphere> obstacles;
  if (params.max_num_obstacles > 0) {
    const size_t n = static_cast<size_t>(
        std::uniform_int_distribution<>(1, params.max_num_obstacles)(rng_engine));
    const double total_volume = dx * dy * dz;
    // upper bound (no overlap)
    const double volume_per_obstacle =
        total_volume * params.obstacle_fraction / static_cast<double>(n);
    const double radius_per_obstacle = sqrt(volume_per_obstacle / M_PI);

    int32_t num_failed_insertions = 0;
    while (obstacles.size() < n) {
      const double obstacle_radius = radius_per_obstacle * (0.5 + 0.5 * uniform());
      const glm::dvec3 p = {dx * (uniform() - 0.5), dy * (uniform() - 0.5), dz * (uniform() - 0.5)};
      const bool avoids_goal = glm::distance(p, goal) > obstacle_radius + params.goal_radius;
      const bool avoids_start =
          glm::distance(p, x_init.Xyz()) > obstacle_radius + 1.3;  // careful of turning radius
      if (avoids_goal && avoids_start) {
        R3Sphere obstacle = {p, obstacle_radius};
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

  return XyzqProblem(x_init, rho, max_glideslope, params.eta, lb, ub, goal_region, obstacles);
}
