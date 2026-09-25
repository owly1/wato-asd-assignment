#ifndef GRID_SEARCH_HPP_
#define GRID_SEARCH_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace robot {

struct GridSearchOptions {
  double resolution = 0.1;
  double robot_radius = 1.5;
  int occupied_threshold = 100;
  bool allow_unknown = true;
  double unknown_penalty = 3.0;
  double cost_weight = 2.0;
  double clearance_distance = 0.8;
  double clearance_weight = 3.0;
};

// A* is independent of ROS so its route and collision rules can be tested directly.
inline std::vector<int> findGridPath(const std::vector<std::int8_t>& cells,
                                   int width, int height, int start, int goal,
                                   const GridSearchOptions& options,
                                   const char** failure = nullptr) {
  if (failure) *failure = "invalid grid or search parameters";
  if (width <= 0 || height <= 0 ||
      cells.size() != static_cast<std::size_t>(width) * height ||
      cells.size() > static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
      start < 0 || goal < 0 || static_cast<std::size_t>(start) >= cells.size() ||
      static_cast<std::size_t>(goal) >= cells.size() ||
      !std::isfinite(options.resolution) || options.resolution <= 0.0 ||
      !std::isfinite(options.robot_radius) || options.robot_radius < 0.0 ||
      !std::isfinite(options.unknown_penalty) || options.unknown_penalty < 0.0 ||
      !std::isfinite(options.cost_weight) || options.cost_weight < 0.0 ||
      !std::isfinite(options.clearance_distance) || options.clearance_distance < 0.0 ||
      !std::isfinite(options.clearance_weight) || options.clearance_weight < 0.0 ||
      options.occupied_threshold < 1 || options.occupied_threshold > 100) return {};

  std::vector<bool> blocked(cells.size(), false);
  // Include a cell-diagonal margin for the finite size of occupied grid squares.
  const double radius = options.robot_radius > 0.0 ?
    options.robot_radius / options.resolution + std::sqrt(2.0) : 0.0;
  if (radius > std::max(width, height)) return {};
  // Penalize a band beyond the collision boundary. The local costmap inflation
  // is narrower than the footprint, so its costs alone cannot provide this gap.
  const double band = options.clearance_distance / options.resolution;
  const double search_radius = std::min(radius + band,
    std::hypot(static_cast<double>(width), static_cast<double>(height)));
  const int extent = static_cast<int>(std::ceil(search_radius));
  std::vector<double> clearance_cost(cells.size(), 0.0);
  std::vector<std::pair<int, int>> offsets;
  for (int dy = -extent; dy <= extent; ++dy)
    for (int dx = -extent; dx <= extent; ++dx)
      if (std::hypot(dx, dy) <= search_radius) offsets.emplace_back(dx, dy);

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const int index = y * width + x;
      const auto value = cells[index];
      if ((!options.allow_unknown && value < 0) || value < -1 || value > 100)
        blocked[index] = true;
      // Keep the robot's footprint inside the map's physical bounds.
      if (std::min({x + 0.5, y + 0.5, width - x - 0.5, height - y - 0.5}) *
          options.resolution < options.robot_radius) blocked[index] = true;
      if (value < options.occupied_threshold) continue;
      for (const auto& offset : offsets) {
        const int nx = x + offset.first, ny = y + offset.second;
        if (nx < 0 || ny < 0 || nx >= width || ny >= height) continue;
        const int target = ny * width + nx;
        const double distance = std::hypot(offset.first, offset.second);
        if (distance <= radius) blocked[target] = true;
        else if (band > 0.0)
          clearance_cost[target] = std::max(clearance_cost[target],
            options.clearance_weight * (1.0 - (distance-radius)/band));
      }
    }
  }
  if (blocked[start]) {
    if (failure) *failure = "robot position is inside the obstacle-clearance boundary";
    return {};
  }
  if (blocked[goal]) {
    if (failure) *failure = "goal is inside the obstacle-clearance boundary";
    return {};
  }
  if (failure) *failure = "start and goal are disconnected";

  struct Entry { double score; double cost; int cell; };
  const auto later = [](const Entry& a, const Entry& b) { return a.score > b.score; };
  std::priority_queue<Entry, std::vector<Entry>, decltype(later)> open(later);
  std::vector<double> costs(cells.size(), std::numeric_limits<double>::infinity());
  std::vector<int> parent(cells.size(), -1);
  const auto heuristic = [&](int cell) {
    return std::hypot(cell % width - goal % width, cell / width - goal / width);
  };
  costs[start] = 0.0;
  open.push({heuristic(start), 0.0, start});
  while (!open.empty()) {
    const Entry current = open.top();
    open.pop();
    if (current.cost > costs[current.cell]) continue;
    if (current.cell == goal) {
      std::vector<int> path;
      for (int cell = goal; cell != -1; cell = parent[cell]) path.push_back(cell);
      std::reverse(path.begin(), path.end());
      if (failure) *failure = nullptr;
      return path;
    }
    const int x = current.cell % width, y = current.cell / width;
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) continue;
        const int nx = x + dx, ny = y + dy;
        if (nx < 0 || ny < 0 || nx >= width || ny >= height) continue;
        const int next = ny * width + nx;
        if (blocked[next]) continue;
        // A diagonal move must not squeeze between two blocked squares.
        if (dx != 0 && dy != 0 &&
            (blocked[y * width + nx] || blocked[ny * width + x])) continue;
        const double penalty = cells[next] < 0 ? options.unknown_penalty :
          options.cost_weight * static_cast<double>(cells[next]) / 100.0;
        const double cost = current.cost + std::hypot(dx, dy) * (1.0 + penalty + clearance_cost[next]);
        if (cost >= costs[next]) continue;
        costs[next] = cost;
        parent[next] = current.cell;
        open.push({cost + heuristic(next), cost, next});
      }
    }
  }
  return {};  // No reachable route: the caller publishes an empty path.
}

}  // namespace robot
#endif
