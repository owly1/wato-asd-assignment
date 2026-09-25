#include "grid_search.hpp"
#include <iostream>
#include <stdexcept>

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

int main() {
  robot::GridSearchOptions options;
  options.resolution = 1.0;
  options.robot_radius = 0.0;
  std::vector<std::int8_t> cells(49, 0);
  auto path = robot::findGridPath(cells, 7, 7, 0, 48, options);
  require(path.size() == 7 && path.front() == 0 && path.back() == 48,
          "Open grid should produce a shortest diagonal route");

  for (int y = 0; y < 6; ++y) cells[y * 7 + 3] = 100;
  path = robot::findGridPath(cells, 7, 7, 7, 13, options);
  require(!path.empty(), "Planner should go around a wall through the gap");
  require(std::find(path.begin(), path.end(), 45) != path.end(), "Route must use wall gap");
  for (std::size_t i = 1; i < path.size(); ++i) {
    const int previous = path[i-1], current = path[i];
    require(cells[current] != 100, "Route crossed an occupied cell");
    const int dx = current % 7 - previous % 7, dy = current / 7 - previous / 7;
    require(std::abs(dx) <= 1 && std::abs(dy) <= 1, "Route contains a non-adjacent step");
    if (dx && dy) require(cells[previous + dx] != 100 && cells[previous + dy*7] != 100,
                         "Route cut an obstacle corner");
  }
  cells[45] = 100;
  require(robot::findGridPath(cells, 7, 7, 7, 13, options).empty(),
          "A closed wall must have no route");
  require(robot::findGridPath({0, 100, 100, 0}, 2, 2, 0, 3, options).empty(),
          "Diagonal corner cutting must be rejected");
  require(robot::findGridPath({0, -1, 0}, 3, 1, 0, 2, options).size() == 3,
          "Unknown cells should be traversable when enabled");
  options.allow_unknown = false;
  require(robot::findGridPath({0, -1, 0}, 3, 1, 0, 2, options).empty(),
          "Unknown cells must block a conservative search");
  options.allow_unknown = true;

  cells.assign(49, 0);
  cells[24] = 100;
  options.robot_radius = 1.0;
  require(robot::findGridPath(cells, 7, 7, 23, 8, options).empty(),
          "Robot footprint must not overlap an obstacle at the start");
  options.robot_radius = 0.0;
  require(!robot::findGridPath(cells, 7, 7, 23, 8, options).empty(),
          "The same point-sized robot should fit");
  require(robot::findGridPath(cells, 7, 7, 24, 8, options).empty(), "Blocked start accepted");
  require(robot::findGridPath(cells, 7, 7, 8, 24, options).empty(), "Blocked goal accepted");
  require(robot::findGridPath(cells, 7, 7, -1, 8, options).empty(), "Invalid start accepted");
  require(robot::findGridPath(cells, 7, 7, 8, 49, options).empty(), "Invalid goal accepted");
  require(robot::findGridPath(cells, 7, 7, 8, 8, options).size() == 1, "Same-cell goal failed");

  cells.assign(25, 0);
  cells[11] = cells[12] = cells[13] = -1;
  path = robot::findGridPath(cells, 5, 5, 10, 14, options);
  require(!path.empty(), "Known-space detour failed");
  for (int cell : path) require(cells[cell] == 0, "Unknown penalty should favor clear detour");
  cells[11] = cells[12] = cells[13] = 90;
  path = robot::findGridPath(cells, 5, 5, 10, 14, options);
  for (int cell : path) require(cells[cell] == 0, "Inflation costs should favor clear detour");

  std::cout << "Planner checks passed: routing, walls, corners, footprint, costs, and invalid inputs\n";
}
