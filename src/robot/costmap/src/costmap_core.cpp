#include "costmap_core.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}

nav_msgs::msg::OccupancyGrid CostmapCore::build(
  const sensor_msgs::msg::LaserScan& scan,
  double resolution, int grid_size, double inflation_radius) const {
  nav_msgs::msg::OccupancyGrid grid;
  // Keep the scan's timestamp and frame: coordinates are relative to the lidar.
  grid.header = scan.header;
  grid.info.map_load_time = scan.header.stamp;
  grid.info.resolution = resolution;
  grid.info.width = grid_size;
  grid.info.height = grid_size;
  const double half_size = grid_size * resolution / 2.0;
  grid.info.origin.position.x = -half_size;
  grid.info.origin.position.y = -half_size;
  grid.info.origin.orientation.w = 1.0;
  // -1 = unseen, 0 = clear, 100 = obstacle; intermediate values are a buffer.
  grid.data.assign(static_cast<std::size_t>(grid_size) * grid_size, -1);
  if (!std::isfinite(scan.range_max) || scan.range_max <= 0.0 ||
      !std::isfinite(scan.range_min) || scan.range_min < 0.0 ||
      scan.range_min >= scan.range_max || !std::isfinite(scan.angle_min) ||
      !std::isfinite(scan.angle_increment)) {
    return grid;
  }

  const auto cell = [&](double x, double y, int& gx, int& gy) {
    gx = static_cast<int>(std::floor((x + half_size) / resolution));
    gy = static_cast<int>(std::floor((y + half_size) / resolution));
    return gx >= 0 && gy >= 0 && gx < grid_size && gy < grid_size;
  };
  std::vector<std::pair<int, int>> obstacles;
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double range = scan.ranges[i];
    if (std::isnan(range) || range < scan.range_min) continue;
    const bool hit = std::isfinite(range) && range < scan.range_max;
    const double distance = std::min(range, static_cast<double>(scan.range_max));
    const double angle = scan.angle_min + i * static_cast<double>(scan.angle_increment);
    if (!std::isfinite(angle)) continue;
    const double dx = std::cos(angle);
    const double dy = std::sin(angle);

    // Only mark cells along measured beams as clear, preserving unseen areas.
    const double ray_length = std::min(distance, std::sqrt(2.0) * half_size);
    for (double r = 0.0; r < ray_length; r += resolution / 2.0) {
      int gx, gy;
      if (!cell(r * dx, r * dy, gx, gy)) break;
      auto& value = grid.data[gy * grid_size + gx];
      if (value != 100) value = 0;
    }
    // A reading at maximum range (or +infinity) does not indicate an obstacle.
    if (hit && distance <= std::sqrt(2.0) * half_size) {
      int gx, gy;
      if (cell(distance * dx, distance * dy, gx, gy)) {
        auto& value = grid.data[gy * grid_size + gx];
        if (value != 100) obstacles.emplace_back(gx, gy);
        value = 100;
      }
    }
  }

  // Add a fading buffer around obstacles after tracing all beams.
  const int radius_cells = static_cast<int>(std::ceil(inflation_radius / resolution));
  for (const auto& obstacle : obstacles) {
    for (int oy = -radius_cells; oy <= radius_cells; ++oy) {
      for (int ox = -radius_cells; ox <= radius_cells; ++ox) {
        const int gx = obstacle.first + ox;
        const int gy = obstacle.second + oy;
        const double distance = std::hypot(ox, oy) * resolution;
        if (gx < 0 || gy < 0 || gx >= grid_size || gy >= grid_size ||
            inflation_radius == 0.0 || distance >= inflation_radius) continue;
        const auto cost = static_cast<std::int8_t>(
          std::ceil(100.0 * (1.0 - distance / inflation_radius)));
        auto& value = grid.data[gy * grid_size + gx];
        value = std::max(value, cost);
      }
    }
  }
  return grid;
}

}
