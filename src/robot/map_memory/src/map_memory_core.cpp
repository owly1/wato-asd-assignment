#include "map_memory_core.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger)
  : logger_(logger) {}

void MapMemoryCore::initialize(double resolution, int grid_size, const std::string& frame) {
  map_ = nav_msgs::msg::OccupancyGrid();
  map_.header.frame_id = frame;
  map_.info.resolution = resolution;
  map_.info.width = grid_size;
  map_.info.height = grid_size;
  map_.info.origin.position.x = -grid_size * resolution / 2.0;
  map_.info.origin.position.y = -grid_size * resolution / 2.0;
  map_.info.origin.orientation.w = 1.0;
  map_.data.assign(static_cast<std::size_t>(grid_size) * grid_size, -1);
}

bool MapMemoryCore::merge(const nav_msgs::msg::OccupancyGrid& local,
                          const nav_msgs::msg::Odometry& odometry) {
  // The supplied odometry describes the lidar's pose in sim_world.
  if (local.header.frame_id != odometry.child_frame_id ||
      odometry.header.frame_id != map_.header.frame_id ||
      !std::isfinite(local.info.resolution) || local.info.resolution <= 0.0 ||
      local.data.size() != static_cast<std::size_t>(local.info.width) * local.info.height ||
      !std::isfinite(local.info.origin.position.x) ||
      !std::isfinite(local.info.origin.position.y) || map_.data.empty()) return false;

  const auto& pose = odometry.pose.pose;
  const auto yaw = [](const geometry_msgs::msg::Quaternion& q) {
    // This formula also handles a quaternion that is not perfectly normalized.
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z);
  };
  const auto valid_rotation = [](const geometry_msgs::msg::Quaternion& q) {
    const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    return std::isfinite(norm) && norm > 1e-12;
  };
  if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
      !valid_rotation(pose.orientation) ||
      !valid_rotation(local.info.origin.orientation)) return false;
  const double heading = yaw(pose.orientation);
  const double grid_heading = heading + yaw(local.info.origin.orientation);
  const double c = std::cos(grid_heading), s = std::sin(grid_heading);
  const double origin_x = pose.position.x +
    std::cos(heading) * local.info.origin.position.x -
    std::sin(heading) * local.info.origin.position.y;
  const double origin_y = pose.position.y +
    std::sin(heading) * local.info.origin.position.x +
    std::cos(heading) * local.info.origin.position.y;

  // Several rotated local cells can land on one global cell. Keep the highest
  // cost within this observation so free cells cannot erase a simultaneous hit.
  std::vector<std::int8_t> observation(map_.data.size(), -1);
  for (std::uint32_t y = 0; y < local.info.height; ++y) {
    for (std::uint32_t x = 0; x < local.info.width; ++x) {
      const auto value = local.data[static_cast<std::size_t>(y) * local.info.width + x];
      if (value < 0 || value > 100) continue;
      const double lx = (x + 0.5) * local.info.resolution;
      const double ly = (y + 0.5) * local.info.resolution;
      const double gx = std::floor((origin_x + c*lx - s*ly -
        map_.info.origin.position.x) / map_.info.resolution);
      const double gy = std::floor((origin_y + s*lx + c*ly -
        map_.info.origin.position.y) / map_.info.resolution);
      if (!(gx >= 0 && gy >= 0 && gx < map_.info.width && gy < map_.info.height)) continue;
      const auto index = static_cast<std::size_t>(gy) * map_.info.width +
                         static_cast<std::size_t>(gx);
      observation[index] = std::max(observation[index], value);
    }
  }
  for (std::size_t i = 0; i < observation.size(); ++i) {
    // Unknown cells leave memory untouched. New observations replace old ones.
    if (observation[i] >= 0) map_.data[i] = observation[i];
  }
  map_.header.stamp = local.header.stamp;
  if (map_.info.map_load_time.sec == 0 && map_.info.map_load_time.nanosec == 0)
    map_.info.map_load_time = local.header.stamp;
  return true;
}

}
