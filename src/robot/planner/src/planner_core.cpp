#include "planner_core.hpp"
#include <cmath>

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger)
: logger_(logger) {}

nav_msgs::msg::Path PlannerCore::plan(const nav_msgs::msg::OccupancyGrid& map,
                                     const nav_msgs::msg::Odometry& odometry,
                                     const geometry_msgs::msg::PointStamped& goal,
                                     const GridSearchOptions& options) const {
  nav_msgs::msg::Path path;
  path.header = map.header;
  path.header.stamp = odometry.header.stamp;
  if (map.header.frame_id != odometry.header.frame_id ||
      map.header.frame_id != goal.header.frame_id ||
      !std::isfinite(map.info.resolution) || map.info.resolution <= 0.0 ||
      map.info.width == 0 || map.info.height == 0 ||
      map.info.width > 2000 || map.info.height > 2000 ||
      !std::isfinite(map.info.origin.position.x) ||
      !std::isfinite(map.info.origin.position.y) ||
      std::none_of(map.data.begin(), map.data.end(), [](std::int8_t v) { return v >= 0; }))
    return path;

  const auto& q = map.info.origin.orientation;
  const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
  if (!std::isfinite(norm) || norm < 1e-12) return path;
  const double yaw = std::atan2(2.0 * (q.w*q.z + q.x*q.y),
                                q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z);
  const double c = std::cos(yaw), s = std::sin(yaw);
  const auto to_cell = [&](double x, double y) -> int {
    if (!std::isfinite(x) || !std::isfinite(y)) return -1;
    const double dx = x - map.info.origin.position.x;
    const double dy = y - map.info.origin.position.y;
    const double gx = std::floor((c*dx + s*dy) / map.info.resolution);
    const double gy = std::floor((-s*dx + c*dy) / map.info.resolution);
    if (!(gx >= 0 && gy >= 0 && gx < map.info.width && gy < map.info.height)) return -1;
    return static_cast<int>(gy) * static_cast<int>(map.info.width) + static_cast<int>(gx);
  };
  GridSearchOptions search_options = options;
  search_options.resolution = map.info.resolution;
  const char* failure = nullptr;
  const auto cells = findGridPath(map.data, map.info.width, map.info.height,
    to_cell(odometry.pose.pose.position.x, odometry.pose.pose.position.y),
    to_cell(goal.point.x, goal.point.y), search_options, &failure);
  if (failure) {
    RCLCPP_WARN(logger_, "A* failed: %s; robot=(%.2f, %.2f), goal=(%.2f, %.2f)",
      failure, odometry.pose.pose.position.x, odometry.pose.pose.position.y,
      goal.point.x, goal.point.y);
  }
  for (int cell : cells) {
    const double x = (cell % map.info.width + 0.5) * map.info.resolution;
    const double y = (cell / map.info.width + 0.5) * map.info.resolution;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = map.info.origin.position.x + c*x - s*y;
    pose.pose.position.y = map.info.origin.position.y + s*x + c*y;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  // End at the requested point, not the cell center. Otherwise grid rounding
  // can make the controller stop just outside the planner's goal tolerance.
  // Both points are in the same checked cell; inflation includes a cell margin.
  if (!path.poses.empty()) {
    path.poses.back().pose.position.x = goal.point.x;
    path.poses.back().pose.position.y = goal.point.y;
  }
  for (std::size_t i = 0; i + 1 < path.poses.size(); ++i) {
    const double heading = std::atan2(
      path.poses[i+1].pose.position.y - path.poses[i].pose.position.y,
      path.poses[i+1].pose.position.x - path.poses[i].pose.position.x);
    path.poses[i].pose.orientation.z = std::sin(heading / 2.0);
    path.poses[i].pose.orientation.w = std::cos(heading / 2.0);
  }
  if (path.poses.size() > 1)
    path.poses.back().pose.orientation = path.poses[path.poses.size()-2].pose.orientation;
  return path;
}

}
