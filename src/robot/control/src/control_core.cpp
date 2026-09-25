#include "control_core.hpp"

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger)
  : logger_(logger) {}

geometry_msgs::msg::Twist ControlCore::command(const nav_msgs::msg::Path& path,
  const nav_msgs::msg::Odometry& odometry, const PursuitOptions& options) const {
  geometry_msgs::msg::Twist result;
  if (path.header.frame_id.empty() || path.header.frame_id != odometry.header.frame_id)
    return result;
  const auto& pose = odometry.pose.pose;
  const auto& q = pose.orientation;
  const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
  if (!std::isfinite(norm) || norm < 1e-12) return result;
  const double yaw = std::atan2(2.0*(q.w*q.z + q.x*q.y),
                                q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z);
  std::vector<Point2> points;
  points.reserve(path.poses.size());
  for (const auto& waypoint : path.poses) {
    if (!waypoint.header.frame_id.empty() && waypoint.header.frame_id != path.header.frame_id)
      return result;
    points.push_back({waypoint.pose.position.x, waypoint.pose.position.y});
  }
  const auto drive = followPath(points, {pose.position.x, pose.position.y}, yaw, options);
  result.linear.x = drive.linear;
  result.angular.z = drive.angular;
  return result;
}

}
