#ifndef PURE_PURSUIT_HPP_
#define PURE_PURSUIT_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace robot {
struct Point2 { double x; double y; };
struct DriveCommand { double linear = 0.0; double angular = 0.0; };
struct PursuitOptions {
  double lookahead = 0.8;
  double max_speed = 0.25;
  double max_turn_rate = 0.5;
  double goal_tolerance = 0.25;
};

inline DriveCommand followPath(const std::vector<Point2>& path,
                               Point2 position, double yaw,
                               const PursuitOptions& options) {
  if (path.empty() || !std::isfinite(position.x) || !std::isfinite(position.y) ||
      !std::isfinite(yaw)) return {};
  for (const auto& point : path)
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) return {};
  const double remaining = std::hypot(path.back().x-position.x, path.back().y-position.y);
  if (remaining <= options.goal_tolerance) return {};

  // Find the nearest waypoint, then look ahead along the route by arc length.
  std::size_t nearest = 0;
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < path.size(); ++i) {
    const double distance = std::hypot(path[i].x-position.x, path[i].y-position.y);
    if (distance < best) { best = distance; nearest = i; }
  }
  Point2 target = path[nearest];
  double budget = options.lookahead;
  for (std::size_t i = nearest+1; i < path.size(); ++i) {
    const double segment = std::hypot(path[i].x-target.x, path[i].y-target.y);
    if (segment >= budget && segment > 0.0) {
      const double ratio = budget / segment;
      target = {target.x + ratio*(path[i].x-target.x),
                target.y + ratio*(path[i].y-target.y)};
      break;
    }
    budget -= segment;
    target = path[i];
  }
  const double dx = target.x-position.x, dy = target.y-position.y;
  const double forward = std::cos(yaw)*dx + std::sin(yaw)*dy;
  const double lateral = -std::sin(yaw)*dx + std::cos(yaw)*dy;
  const double distance_squared = dx*dx + dy*dy;
  if (distance_squared < 1e-12) return {};
  const double angle = std::atan2(lateral, forward);
  if (std::abs(angle) > 0.7) {
    // Turn toward the route before driving forward.
    return {0.0, std::clamp(1.2*angle, -options.max_turn_rate, options.max_turn_rate)};
  }
  const double speed = std::min(options.max_speed, 0.5*remaining) *
                       std::max(0.2, std::cos(angle));
  const double curvature = 2.0*lateral / distance_squared;
  const double linear = std::min(speed,
    options.max_turn_rate / std::max(std::abs(curvature), 1e-9));
  return {linear, std::clamp(linear*curvature,
                           -options.max_turn_rate, options.max_turn_rate)};
}
}  // namespace robot
#endif
