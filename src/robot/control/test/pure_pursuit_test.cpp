#include "pure_pursuit.hpp"
#include <iostream>
#include <stdexcept>

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

int main() {
  robot::PursuitOptions options;
  const robot::Point2 origin{0.0, 0.0};
  auto command = robot::followPath({}, origin, 0.0, options);
  require(command.linear == 0.0 && command.angular == 0.0, "Empty path must stop");
  command = robot::followPath({{0,0}, {1,0}, {2,0}}, origin, 0.0, options);
  require(command.linear > 0 && command.angular == 0, "Straight path must drive forward");
  require(command.linear <= options.max_speed, "Speed limit exceeded");
  command = robot::followPath({{0,0}, {0,2}}, origin, 0.0, options);
  require(command.linear == 0 && command.angular > 0, "Left target should turn left first");
  command = robot::followPath({{0,0}, {0,-2}}, origin, 0.0, options);
  require(command.linear == 0 && command.angular < 0, "Right target should turn right first");
  command = robot::followPath({{0,0}, {2,0}}, {1.9,0}, 0.0, options);
  require(command.linear == 0 && command.angular == 0, "Near goal must stop");
  command = robot::followPath({{0,0}, {2,0}}, origin, 3.141592653589793, options);
  require(command.linear == 0 && std::abs(command.angular) <= options.max_turn_rate,
          "Target behind robot should rotate within rate limit");
  command = robot::followPath({{0,0}, {2,1}}, origin, 0.0, options);
  require(command.linear > 0 && command.angular > 0, "Gentle left curve has wrong sign");
  command = robot::followPath({{0,0}, {2,0}}, {1.6,0}, 0.0, options);
  require(command.linear > 0 && command.linear < options.max_speed, "Must slow near goal");
  command = robot::followPath({{0,0}, {NAN,1}}, origin, 0.0, options);
  require(command.linear == 0 && command.angular == 0, "Invalid coordinates must stop");

  // Follow a turning route with a simulated drive axle and a lidar 0.8 m ahead.
  // This checks convergence despite odometry tracking the offset sensor.
  const std::vector<robot::Point2> path{{0.8,0}, {0.5,0.5}, {0,1}, {0,2}};
  double x = 0.0, y = 0.0, yaw = 0.0;
  bool reached = false;
  for (int step = 0; step < 6000; ++step) {
    const robot::Point2 lidar{x+0.8*std::cos(yaw), y+0.8*std::sin(yaw)};
    if (std::hypot(lidar.x, lidar.y-2.0) <= options.goal_tolerance) {
      reached = true; break;
    }
    command = robot::followPath(path, lidar, yaw, options);
    require(std::abs(command.angular) <= options.max_turn_rate &&
            command.linear >= 0 && command.linear <= options.max_speed,
            "Simulated route exceeded command limits");
    x += command.linear*std::cos(yaw)*0.05;
    y += command.linear*std::sin(yaw)*0.05;
    yaw += command.angular*0.05;
  }
  require(reached, "Controller did not reach the goal with an offset lidar");
  std::cout << "Controller checks passed: steering, limits, stopping, and route convergence\n";
}
